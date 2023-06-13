#include <stdlib.h>
#include <math.h>
#include <main.h>
#include "system.h"
//#include "CommDataDefine.h"
#include "CalcDoseRate.h"
#include "sensor.h"

extern float parama,paramb,paramc,paramd;

static void CpsFilter(DWORD Cps);
static float CounterPH();
static float CpsToUsv_h(float count);

// ȡƽ��ֵ
#define CT 60   
DWORD CpsArr[CT];
static BOOL Full = FALSE;
static BYTE ArrCnt = 0;
static float CpsHis = 0;

void InitArr()
{
    BYTE i;
    // ��ʼ״̬��15 ��  1
    memset(CpsArr,0,sizeof(CpsArr));
    for (i=0;i<CT;i+=4)
    {
        CpsArr[i] = 1;
    }
    Full = TRUE;
    ArrCnt = CT;
    CpsHis = 15.0/CT;
}

// ������ѹ���ջ
void PushCps(DWORD cps)
{
    BYTE i;
    if (ArrCnt >= CT)
    {
        Full = TRUE;
        for (i=1;i<CT;i++)
        {
            CpsArr[i-1] = CpsArr[i];
        }
        CpsArr[CT-1] = cps;
    }
    else
    {
        CpsArr[ArrCnt++] = cps;
    }
}

void ClearCounter()
{
    memset(CpsArr,0,sizeof(CpsArr));
    Full = FALSE;
    ArrCnt = 0;
    CpsHis = 0;
}


// ����ƽ��
float CounterPH()
{
    #define BK_CPS 0.01
    float Val = 0.0;
    BYTE i;
    
    Val = 0.0;
    if (Full)
    {
        for (i=0;i<CT;i++)
        {
            Val += CpsArr[i];
        }
        Val = Val/CT;
    }
    else
    {
        if (ArrCnt == 0)
        {
            CpsHis = BK_CPS;
            return -1;
        }
        else
        {
            for (i=0;i<ArrCnt;i++)
            {
                Val += CpsArr[i];
            }
            Val = Val/ArrCnt;
        }
    }

    if (Val < BK_CPS)
    {
        Val = BK_CPS;
    }
    
    CpsHis = Val;
    return Val;
}


float GetHis(BYTE time)
{
    BYTE i;
    float ret = 0.0;
    if (Full)
    {
        for (i=CT-time;i<CT;i++)
        {
            ret += CpsArr[i];
        }

        return (ret/time);
    }
    else
    {
        if (time > ArrCnt)
        {
            for (i=0;i<ArrCnt;i++)
            {
                ret += CpsArr[i];
            }

            return (ret/ArrCnt);
        }
        else
        {
            for (i=ArrCnt-time;i<ArrCnt;i++)
            {
                ret += CpsArr[i];
            }

            return (ret/time);
        }
    }

    //return 0.0;
}


// time :  ������������ʱ��
void ResvCps(BYTE time)
{
    BYTE i;
    if (Full)
    {
        for (i=0;i<CT;i++)
        {
            if (i<time)
            {
                CpsArr[i] = CpsArr[CT-time+i];
            }
            else
            {
                CpsArr[i] = 0;
            }
        }
        ArrCnt = time;
        Full = FALSE;
    }
    else
    {
        if (time < ArrCnt)
        {
            for (i=0;i<ArrCnt;i++)
            {
                if (i<time)
                {
                    CpsArr[i] = CpsArr[ArrCnt-time+i];
                }
                else
                {
                    CpsArr[i] = 0;
                }
            }

            ArrCnt = time;
        }
        else
        {
            // ȫ���������Ͳ���������
        }
    }
}

// ����ѹ�����
void AddCounter(DWORD Cps)
{
    PushCps(Cps);   
}


void CpsFilter(DWORD Cps)
{
    BYTE i;
    
    // ������3-5����Ӧ
    static DWORD BigCpsErr[5] = {0};
    static BYTE BigErrCnt = 0;
    BYTE BigCpsTime = 0;
    
    static DWORD SmCpsErr[5] = {0};
    static BYTE SmErrCnt = 0;

    // ����2CPS(0.8uSv/h����) ���ж϶��� -- �Ǹ�ܵ���С����Ϊ 1uSv/h
    if ( (Cps>(CpsHis+3*sqrt(CpsHis)))  
          && (Cps >= 2) )
    {
        if (Cps >= 4.0)   
        {
            BigCpsTime = 3;  // 3����Ӧ
        }
        else        
        {
            BigCpsTime = 5;  // 5����Ӧ
        }
        

        SmErrCnt = 0;
        BigCpsErr[BigErrCnt++] = Cps;
        if (BigErrCnt >= BigCpsTime)
        {
            // ����3����������㶸��
            ClearCounter();
            for (i=0;i<BigErrCnt;i++)
            {
                AddCounter(BigCpsErr[i]);
            }
            
            BigErrCnt = 0; 
        }

        return;
    }
    // С����
    if (CpsHis > 9)  // >4uSv/h 
    {
        if (Cps < (CpsHis-3*sqrt(CpsHis))) 
        {
            BigErrCnt = 0;
            SmCpsErr[SmErrCnt++] = Cps;
            if (SmErrCnt == 3)
            {
                // ����3��С�������㶸��
                ClearCounter();
                for (i=0;i<SmErrCnt;i++)
                {
                    AddCounter(SmCpsErr[i]);
                }
                SmErrCnt = 0;
            }
            return;
        }
    }
    else if (CpsHis > 4)   // 2.5uSv/h ����
    {
        if (Cps < (CpsHis-2*sqrt(CpsHis))) 
        {
            BigErrCnt = 0;
            SmCpsErr[SmErrCnt++] = Cps;
            if (SmErrCnt == 5)
            {
                // ����5��С�������㶸��
                ClearCounter();
                for (i=0;i<SmErrCnt;i++)
                {
                    AddCounter(SmCpsErr[i]);
                }
                SmErrCnt = 0;
            }
            return;
        }
    }


    // ������ͻ��������ֱ�ӽ������
    AddCounter(Cps);
    BigErrCnt = 0;
    SmErrCnt = 0;
}


float CpsToUsv_h(float count)
{   
    
    float ret;

    #ifdef POE_SEN
    float c = count;
    float c2 = c*c;
    float c3 = c2*c;
    ret = parama + paramb*c + paramc*c2 + paramd*c3;
    #else

    if(CPSFlag)
    {
        CPSFlag = 0;
        if(count >= 1)
        {
            count -= 1;
        }       
    }
//    if(count < 0.02)
//    {
//        ret = 0;
//        //Error();
//    }
//    else
//    {
        ret = paramc * parama * count * exp(paramb*count);
    //}
    //printf("parama = %f\r\n paramb = %f\r\n paramc = %f\r\n",parama,paramb,paramc);
    #endif
    
    return ret;
}

// ÿһ�뱻����һ��,�������������CPS������������
float SmothCpsCalcRate(float RealCPS, float OlduSvh, float *NewuSvh)
{
    float CPS = 0;
    float dr = 0;
    
    CpsFilter((DWORD)RealCPS);    
    CPS = CounterPH(); //����ƽ��
    
    if (CPS != -1)
    {
        //*NewuSvh = CpsToUsv_h(CPS);   // uSv/h
        dr =  CpsToUsv_h(CPS)*0.24;
        if(dr < 0.15)
        {
            *NewuSvh = 0;
        }
        else
        {
            *NewuSvh = dr;
        }
    }
    else
    {
        *NewuSvh = OlduSvh;
    }
    return CPS;
}