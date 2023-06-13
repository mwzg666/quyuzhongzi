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

// 取平均值
#define CT 60   
DWORD CpsArr[CT];
static BOOL Full = FALSE;
static BYTE ArrCnt = 0;
static float CpsHis = 0;

void InitArr()
{
    BYTE i;
    // 初始状态填15 个  1
    memset(CpsArr,0,sizeof(CpsArr));
    for (i=0;i<CT;i+=4)
    {
        CpsArr[i] = 1;
    }
    Full = TRUE;
    ArrCnt = CT;
    CpsHis = 15.0/CT;
}

// 将计数压入堆栈
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


// 计数平滑
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


// time :  保留最后计数的时间
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
            // 全部保留，就不做处理了
        }
    }
}

// 计数压入队列
void AddCounter(DWORD Cps)
{
    PushCps(Cps);   
}


void CpsFilter(DWORD Cps)
{
    BYTE i;
    
    // 剂量率3-5秒响应
    static DWORD BigCpsErr[5] = {0};
    static BYTE BigErrCnt = 0;
    BYTE BigCpsTime = 0;
    
    static DWORD SmCpsErr[5] = {0};
    static BYTE SmErrCnt = 0;

    // 大于2CPS(0.8uSv/h左右) 才判断陡升 -- 盖格管的最小剂量为 1uSv/h
    if ( (Cps>(CpsHis+3*sqrt(CpsHis)))  
          && (Cps >= 2) )
    {
        if (Cps >= 4.0)   
        {
            BigCpsTime = 3;  // 3秒响应
        }
        else        
        {
            BigCpsTime = 5;  // 5秒响应
        }
        

        SmErrCnt = 0;
        BigCpsErr[BigErrCnt++] = Cps;
        if (BigErrCnt >= BigCpsTime)
        {
            // 连续3个大计数则算陡升
            ClearCounter();
            for (i=0;i<BigErrCnt;i++)
            {
                AddCounter(BigCpsErr[i]);
            }
            
            BigErrCnt = 0; 
        }

        return;
    }
    // 小计数
    if (CpsHis > 9)  // >4uSv/h 
    {
        if (Cps < (CpsHis-3*sqrt(CpsHis))) 
        {
            BigErrCnt = 0;
            SmCpsErr[SmErrCnt++] = Cps;
            if (SmErrCnt == 3)
            {
                // 连续3个小计数则算陡降
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
    else if (CpsHis > 4)   // 2.5uSv/h 左右
    {
        if (Cps < (CpsHis-2*sqrt(CpsHis))) 
        {
            BigErrCnt = 0;
            SmCpsErr[SmErrCnt++] = Cps;
            if (SmErrCnt == 5)
            {
                // 连续5个小计数则算陡降
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


    // 不属于突变的情况就直接进入队列
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

// 每一秒被调用一次,输入参数，返回CPS，带出剂量率
float SmothCpsCalcRate(float RealCPS, float OlduSvh, float *NewuSvh)
{
    float CPS = 0;
    float dr = 0;
    
    CpsFilter((DWORD)RealCPS);    
    CPS = CounterPH(); //计数平滑
    
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