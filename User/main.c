#include "Sensor.h"
#include "mcp4725.h"
#include "system.h"
#include "main.h"
#include "CMD.h"
#include "uart.h"
#include "i2c.h"
#include "flash.h"
#include "CalcDoseRate.h"

LP_SYSTEM_STTAE SysRunState={0};

//========================================================================
// ��������: void InitParam()
// ��������: ��ʼ����������
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void InitParam()
{
	memset((void*)&SysRunState.stParam,0,sizeof(LP_PARAM));
    SysRunState.stParam.Sign     = 0x4142;
	SysRunState.stParam.address = 1;                         //̽ͷ��ַ
	
	SysRunState.stParam.s_Alarm.DosePreAlarm      = 300;     //300uSv
	SysRunState.stParam.s_Alarm.DoseAlarm         = 400;     //400uSv
	SysRunState.stParam.s_Alarm.DoseRatePreAlarm = 300;      //300uSv/h
	SysRunState.stParam.s_Alarm.DoseRateAlarm     = 400;     //400uSv/h
    SysRunState.stParam.s_Alarm.InvalidAlarm1     = 0;      
	SysRunState.stParam.s_Alarm.InvalidAlarm2     = 0; 
    
	SysRunState.stParam.s_SysParam.Canshu1 = 1;
	SysRunState.stParam.s_SysParam.Canshu2 = 1;

    #ifdef POE_SEN
    // ��������Co60��ϣ���������Ҫ��Co/Cs��
    SysRunState.stParam.s_Jiaozhun.DI_A = -2.8421709430404E-14;//-0.119135107108512;    
	SysRunState.stParam.s_Jiaozhun.DI_B = 0.23314990868541;//0.406688356856999; 
	SysRunState.stParam.s_Jiaozhun.DI_C = 0.000122186491801752;//-0.000795882360575705;
    SysRunState.stParam.s_Jiaozhun.DI_D = -1.48593654542308E-07;//8.30493670182536E-06;

    //SysRunState.stParam.s_Jiaozhun.MID_A = 25.2437957332788;    
	//SysRunState.stParam.s_Jiaozhun.MID_B = 0.352560222117065; 
	//SysRunState.stParam.s_Jiaozhun.MID_C = 6.84378068779986E-05;
    //SysRunState.stParam.s_Jiaozhun.MID_D = 0;
    
    // Cs137 > 1.2m
	//SysRunState.stParam.s_Jiaozhun.GAO_A = -32435.201473664; 
	//SysRunState.stParam.s_Jiaozhun.GAO_B = 35.1613941353375; 
	//SysRunState.stParam.s_Jiaozhun.GAO_C = -6.81597532235814E-4;  
    //SysRunState.stParam.s_Jiaozhun.GAO_D = 2.46287003496529E-08;
    SysRunState.stParam.s_Jiaozhun.GAO_A = 0;                    // 31.92;
	SysRunState.stParam.s_Jiaozhun.GAO_B = 0;                    // 0.000018;
	SysRunState.stParam.s_Jiaozhun.GAO_C = 0;
    SysRunState.stParam.s_Jiaozhun.SUPER_GAO_A = 0;
    SysRunState.stParam.s_Jiaozhun.SUPER_GAO_B = 0;
    SysRunState.stParam.s_Jiaozhun.SUPER_GAO_C = 0;
    #else
	SysRunState.stParam.s_Jiaozhun.DI_A = 0.5593;                // 0.56;
	SysRunState.stParam.s_Jiaozhun.DI_B = 0.000119;              // 0.00017;
	SysRunState.stParam.s_Jiaozhun.DI_C = SysRunState.stParam.s_SysParam.Canshu1;
	SysRunState.stParam.s_Jiaozhun.GAO_A = 0;                    // 31.92;
	SysRunState.stParam.s_Jiaozhun.GAO_B = 0;                    // 0.000018;
	SysRunState.stParam.s_Jiaozhun.GAO_C = SysRunState.stParam.s_SysParam.Canshu2;
    SysRunState.stParam.s_Jiaozhun.SUPER_GAO_A = 0;
    SysRunState.stParam.s_Jiaozhun.SUPER_GAO_B = 0;
    SysRunState.stParam.s_Jiaozhun.SUPER_GAO_C = 0;
    #endif
    
	sprintf((char*)SysRunState.stParam.s_SysParam.yuzhi1,"150");

    SysRunState.stParam.CoCsRate = 1.19;
    
    WritePara();                    //�������
}

//========================================================================
// ��������: void delay_ms(WORD ms)  
// ��������: ������ʱ����
// ��ڲ���: @WORD ms����ʱ���ٺ���
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void delay_ms(WORD ms)              
{
    WORD t = 1000;
    while(ms--)
    {
        for (t=0;t<1000;t++) ;
    }
}


void delay_us(BYTE us)
{
    while(us--)
    {
        ;
    }
}

//========================================================================
// ��������: void IoInit()
// ��������: ��Ƭ��I/O�ڳ�ʼ��
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void IoInit()
{
    EAXFR = 1;
    WTST = 0;                       //���ó���ָ����ʱ��������ֵΪ0�ɽ�CPUִ��ָ����ٶ�����Ϊ���

    P0M1 = 0x10;   P0M0 = 0x00;     //����Ϊ׼˫���
    P1M1 = 0x00;   P1M0 = 0x00;     //����Ϊ׼˫���
    P2M1 = 0x00;   P2M0 = 0x00;     //����Ϊ׼˫���
    P3M1 = 0x00;   P3M0 |= (1<<3) ;     //P3.3����Ϊ�������
    P4M1 = 0x00;   P4M0 = 0x00;     //����Ϊ׼˫���
    P5M1 = 0x00;   P5M0 = 0x00;     //����Ϊ׼˫���
    P6M1 = 0x00;   P6M0 = 0x00;     //����Ϊ׼˫���
    P7M1 = 0x00;   P7M0 = 0x00;     //����Ϊ׼˫���
}

//========================================================================
// ��������: void TimerTask()
// ��������: ��ʱ����ͨ����ʱ��0��ʱ10ms�������������
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void TimerTask()
{
    u16 delta = 0;
    static u16 Time1s = 0;
    static u16 CpsTime = 0;
    if(Timer0Cnt)
    {
        delta = Timer0Cnt * 10;
        Timer0Cnt = 0;
        if(RX1_Cnt>0)
        {
            Rx1_Timer += delta;
        }
        Time1s += delta;
        if(Time1s >= 1000)                      //100*10=1000ms
        {
            CpsTime += Time1s;
            Time1s = 0;
            SysRunState.isCanReadSensor = 1;

            PluseCnt = ((u32)LowOverFlowFlag*65536) | (u32)((u32)T3H*256+T3L) ;
            T3R = 0; 
            T3H = 0;
            T3L = 0;
            T3R = 1;            

            PlusePortCnt = 0;
            LowOverFlowFlag = 0;
        }
        
        if(CpsTime >= 10000)
        {
            CpsTime = 0;
            CPSFlag = 1;
        }
    }
}

//========================================================================
// ��������: void Rs485Hnd()
// ��������: ͨ��RS485����λ������
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void Rs485Hnd()
{
    if(Rx1_Timer > 20)                  //���ڳ�ʱ20ms
    {
        Rx1_Timer = 0;
        DataPro(RX1_Buffer,RX1_Cnt);
        ClearRs485Buf();
    }
}


void Error()
{
    while(1)
    {
        
        RUN_LED(1);
        delay_ms(200);
        RUN_LED(0);
        delay_ms(200);
    }
}

int main(void)
{ 
    SysInit();
	IoInit();
	checkApp();
	Timer0_Init();
	Timer3_Init();
	SensorInit();
    Uart1_Init();
    ClearRs485Buf();
    
	delay_ms(1000);    
	GetPara(&SysRunState.stParam);
    memcpy(SysRunState.stParam.VerSion,VERSION,6);        
    MCP4725_OutVol(MCP4725_S1_ADDR,SysRunState.stParam.s_SysParam.yuzhi1);
	EA = 1;  
    
	ClearCounter();
	SensorMeasureBegin();                       //��ʼ����
	delay_ms(100);
    while(1)
    { 
        TimerTask();
        if(SysRunState.isCanReadSensor == 1)
		{
		  	SysRunState.isCanReadSensor = 0;
			CaptureSensorPluseCounter();        //����ǰ�������
		}        
        Rs485Hnd(); 
    }
}

