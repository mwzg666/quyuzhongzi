#include "Sensor.h"
#include "CalcDoseRate.h"


extern LP_SYSTEM_STTAE SysRunState;

extern unsigned long Get_HV_Vol();

#ifdef POE_SEN
#define USE_MID_FIX  80         // 分段拟合
#define USE_LOW_USV 800		    //1mSv
#define USE_HIGH_USV 1200		//1.2mSv
#else
#define USE_LOW_USV   1000		//1mSv
#define USE_HIGH_USV 2000		//2mSv
#endif

extern LP_SYSTEM_STTAE SysRunState;
extern u32 GetCounter(void);

u8 CPSFlag = 0;

static float LowSmothCPS;
static float LowNOSmothCPS;
static float LowSumCPS;

float parama,paramb,paramc,paramd;

void SensorInit(void)
{
	InitArr();
        
	parama = SysRunState.stParam.s_Jiaozhun.DI_A;                   //0.63;
	paramb = SysRunState.stParam.s_Jiaozhun.DI_B;                   //0.00019;
	paramc = SysRunState.stParam.s_Jiaozhun.DI_C;                   //SysRunState.stParam.s_SysParam.Canshu1;
	//memset((void*)&SysRunState.s_DoseMSG,0,sizeof(STU_DOSERATE));
	SysRunState.s_DoseMSG.DoseRate = 0;
	SysRunState.s_DoseMSG.State = 0;
	
	SysRunState.SensorType = LOW;           //GetSensorType();
	RefreshParam();
	
	SensorMeasureBegin();                   //开始测量 
}

void RefreshParam(void)
{
  	parama = SysRunState.stParam.s_Jiaozhun.DI_A;       //0.63;
	paramb = SysRunState.stParam.s_Jiaozhun.DI_B;       //0.00019;
	paramc = SysRunState.stParam.s_Jiaozhun.DI_C;
	#ifdef POE_SEN
    paramd = SysRunState.stParam.s_Jiaozhun.DI_D;
    #endif
}

void SensorMeasureBegin(void)
{ 
	GetCounter();
}

u8 GetSensorType(void)
{
	u8 temp = (P7 >> 5)&0x03;
	if(temp == 0)
	{
	  	return LOW;
	}
//	else if(temp == 3)
//	{
//	  	return HIGH;
//	}
//	else if(temp == 3)
//	{
//	  	return WHOLE;
//	}
	return LOW;
}

#ifdef POE_SEN
void UseLowParam(float dr)
{
    if (dr< 80)
    {
        parama = SysRunState.stParam.s_Jiaozhun.DI_A;
		paramb = SysRunState.stParam.s_Jiaozhun.DI_B;
		paramc = SysRunState.stParam.s_Jiaozhun.DI_C;
        paramd = SysRunState.stParam.s_Jiaozhun.DI_D;
    }
//    else
//    {
//        parama = SysRunState.stParam.s_Jiaozhun.MID_A;
//		paramb = SysRunState.stParam.s_Jiaozhun.MID_B;
//		paramc = SysRunState.stParam.s_Jiaozhun.MID_C;
//        paramd = SysRunState.stParam.s_Jiaozhun.MID_D;
//    }
}
#endif

//========================================================================
// 函数名称: void CaptureSensorPluseCounter(void)
// 函数功能: 捕获传感器的脉冲计数
// 入口参数: @无
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================
void CaptureSensorPluseCounter(void)
{
	float uSvh;
    float nSmothCPS;
	LowSumCPS = GetCounter();
	if(LowSumCPS == 0)
	{
	  	SysRunState.LChannelNoCountTime++;
	}
	else
	{
	  	SysRunState.LChannelNoCountTime = 0;
	}
    
	SysRunState.s_CPS.CPS1 = LowSumCPS;
	SysRunState.s_CPS.CPS2 = 0;             //LowSumCPS;
	nSmothCPS = SmothCpsCalcRate(LowSumCPS, SysRunState.s_DoseMSG.DoseRate, &uSvh);
    
      
//	if(uSvh > USE_HIGH_USV)//&&(SysRunState.RealRange == 'L')&&(SysRunState.range =='A'))
//	{
//		parama = SysRunState.stParam.s_Jiaozhun.GAO_A;
//		paramb = SysRunState.stParam.s_Jiaozhun.GAO_B;
//		paramc = SysRunState.stParam.s_Jiaozhun.GAO_C;
//        #ifdef POE_SEN
//        paramd = SysRunState.stParam.s_Jiaozhun.GAO_D;
//        #endif
//        ClearCounter();
//	}
//	if(uSvh < USE_LOW_USV)//&&(SysRunState.RealRange == 'H')&&(SysRunState.range =='A'))
//	{
//		parama = SysRunState.stParam.s_Jiaozhun.DI_A;
//		paramb = SysRunState.stParam.s_Jiaozhun.DI_B;
//		paramc = SysRunState.stParam.s_Jiaozhun.DI_C;
//        #ifdef POE_SEN
//        UseLowParam(uSvh);
//        #endif
//        ClearCounter();
//	}
    parama = SysRunState.stParam.s_Jiaozhun.DI_A;
	paramb = SysRunState.stParam.s_Jiaozhun.DI_B;
	paramc = SysRunState.stParam.s_Jiaozhun.DI_C;
    #ifdef POE_SEN
    UseLowParam(uSvh);
    #endif
    ClearCounter();


    #ifdef POE_SEN   
    UseLowParam(uSvh);
    SysRunState.s_DoseMSG.DoseRate *= SysRunState.stParam.s_SysParam.Canshu1;
    //SysRunState.s_DoseMSG.DoseRate *= SysRunState.stParam.s_SysParam.Canshu2;
    #else
	SysRunState.s_DoseMSG.DoseRate  = uSvh;	
    #endif

	CalcAlarmState(&SysRunState);

	SysRunState.s_DoseMSG.Dose += SysRunState.s_DoseMSG.DoseRate/3600.0f;
}

float Get_Low_Counter(void)
{
	return LowNOSmothCPS;
}

float Get_Low_Smooth_Counter(void)
{
	return LowSmothCPS;
}

//========================================================================
// 函数名称: u16 CalcAlarmState(LP_SYSTEM_STTAE *me)
// 函数功能: 监测报警状态
// 入口参数: @*me：数据
// 函数返回: 报警状态
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================
u16 CalcAlarmState(LP_SYSTEM_STTAE *me)
{
    #if 1
    /* 剂量当量报警检查 */	
	if ((me->s_DoseMSG.Dose >= me->stParam.s_Alarm.DoseAlarm)
        &&(me->stParam.s_Alarm.DoseAlarm > 0)) 
	{ 
		me->s_DoseMSG.State |= DOSE_ALARM_BIT;
    } 
	else
	{
	  	me->s_DoseMSG.State &= ~DOSE_ALARM_BIT;
	}
    
	/* 剂量当量预警检查 */	
	if( (me->s_DoseMSG.Dose >= me->stParam.s_Alarm.DosePreAlarm)
        &&(me->stParam.s_Alarm.DosePreAlarm > 0)
        &&(me->s_DoseMSG.Dose < me->stParam.s_Alarm.DoseAlarm) ) 
	{ 
		me->s_DoseMSG.State |= DOSE_PRE_ALARM_BIT;
    } 
	else
	{
	  	me->s_DoseMSG.State &= ~DOSE_PRE_ALARM_BIT;
	}
    #endif	
    
	/* 剂量当量率报警检查 */	
	if ( (me->s_DoseMSG.DoseRate >= me->stParam.s_Alarm.DoseRateAlarm)
        &&(me->stParam.s_Alarm.DoseRateAlarm > 0) ) 
	{ 
		if((++me->DoseRateAlarmCnt) >= MIB_CST_DOSERATE_THRESHOLD_ALARM) 
        {   
            //连续两次报警则认为报警
			me->s_DoseMSG.State |= DOSERATE_ALARM_BIT;
		}
    } 
    else 
    {
		me->DoseRateAlarmCnt= 0x0;
		me->s_DoseMSG.State &= ~DOSERATE_ALARM_BIT;
	}
	
	/* 剂量当量率预警检查 */	
	if ( (me->s_DoseMSG.DoseRate >= me->stParam.s_Alarm.DoseRatePreAlarm)
        &&(me->s_DoseMSG.DoseRate < me->stParam.s_Alarm.DoseRateAlarm) )
	{ 
		if((++me->DoseRatePreAlarmCnt) >= MIB_CST_DOSERATE_THRESHOLD_WARNING) 
        {
            //连续两次报警则认为报警
			me->s_DoseMSG.State |= DOSERATE_PRE_ALARM_BIT;
		}
    } 
    else 
    {
		me->DoseRatePreAlarmCnt= 0x0;
		me->s_DoseMSG.State &= ~DOSERATE_PRE_ALARM_BIT;
	}

	
	if ((me->s_DoseMSG.Dose >= 10.0*1000*1000)||(me->s_DoseMSG.DoseRate >= 10.0*1000*1000))
	{
	  	me->s_DoseMSG.State |= OVER_RANGE_BIT;
		me->s_DoseMSG.DoseRate = 10.0*1000*1000;
	}
	else
	{
	  	me->s_DoseMSG.State &= ~OVER_RANGE_BIT;
	}
//	if((SysRunState.LChannelNoCountTime > 60)||(SysRunState.LChannelNoCountTime > 600))//1分钟无数据,探测器异常
//	{
//	  	me->s_DoseMSG.State |= LOW_BACK_BIT;
//	}
	//else
	//{
	  	//me->s_DoseMSG.State &= ~LOW_BACK_BIT;
	//}
	if(me->s_CPS.State != me->s_DoseMSG.State)//报警信息发生变化则，重启蜂鸣器报警
	{
	  	SysRunState.keyValue = 0;
	}
	me->s_CPS.State = me->s_DoseMSG.State;
	return true;
}
