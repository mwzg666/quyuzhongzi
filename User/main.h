#ifndef _MAIN_H
#define _MAIN_H

#include "STC32G.h"
#include "stdio.h"
#include "intrins.h"
#include "string.h"
#include "stdlib.h"

//===============================变量定义=============================
typedef 	unsigned char	uint8_t;
typedef 	unsigned short	uint16_t;
typedef 	unsigned long	uint32_t;
typedef     unsigned int    uint;    
   
typedef 	unsigned char	BOOL;
#define     TRUE    1
#define     FALSE   0

#define     true    1
#define     false   0

typedef 	unsigned char	BYTE;
typedef 	unsigned short	WORD;
typedef 	unsigned long	DWORD;

typedef 	unsigned char	u8;
typedef 	unsigned short	u16;
typedef 	unsigned long	u32;   

#define bool BYTE

#define alt_u8   BYTE
#define alt_u16 WORD
#define alt_u32 DWORD 


#define MAIN_Fosc        11059200UL             // 11.0592M

#define SysTick          9216                   // 10ms    // 次/秒, 系统滴答频率, 在4000~16000之间

#define Timer1_Reload   (65536UL - SysTick)     //Timer 0 中断频率

#define HEAD   0x02
#define TAIL   0x03

#define TR600  0x20                             //探头型号    

//定义软件版本号
#define VERSION "V2.0.0"	                    //最长6个字节

#define POE_SEN                               // POE探头盖格管   

#define MIB_CST_DOSERATE_THRESHOLD_ALARM	2	
#define MIB_CST_DOSERATE_THRESHOLD_WARNING	2
   
#define DOSERATE_PRE_ALARM_BIT (1<<7)		//剂量率预警位
#define DOSERATE_ALARM_BIT (1<<6)			//剂量率报警位
#define DOSE_PRE_ALARM_BIT (1<<5)			//剂量当量预警位
#define DOSE_ALARM_BIT (1<<4)		        //剂量当量报警位
#define OVER_RANGE_BIT (1<<3)			    //过载报警位
#define LOW_BACK_BIT (1<<0)			        //低本底报警位
   

#define RUN_LED(x)  (x)?(P5 |= (1<<1)):(P5 &= ~(1<<1))    // 板载LED

#define Uart485_EN(x)(x)?(P3 |= (1<<3)):(P3 &= ~(1<<3))


//===============================结构体定义===========================
enum SNESOR_TYPE{LOW,HIGH,WHOLE}; 

#pragma pack(1)
//系统配置参数
typedef struct
{
	float Canshu1;	                // 低量程通道校准因子C		
	float Canshu2;	                // 高量程通道校准因子C 	
	char   yuzhi1[4];	            // 阈值
	char   yuzhi2[4];	            // 
	char   PingHuaShiJian[4];     // 平滑时间,发送的时候需要转换为4字节ASC字符发送
}SYS_PRAM;

//系统配置参数
typedef struct
{
    // < 80u
	float DI_A;	                    //低量程通道校准因子A		
	float DI_B;	                    // 低量程通道校准因子B 	
	float DI_C;	                    //低量程通道校准因子C	
	#ifdef POE_SEN
    float DI_D; 	                //低量程通道校准因子D

    // 80 - 1300
    //float MID_A;	                //中量程通道校准因子A		
	//float MID_B;	                //中量程通道校准因子B 	
	//float MID_C;	                //中量程通道校准因子C	
    //float MID_D;	                //中量程通道校准因子D
    
    float SUPER_GAO_A;              // 超高量程通道校准因子A +
    float SUPER_GAO_B;              // 超高量程通道校准因子B +
    float SUPER_GAO_C;              // 超高量程通道校准因子C +
    #endif
	float GAO_A;	                // 高量程通道校准因子A		
	float GAO_B;	                // 高量程通道校准因子B 
	float GAO_C;	                // 高量程通道校准因子C 	
	//float SUPER_GAO_A;              // 超高量程通道校准因子A +
    //float SUPER_GAO_B;              // 超高量程通道校准因子B +
    //float SUPER_GAO_C;              // 超高量程通道校准因子C +
	#ifdef POE_SEN
    //float GAO_D;	                //高量程通道校准因子D
    #endif
    alt_u32 DET_THR_1;              //探测器阈值1
    alt_u32 DET_THR_2;              //探测器阈值2
    alt_u32 DET_THR_3;              //探测器阈值3
    alt_u32 DET_TIME;               //探测器时间
    alt_u32 HV_THR;                 //高压阈值阈值
}SYS_JIAOZHUNYINZI;

//报警信息
typedef struct
{
    float  DoseRatePreAlarm;    	//剂量率一级报警(uSv/h)
    float  DoseRateAlarm;  		    //剂量率二级报警(uSv/h)
    float  DosePreAlarm;  		    //累计剂量预警阀值
    float  DoseAlarm;  			    //累计剂量报警阀值
    float  InvalidAlarm1;           //失效一级报警（过载）+
    float  InvalidAlarm2;           //失效二级报警（过载）+
}SYS_ALARM;

//计数结构体    
typedef struct
{
    float  DoseRate;
    float  Dose;                    //累积剂量
    
    float WAITINGFORUSE;            //备用
    DWORD PULSE1;                   //脉冲1
    DWORD PULSE2;                   //脉冲2

    /*
    bit7   剂量率一级报警、
    bit6位 剂量率二级报警、
    bit5位 累计剂量率一级报警、
    bit4位 累计剂量率二级报警、
    bit3位 过载报警、       
    bit2位 失效一级报警、
    bit1位 失效二级报警、
    bit0位 低本底故障
    */
    uint8_t State;	                //报警状态
    //INF_SOUND_CERTAIN CERTAIN_KEY;//按钮报
    DWORD PULSE3;                   //脉冲3 -- 临界探头专用
    
    //float  DoseRate;  
    //float  Dose;
	//uint8_t State;	            //状态字‘１’为有效，‘０’为无效，７—2位分别表示剂量率一级报警、剂量率二级报警、累计剂量率一级报警、累计剂量率二级报警、过载报警、低本底故障，１、０位未定义
}STU_DOSERATE;

//脉冲计数结构体    
typedef struct
{
    float  CPS1;                    //发送的时候要转为6字节asc编码发送
    float  CPS2;	                //发送的时候要转为6字节asc编码发送
	uint8_t State;	                //状态字‘１’为有效，‘０’为无效，７—2位分别表示剂量率一级报警、剂量率二级报警、累计剂量率一级报警、累计剂量率二级报警、过载报警、低本底故障，１、０位未定义
}STU_CPS;
#pragma pack()


typedef struct
{
    uint16_t   Sign;                // 0x4142
	uint8_t    address;             //探头类型
	char       VerSion[6];	        //版本号
	SYS_PRAM   s_SysParam;
	SYS_JIAOZHUNYINZI s_Jiaozhun;   //校准因子
	SYS_ALARM  s_Alarm;
    float      CoCsRate;   
	WORD   ParaCheckSum;            //参数校验和
}LP_PARAM;



typedef struct
{
  	LP_PARAM	stParam;
	uint8_t SensorType;
	uint8_t keydownFlag;            //按键按下标志
	uint8_t keyValue;               //按键键值
	uint32_t keydownTime;           //按键按下计数时间*10ms
	uint8_t isCanReadSensor;
	STU_DOSERATE s_DoseMSG;
	STU_CPS s_CPS;
	uint8_t range;	                //设置的量程
	uint8_t RealRange;              //实际使用的量程
	uint8_t isSwitching;	        // >0正在切换档位，
	uint16_t DoseRatePreAlarmCnt;   //剂量率预警次数，连续2次报警能报警
	uint16_t DoseRateAlarmCnt;      //剂量率报警次数，连续2次报警能报警
	uint32_t LChannelNoCountTime;   //低通道无计数时间
}LP_SYSTEM_STTAE;


//===============================变量声明=============================
extern LP_SYSTEM_STTAE SysRunState;


//===============================函数声明=============================
void Error();
void InitParam();
void delay_ms(WORD ms);
void delay_us(BYTE us);
void TimerTask();
void Rs485Hnd();

#ifdef __cplusplus
}
#endif

#endif





