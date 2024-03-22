#ifndef _MAIN_H
#define _MAIN_H

#include "STC32G.h"
#include "stdio.h"
#include "intrins.h"
#include "string.h"
#include "stdlib.h"

//===============================��������=============================
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

#define SysTick          9216                   // 10ms    // ��/��, ϵͳ�δ�Ƶ��, ��4000~16000֮��

#define Timer1_Reload   (65536UL - SysTick)     //Timer 0 �ж�Ƶ��

#define HEAD   0x02
#define TAIL   0x03

#define TR600  0x20                             //̽ͷ�ͺ�    

//��������汾��
#define VERSION "V2.0.0"	                    //�6���ֽ�

#define POE_SEN                               // POE̽ͷ�Ǹ��   

#define MIB_CST_DOSERATE_THRESHOLD_ALARM	2	
#define MIB_CST_DOSERATE_THRESHOLD_WARNING	2
   
#define DOSERATE_PRE_ALARM_BIT (1<<7)		//������Ԥ��λ
#define DOSERATE_ALARM_BIT (1<<6)			//�����ʱ���λ
#define DOSE_PRE_ALARM_BIT (1<<5)			//��������Ԥ��λ
#define DOSE_ALARM_BIT (1<<4)		        //������������λ
#define OVER_RANGE_BIT (1<<3)			    //���ر���λ
#define LOW_BACK_BIT (1<<0)			        //�ͱ��ױ���λ
   

#define RUN_LED(x)  (x)?(P5 |= (1<<1)):(P5 &= ~(1<<1))    // ����LED

#define Uart485_EN(x)(x)?(P3 |= (1<<3)):(P3 &= ~(1<<3))


//===============================�ṹ�嶨��===========================
enum SNESOR_TYPE{LOW,HIGH,WHOLE}; 

#pragma pack(1)
//ϵͳ���ò���
typedef struct
{
	float Canshu1;	                // ������ͨ��У׼����C		
	float Canshu2;	                // ������ͨ��У׼����C 	
	char   yuzhi1[4];	            // ��ֵ
	char   yuzhi2[4];	            // 
	char   PingHuaShiJian[4];     // ƽ��ʱ��,���͵�ʱ����Ҫת��Ϊ4�ֽ�ASC�ַ�����
}SYS_PRAM;

//ϵͳ���ò���
typedef struct
{
    // < 80u
	float DI_A;	                    //������ͨ��У׼����A		
	float DI_B;	                    // ������ͨ��У׼����B 	
	float DI_C;	                    //������ͨ��У׼����C	
	#ifdef POE_SEN
    float DI_D; 	                //������ͨ��У׼����D

    // 80 - 1300
    //float MID_A;	                //������ͨ��У׼����A		
	//float MID_B;	                //������ͨ��У׼����B 	
	//float MID_C;	                //������ͨ��У׼����C	
    //float MID_D;	                //������ͨ��У׼����D
    
    float SUPER_GAO_A;              // ��������ͨ��У׼����A +
    float SUPER_GAO_B;              // ��������ͨ��У׼����B +
    float SUPER_GAO_C;              // ��������ͨ��У׼����C +
    #endif
	float GAO_A;	                // ������ͨ��У׼����A		
	float GAO_B;	                // ������ͨ��У׼����B 
	float GAO_C;	                // ������ͨ��У׼����C 	
	//float SUPER_GAO_A;              // ��������ͨ��У׼����A +
    //float SUPER_GAO_B;              // ��������ͨ��У׼����B +
    //float SUPER_GAO_C;              // ��������ͨ��У׼����C +
	#ifdef POE_SEN
    //float GAO_D;	                //������ͨ��У׼����D
    #endif
    alt_u32 DET_THR_1;              //̽������ֵ1
    alt_u32 DET_THR_2;              //̽������ֵ2
    alt_u32 DET_THR_3;              //̽������ֵ3
    alt_u32 DET_TIME;               //̽����ʱ��
    alt_u32 HV_THR;                 //��ѹ��ֵ��ֵ
}SYS_JIAOZHUNYINZI;

//������Ϣ
typedef struct
{
    float  DoseRatePreAlarm;    	//������һ������(uSv/h)
    float  DoseRateAlarm;  		    //�����ʶ�������(uSv/h)
    float  DosePreAlarm;  		    //�ۼƼ���Ԥ����ֵ
    float  DoseAlarm;  			    //�ۼƼ���������ֵ
    float  InvalidAlarm1;           //ʧЧһ�����������أ�+
    float  InvalidAlarm2;           //ʧЧ�������������أ�+
}SYS_ALARM;

//�����ṹ��    
typedef struct
{
    float  DoseRate;
    float  Dose;                    //�ۻ�����
    
    float WAITINGFORUSE;            //����
    DWORD PULSE1;                   //����1
    DWORD PULSE2;                   //����2

    /*
    bit7   ������һ��������
    bit6λ �����ʶ���������
    bit5λ �ۼƼ�����һ��������
    bit4λ �ۼƼ����ʶ���������
    bit3λ ���ر�����       
    bit2λ ʧЧһ��������
    bit1λ ʧЧ����������
    bit0λ �ͱ��׹���
    */
    uint8_t State;	                //����״̬
    //INF_SOUND_CERTAIN CERTAIN_KEY;//��ť��
    DWORD PULSE3;                   //����3 -- �ٽ�̽ͷר��
    
    //float  DoseRate;  
    //float  Dose;
	//uint8_t State;	            //״̬�֡�����Ϊ��Ч��������Ϊ��Ч������2λ�ֱ��ʾ������һ�������������ʶ����������ۼƼ�����һ���������ۼƼ����ʶ������������ر������ͱ��׹��ϣ�������λδ����
}STU_DOSERATE;

//��������ṹ��    
typedef struct
{
    float  CPS1;                    //���͵�ʱ��ҪתΪ6�ֽ�asc���뷢��
    float  CPS2;	                //���͵�ʱ��ҪתΪ6�ֽ�asc���뷢��
	uint8_t State;	                //״̬�֡�����Ϊ��Ч��������Ϊ��Ч������2λ�ֱ��ʾ������һ�������������ʶ����������ۼƼ�����һ���������ۼƼ����ʶ������������ر������ͱ��׹��ϣ�������λδ����
}STU_CPS;
#pragma pack()


typedef struct
{
    uint16_t   Sign;                // 0x4142
	uint8_t    address;             //̽ͷ����
	char       VerSion[6];	        //�汾��
	SYS_PRAM   s_SysParam;
	SYS_JIAOZHUNYINZI s_Jiaozhun;   //У׼����
	SYS_ALARM  s_Alarm;
    float      CoCsRate;   
	WORD   ParaCheckSum;            //����У���
}LP_PARAM;



typedef struct
{
  	LP_PARAM	stParam;
	uint8_t SensorType;
	uint8_t keydownFlag;            //�������±�־
	uint8_t keyValue;               //������ֵ
	uint32_t keydownTime;           //�������¼���ʱ��*10ms
	uint8_t isCanReadSensor;
	STU_DOSERATE s_DoseMSG;
	STU_CPS s_CPS;
	uint8_t range;	                //���õ�����
	uint8_t RealRange;              //ʵ��ʹ�õ�����
	uint8_t isSwitching;	        // >0�����л���λ��
	uint16_t DoseRatePreAlarmCnt;   //������Ԥ������������2�α����ܱ���
	uint16_t DoseRateAlarmCnt;      //�����ʱ�������������2�α����ܱ���
	uint32_t LChannelNoCountTime;   //��ͨ���޼���ʱ��
}LP_SYSTEM_STTAE;


//===============================��������=============================
extern LP_SYSTEM_STTAE SysRunState;


//===============================��������=============================
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





