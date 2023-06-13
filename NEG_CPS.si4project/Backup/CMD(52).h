#ifndef CMD_H
#define CMD_H

#include "main.h"

//===============================��������=============================
#define WORKMODE1 0x0001
#define WORKMODE2 0x0002
#define WORKMODE3 0x0003
#define Sensor_Area 100             //̽�������


//===============================�ṹ�嶨��===========================
#pragma pack(1)
typedef struct
{
	uint8_t head;	
	uint8_t length;
	uint8_t type;
	uint8_t channel;
	uint8_t cmd;
	//uint8_t *data;
	//uint16_t SumCheck;
	//uint8_t tail;
}STU_CMD;
#pragma pack()


//===============================��������=============================
extern LP_SYSTEM_STTAE SysRunState;


//===============================��������=============================
void DataPro(uint8_t *cdata, uint8_t length);

void ACK_CMD_C(void);                        //��������
void ACK_CMD_H(uint8_t Address);             //����̽������ַ


void ACK_CMD_R(void);                        //����������
void ACK_CMD_W(uint8_t *cdata);              //д��������
void ACK_CMD_N(unsigned char *cdata);   //����У׼����
void ACK_CMD_J(void);                        //��У׼����

void ACK_CMD_V(void);                        //����������
void ACK_CMD_B(uint8_t *cdata);              //д������������
void ACK_CMD_F(void);                        //��������������
void ACK_CMD_P(void);
void ACK_CMD_T(void);
//void ACK_CMD_Q(uint8_t range);
void ACK_CMD_M(void);

void ReadCS();
void SetCS(u8 *cs);

void CMD(u32 id,unsigned char dlc,unsigned char *cdata);
void GetPara(LP_PARAM *me);                  //��ʼ��ʱ��ͨ�������������ݴ�info flash������RAM��
void WritePara();
uint8_t GetCautionStatus_Alphy(float CNT);
uint8_t GetCautionStatus_Beta(float CNT);
void Get_Cur_Effic(unsigned char netro_num);
WORD CheckSum(BYTE *buf, WORD len);
void Updata_BenDi_Value();
void SaveParam();
DWORD DwordToSmall(DWORD dat);

#endif
