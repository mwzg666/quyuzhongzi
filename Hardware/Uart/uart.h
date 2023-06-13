#ifndef UART_H
#define UART_H
#include "main.h"

//===============================��������=============================
#define Uart_BaudRate       9600
#define Baudrate            38400UL
#define MAX_LENGTH          200

#define SYSTEM_FREQ         25000000

#define UART_MAX_BAUDRATE   115200    //���崮��������С������
#define UART_MIN_BAUDRATE   1200

#define UART_USB_BAUDRATE   9600      //���ô��ڲ�����


//===============================�ṹ�嶨��===========================



//===============================��������=============================
extern u8  TX1_Cnt;                 //���ͼ���
extern u8  RX1_Cnt;                 //���ռ���
extern bit B_TX1_Busy;              //����æ��־
extern u16 Rx1_Timer;
extern u8  RX1_Buffer[MAX_LENGTH];  //���ջ���


//===============================��������=============================
void Uart1_Init(void);
void InitUart();
void uart_send(u8 *buf, u8 len);
void ClearRs485Buf();

#endif