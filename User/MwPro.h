#ifndef __MWPRO_H__
#define __MWPRO_H__
#include "main.h"

//#define BUFF_LENGTH  MAX_LENGTH
//#define RecLength   RX1_Cnt 
//#define RecvBuf    RX1_Buffer

#define HEAD   0x02
#define TAIL   0x03


#pragma pack(1)
/*
// Э��ͷ�ṹ
typedef struct
{
    BYTE Head;       // 1 0x02 ֡ͷ
    BYTE Len;        // 2 ֡�� ����֡�ĳ���
    BYTE Type;       // 3 ����
    BYTE Addr;       // 4 ��ַ
    BYTE Cmd;        // 5 ����
}FRAME_HEAD;
*/

#pragma pack()


BYTE CheckSum1(BYTE *buf, BYTE len);
BYTE BcdToHex(BYTE Bh, BYTE Bl);

void SendCmd(BYTE Cmd, BYTE *buf, BYTE Length);
BOOL ValidUartFrame();

#endif
