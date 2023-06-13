#include <stdlib.h>
#include "CMD.h"
#include "sensor.h"
#include "mcp4725.h"
#include "flash.h"
#include "uart.h"
#include "mwpro.h"

STU_CMD s_Head={0x02};
uint8_t sendbuf[100];


//========================================================================
// ��������: WORD WordToSmall(WORD dat)
// ��������: ��WORD������ת��ΪС��ģʽ
// ��ڲ���: @WORD dat��Ҫת��������
// ��������: ��������ΪWORD��С��ģʽ����
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================

WORD WordToSmall(WORD dat)
{
	BYTE buf[2];
    BYTE t;
    WORD ret;
    
    memcpy(buf, &dat, 2);
	t = buf[1];
	buf[0] = buf[1];
	buf[1] = t;
	
    memcpy(&ret, buf, 2);
    return ret;
}

float FloatToSmall(float dat)
{
	BYTE buf[4];
    BYTE t;
    float ret;
    
    memcpy(buf, &dat, 4);
	t = buf[3];
	buf[3] = buf[0];
	buf[0] = t;
	t = buf[2];
	buf[2] = buf[1];
	buf[1] = t;

    memcpy(&ret, buf, 4);
    return ret;
}

DWORD DwordToSmall(DWORD dat)
{
	BYTE buf[4];
    BYTE t;
    DWORD ret;
    
    memcpy(buf, &dat, 4);
	t = buf[3];
	buf[3] = buf[0];
	buf[0] = t;
	t = buf[2];
	buf[2] = buf[1];
	buf[1] = t;

    memcpy(&ret, buf, 4);
    return ret;
}

//========================================================================
// ��������: void GetPara(LP_PARAM *me)
// ��������: ��FLASH�ж�ȡ����������"���Ʋ���"��"��������"
// ��ڲ���: @me������
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void GetPara(LP_PARAM *me)
{ 
  	EEPROM_read(0,(u8 *)me,sizeof(LP_PARAM));
    if ( (SysRunState.stParam.ParaCheckSum !=  CheckSum((BYTE *)&SysRunState.stParam,sizeof(LP_PARAM)-2))
        || (SysRunState.stParam.Sign != 0x4142) )
	{
	  	InitParam();
	}
}

//========================================================================
// ��������: void WritePara()
// ��������: д�����ݵ��ڴ���
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void WritePara()
{
	EA = 0;
    EEPROM_SectorErase(0);
    EEPROM_SectorErase(512);
	SysRunState.stParam.ParaCheckSum = CheckSum((BYTE *)&SysRunState.stParam,sizeof(LP_PARAM)-2);//add by kevin at 20150417
	if (!EEPROM_write(0, (u8 *)&SysRunState.stParam, sizeof(LP_PARAM)))
    {
        Error();
    }	
	EA = 1;
}

//========================================================================
// ��������: void SendData(uint8_t cmd, uint8_t *cdata, uint8_t length)
// ��������: ����λ����������
// ��ڲ���: @cmd�����@*cdata�����͵����ݣ�@length���������ݵĳ���
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void SendData(uint8_t cmd, uint8_t *cdata, uint8_t length)
{
  	u8 sumCheck=0,i;

	s_Head.head = HEAD;
	s_Head.length = length + 8;
	s_Head.type = TR600;             //SysRunState.SensorType+0x11;//0x11<->310; 0x12<->TR320; 0x13<->TR330; 0x20<->TR600
	s_Head.channel = SysRunState.stParam.address;
	s_Head.cmd = cmd;
	memcpy(sendbuf,(uint8_t*)&s_Head,sizeof(STU_CMD));
	if(length>0)
	{
		memcpy(&sendbuf[sizeof(STU_CMD)],cdata,length);
	}
    
    for(i=0;i<length+4;i++)
	{
	  	sumCheck += sendbuf[i+1];
	}
	sprintf((char*)&sendbuf[length+sizeof(STU_CMD)],"%02X",sumCheck);
	sendbuf[length+sizeof(STU_CMD)+2] = 3;
	uart_send(sendbuf,(u8)(length+8));
}



void AlarmConfirm()
{
    SysRunState.s_DoseMSG.Dose = 0;   // ���ۼƼ���
    SysRunState.s_DoseMSG.State = 0; // �屨��״̬  
    SendData('E',NULL,0);
}

STU_CMD gs_CMD={0};

//========================================================================
// ��������: void DataPro(uint8_t *cdata, uint8_t length)
// ��������: ���������ִ��
// ��ڲ���: @*cdata: ����ָ�룻@length�����ݳ���
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void DataPro(uint8_t *cdata, uint8_t length)
{  
  	uint16_t i,j;
	uint8_t checkOut = 0;
	char checkOutbuf[3] = {0};
    //for(i=0;i<length; i++)	
	{
        i = 0;
        
	  	if(cdata[i] == HEAD)
		{
		  	memcpy(&gs_CMD,&cdata[i],sizeof(STU_CMD));
			if((gs_CMD.length > length)||(cdata[gs_CMD.length-1] != TAIL)
			   ||(!((gs_CMD.channel == SysRunState.stParam.address)||(gs_CMD.channel == 0xff)))
			   ||(!((gs_CMD.type == TR600)||(gs_CMD.type == 0xff)))    
				 )
			{
		        //continue;
			    return;    
			}
            
			for(j=0; j< gs_CMD.length-4; j++)
			{
			  	checkOut += cdata[i+j+1];
			}
			sprintf(checkOutbuf,"%02X",checkOut);
            
            //printf("checkOutbuf = %s\r\n",checkOutbuf);
            
			if(memcmp(checkOutbuf,&cdata[i+gs_CMD.length-3],2) == 0)
			{
				switch(gs_CMD.cmd)
				{
					case 'C':  ACK_CMD_C();                 break;      // ��������
					case 'V':  ACK_CMD_V();                 break;      // ������
					case 'P':  ACK_CMD_P();                 break;      // ������
					case 'H':  ACK_CMD_H(cdata[i+5]);       break;      // ����̽������ַ
					case 'B':  ACK_CMD_B(&cdata[i+5]);      break;      // д��������
                    case 'W':  ACK_CMD_W(&cdata[i+5]);      break;      // д����
					case 'F':  ACK_CMD_F();break;                       // ����������
				    case 'R':  ACK_CMD_R();break;                       // ������
                    case 'T':  ACK_CMD_T();break;                       // ����
					//case 'Q':  ACK_CMD_Q(cdata[i+5]);     break;      // ���̲�ѯ��������
					case 'M':  ACK_CMD_M();break;                       // ����汾
					//case 'J':  ACK_CMD_J();break;                       // ��У׼����
					//case 'N':  ACK_CMD_N(&cdata[i+5]);      break;      // �޸�У׼����
                    case 'X':  ReadCS(); break;                         // ��Co/Cs��
					case 'Y':  SetCS(&cdata[i+5]);          break;      // дCo/Cs��
					case 'E':  AlarmConfirm();              break;      // ����ȷ��
					//case 0x28:  asm(" mov &0xFFBE, PC;");  break;     // Զ������,��ת����������
				    default:break;
				}
				i += (gs_CMD.length+4);
			}
		}
    }
}

//========================================================================
// ��������: void ReadCS()
// ��������: ��Co/Cs�����X����Ӧ
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void ReadCS()
{
    float Cocs;
    Cocs = FloatToSmall(SysRunState.stParam.CoCsRate);
    SendData('X',(uint8_t*)&Cocs,4);
}

//========================================================================
// ��������: void SetCS(u8 *cs)
// ��������: ����Co/Cs�ȣ�Y��
// ��ڲ���: @*cs��Ҫд���Co/Cs������ָ��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void SetCS(u8 *cs)
{
    float Cocs;
    Cocs = FloatToSmall(*cs);    
    memcpy(&SysRunState.stParam.CoCsRate,&Cocs, 4);   
    SendData('Y',NULL,0);
	//SaveParam(&SysRunState.stParam);
    SaveParam();
}  

//========================================================================
// ��������: void ACK_CMD_C(void)
// ��������: �������C����Ӧ
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void ACK_CMD_C(void)
{
	SendData('C',NULL,0);
}

//========================================================================
// ��������: void ACK_CMD_H(uint8_t Address)
// ��������: ����̽������ַ��H��
// ��ڲ���: @Address��̽������ַ
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void ACK_CMD_H(uint8_t Address)
{
	SysRunState.stParam.address = Address;
	SendData('H',NULL,0);
    SaveParam();
}

//========================================================================
// ��������: void ACK_CMD_R(void)
// ��������: ��̽������������(R)��Ӧ
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void ACK_CMD_R(void)
{   
    SYS_PRAM cs;

    cs.Canshu1 = FloatToSmall(SysRunState.stParam.s_SysParam.Canshu1);
    cs.Canshu2 = FloatToSmall(SysRunState.stParam.s_SysParam.Canshu2);
    memcpy(cs.yuzhi1,SysRunState.stParam.s_SysParam.yuzhi1,4);
    memcpy(cs.yuzhi2,SysRunState.stParam.s_SysParam.yuzhi2,4);
    memcpy(cs.PingHuaShiJian,SysRunState.stParam.s_SysParam.PingHuaShiJian,4);  
	SendData('R',(uint8_t*)&cs,sizeof(SYS_PRAM));
}


//========================================================================
// ��������: void ACK_CMD_W(unsigned char *cdata)
// ��������: д̽������������(W)��Ӧ
// ��ڲ���: @*cdata��Ҫд�������ָ��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void ACK_CMD_W(unsigned char *cdata)
{
    SYS_PRAM wcm;
    memcpy((uint8_t*)&wcm,cdata,sizeof(SYS_PRAM));
    
    SysRunState.stParam.s_SysParam.Canshu1 = FloatToSmall(wcm.Canshu1);
    SysRunState.stParam.s_SysParam.Canshu2 = FloatToSmall(wcm.Canshu2);
    
    memcpy(SysRunState.stParam.s_SysParam.yuzhi1,wcm.yuzhi1,4);
    memcpy(SysRunState.stParam.s_SysParam.yuzhi2,wcm.yuzhi2,4);
    memcpy(SysRunState.stParam.s_SysParam.PingHuaShiJian,wcm.PingHuaShiJian,4);

    #ifndef POE_SEN
    SysRunState.stParam.s_SysParam.Canshu1 = FloatToSmall(wcm.Canshu1);
    SysRunState.stParam.s_SysParam.Canshu2  = FloatToSmall(wcm.Canshu2);
	SysRunState.stParam.s_Jiaozhun.DI_C     = SysRunState.stParam.s_SysParam.Canshu1;
	SysRunState.stParam.s_Jiaozhun.GAO_C    = SysRunState.stParam.s_SysParam.Canshu2;
    RefreshParam();
    #endif
	
	MCP4725_OutVol(MCP4725_S1_ADDR,SysRunState.stParam.s_SysParam.yuzhi1);
	
	SendData('W',NULL,0);
    SaveParam();
}

//========================================================================
// ��������: void ACK_CMD_N(unsigned char *cdata)
// ��������: дУ׼����(N)��Ӧ
// ��ڲ���: @*cdata��Ҫд�������ָ��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void ACK_CMD_N(unsigned char *cdata)
{
//    SYS_JIAOZHUNYINZI wxz;
//    
//    memcpy(&wxz,cdata,sizeof(SYS_JIAOZHUNYINZI));
//    SysRunState.stParam.s_Jiaozhun.DI_A = FloatToSmall(wxz.DI_A);
//    SysRunState.stParam.s_Jiaozhun.DI_B = FloatToSmall(wxz.DI_B );
//    SysRunState.stParam.s_Jiaozhun.DI_C = FloatToSmall(wxz.DI_C);
//    
//    SysRunState.stParam.s_Jiaozhun.GAO_A = FloatToSmall(wxz.GAO_A);
//    SysRunState.stParam.s_Jiaozhun.GAO_B = FloatToSmall(wxz.GAO_B);
//    SysRunState.stParam.s_Jiaozhun.GAO_C = FloatToSmall(wxz.GAO_C);
//    
//    SysRunState.stParam.s_Jiaozhun.SUPER_GAO_A = FloatToSmall(wxz.SUPER_GAO_A);
//    SysRunState.stParam.s_Jiaozhun.SUPER_GAO_B = FloatToSmall(wxz.SUPER_GAO_B);
//    SysRunState.stParam.s_Jiaozhun.SUPER_GAO_C = FloatToSmall(wxz.SUPER_GAO_C);
     
	SendData('N',NULL,0);
    //SaveParam();
	//RefreshParam();
}

//========================================================================
// ��������: void ACK_CMD_J(void)
// ��������: ��У׼����(J)��Ӧ
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void ACK_CMD_J(void)
{
//    SYS_JIAOZHUNYINZI xz;
//    xz.DI_A = FloatToSmall(SysRunState.stParam.s_Jiaozhun.DI_A);
//    xz.DI_B = FloatToSmall(SysRunState.stParam.s_Jiaozhun.DI_B);
//    xz.DI_C = FloatToSmall(SysRunState.stParam.s_Jiaozhun.DI_C);
//    
//    xz.GAO_A = FloatToSmall(SysRunState.stParam.s_Jiaozhun.GAO_A);
//    xz.GAO_B = FloatToSmall(SysRunState.stParam.s_Jiaozhun.GAO_B);
//    xz.GAO_C = FloatToSmall(SysRunState.stParam.s_Jiaozhun.GAO_C);
//    
//    xz.SUPER_GAO_A = FloatToSmall(SysRunState.stParam.s_Jiaozhun.SUPER_GAO_A);
//    xz.SUPER_GAO_B = FloatToSmall(SysRunState.stParam.s_Jiaozhun.SUPER_GAO_B);
//    xz.SUPER_GAO_C = FloatToSmall(SysRunState.stParam.s_Jiaozhun.SUPER_GAO_C);

//    SendData('J',(uint8_t*)&xz,sizeof(SYS_JIAOZHUNYINZI));
}


//========================================================================
// ��������: void ACK_CMD_V(void)
// ��������: ����������(V)��Ӧ
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
STU_DOSERATE gs_Dose;
void ACK_CMD_V(void)
{
    STU_DOSERATE dr;
    
    dr.DoseRate = FloatToSmall(SysRunState.s_DoseMSG.DoseRate);
    dr.Dose     = FloatToSmall(SysRunState.s_DoseMSG.Dose);
    dr.State    = WordToSmall(SysRunState.s_DoseMSG.State);
    dr.PULSE1   = DwordToSmall(SysRunState.s_CPS.CPS1);
    dr.PULSE2   = DwordToSmall(SysRunState.s_CPS.CPS2);
    dr.PULSE3   = 0;

    #ifdef POE_SEN
    {
        dr.DoseRate *= FloatToSmall(SysRunState.stParam.CoCsRate);
    }
    #else
    if ( (SysRunState.s_DoseMSG.DoseRate > 2000) &&
         (SysRunState.s_DoseMSG.DoseRate < 150000) )
    {
        // ����Co/Cs��
        dr.DoseRate *= FloatToSmall(SysRunState.stParam.CoCsRate);
    }
    #endif

	memcpy((void*)&gs_Dose,(void*)&dr,sizeof(STU_DOSERATE));
	SendData('V',(uint8_t*)&gs_Dose,sizeof(STU_DOSERATE));
}


//========================================================================
// ��������: void ACK_CMD_P(void)
// ��������: ����������(P)��Ӧ
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void ACK_CMD_P(void)
{
	char buf[13];
	
	sprintf(buf,"%06ld",(long)SysRunState.s_CPS.CPS1);
	sprintf(&buf[6],"%06ld",(long)SysRunState.s_CPS.CPS2);
	buf[12] = SysRunState.s_CPS.State;

	SendData('P',(uint8_t*)buf,13);
}



//========================================================================
// ��������: void ACK_CMD_B(u8 *cdata)
// ��������: д������������(B)��Ӧ
// ��ڲ���: @*cdata��Ҫд�������ָ��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void ACK_CMD_B(u8 *cdata)
{
    SYS_ALARM wal;   
    memcpy(&wal,cdata,sizeof(SYS_ALARM));
    SysRunState.stParam.s_Alarm.DoseAlarm           = FloatToSmall(  wal.DoseAlarm);
    SysRunState.stParam.s_Alarm.DosePreAlarm        = FloatToSmall(wal.DosePreAlarm);
    SysRunState.stParam.s_Alarm.DoseRateAlarm       = FloatToSmall(wal.DoseRateAlarm);
    SysRunState.stParam.s_Alarm.DoseRatePreAlarm    = FloatToSmall(wal.DoseRatePreAlarm);
    //SysRunState.stParam.s_Alarm.InvalidAlarm1       = FloatToSmall(wal.InvalidAlarm1);
    //SysRunState.stParam.s_Alarm.InvalidAlarm2       = FloatToSmall(wal.InvalidAlarm2);
    
	SendData('B',NULL,0);
	SaveParam();
} 

SYS_ALARM Arm;
//========================================================================
// ��������: void ACK_CMD_F(void)
// ��������: ��������������(F)��Ӧ
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void ACK_CMD_F(void)
{  
    SYS_ALARM al;
    al.DoseAlarm        = FloatToSmall(SysRunState.stParam.s_Alarm.DoseAlarm);
    al.DosePreAlarm     = FloatToSmall(SysRunState.stParam.s_Alarm.DosePreAlarm);
    al.DoseRateAlarm    = FloatToSmall(SysRunState.stParam.s_Alarm.DoseRateAlarm);
    al.DoseRatePreAlarm = FloatToSmall(SysRunState.stParam.s_Alarm.DoseRatePreAlarm);
    //al.InvalidAlarm1    = FloatToSmall(SysRunState.stParam.s_Alarm.InvalidAlarm1);
    //al.InvalidAlarm2    = FloatToSmall(SysRunState.stParam.s_Alarm.InvalidAlarm2);
    memcpy(&Arm,&al,sizeof(SYS_ALARM)); 
	SendData('F',(uint8_t*)&Arm,sizeof(SYS_ALARM));
} 


/*******************************************************************************
���ܣ�����
���룺��
�������
*******************************************************************************/
void ACK_CMD_T(void)
{
	SendData('T',NULL,0);
}

/*******************************************************************************
���ܣ����̲�ѯ��������
���룺
�������
*******************************************************************************/
/*
void ACK_CMD_Q(uint8_t range)
{
	if((range == 'H')||(range == 'L')||(range == 'A'))
	{
	  	if(SysRunState.SensorType == WHOLE)//ֻ��ȫ���̲����ֶ��л�����
		{
			SysRunState.range = range;
			SendData('Q',NULL,0);
			if(range == 'H')
			{
				CHANNEL_H();
			}
			else if(range == 'L')
			{
				CHANNEL_L();
			}
			//RefreshParam();
		}
	}
	else
	{
	  	SendData('Q',(uint8_t*)&SysRunState.range,1);
	}
	
}

*/
/*******************************************************************************
���ܣ����汾��M����Ӧ
���룺��
�������
*******************************************************************************/
void ACK_CMD_M(void)
{	
 	SendData('M',(uint8_t*)SysRunState.stParam.VerSion,6);
}

//========================================================================
// ��������: WORD CheckSum(BYTE *buf, WORD len)
// ��������: У��ͺ���
// ��ڲ���: @*buf�����ݣ�@len������
// ��������: У����
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
WORD CheckSum(BYTE *buf, WORD len)
{
    WORD dwSum = 0;
    WORD i;

    for (i = 0; i < len; i++)
    {
        dwSum += buf[i];
    }
    return dwSum;
}

void SaveParam()
{
  	EA = 0;//��ֹ�ж�
  	WritePara();
  	EA = 1;//�����ж�
}
