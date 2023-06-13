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
// 函数名称: WORD WordToSmall(WORD dat)
// 函数功能: 将WORD的数据转换为小端模式
// 入口参数: @WORD dat：要转换的数据
// 函数返回: 返回类型为WORD的小端模式数据
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
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
// 函数名称: void GetPara(LP_PARAM *me)
// 函数功能: 从FLASH中读取参数，包括"控制参数"和"报警参数"
// 入口参数: @me：数据
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
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
// 函数名称: void WritePara()
// 函数功能: 写入数据到内存中
// 入口参数: @无
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
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
// 函数名称: void SendData(uint8_t cmd, uint8_t *cdata, uint8_t length)
// 函数功能: 向上位机发送命令
// 入口参数: @cmd：命令；@*cdata：发送的数据；@length：发送数据的长度
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
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
    SysRunState.s_DoseMSG.Dose = 0;   // 清累计剂量
    SysRunState.s_DoseMSG.State = 0; // 清报警状态  
    SendData('E',NULL,0);
}

STU_CMD gs_CMD={0};

//========================================================================
// 函数名称: void DataPro(uint8_t *cdata, uint8_t length)
// 函数功能: 命令分析与执行
// 入口参数: @*cdata: 数据指针；@length：数据长度
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
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
					case 'C':  ACK_CMD_C();                 break;      // 联络命令
					case 'V':  ACK_CMD_V();                 break;      // 读计数
					case 'P':  ACK_CMD_P();                 break;      // 读脉冲
					case 'H':  ACK_CMD_H(cdata[i+5]);       break;      // 设置探测器地址
					case 'B':  ACK_CMD_B(&cdata[i+5]);      break;      // 写报警参数
                    case 'W':  ACK_CMD_W(&cdata[i+5]);      break;      // 写参数
					case 'F':  ACK_CMD_F();break;                       // 读报警参数
				    case 'R':  ACK_CMD_R();break;                       // 读参数
                    case 'T':  ACK_CMD_T();break;                       // 测温
					//case 'Q':  ACK_CMD_Q(cdata[i+5]);     break;      // 量程查询或者设置
					case 'M':  ACK_CMD_M();break;                       // 软件版本
					//case 'J':  ACK_CMD_J();break;                       // 读校准因子
					//case 'N':  ACK_CMD_N(&cdata[i+5]);      break;      // 修改校准因子
                    case 'X':  ReadCS(); break;                         // 读Co/Cs比
					case 'Y':  SetCS(&cdata[i+5]);          break;      // 写Co/Cs比
					case 'E':  AlarmConfirm();              break;      // 报警确认
					//case 0x28:  asm(" mov &0xFFBE, PC;");  break;     // 远程升级,跳转到升级代码
				    default:break;
				}
				i += (gs_CMD.length+4);
			}
		}
    }
}

//========================================================================
// 函数名称: void ReadCS()
// 函数功能: 读Co/Cs比命令（X）响应
// 入口参数: @无
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================
void ReadCS()
{
    float Cocs;
    Cocs = FloatToSmall(SysRunState.stParam.CoCsRate);
    SendData('X',(uint8_t*)&Cocs,4);
}

//========================================================================
// 函数名称: void SetCS(u8 *cs)
// 函数功能: 设置Co/Cs比（Y）
// 入口参数: @*cs：要写入的Co/Cs比数据指针
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
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
// 函数名称: void ACK_CMD_C(void)
// 函数功能: 联络命令（C）响应
// 入口参数: @无
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================
void ACK_CMD_C(void)
{
	SendData('C',NULL,0);
}

//========================================================================
// 函数名称: void ACK_CMD_H(uint8_t Address)
// 函数功能: 设置探测器地址（H）
// 入口参数: @Address：探测器地址
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================
void ACK_CMD_H(uint8_t Address)
{
	SysRunState.stParam.address = Address;
	SendData('H',NULL,0);
    SaveParam();
}

//========================================================================
// 函数名称: void ACK_CMD_R(void)
// 函数功能: 读探测器参数命令(R)响应
// 入口参数: @无
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
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
// 函数名称: void ACK_CMD_W(unsigned char *cdata)
// 函数功能: 写探测器参数命令(W)响应
// 入口参数: @*cdata：要写入的数据指针
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
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
// 函数名称: void ACK_CMD_N(unsigned char *cdata)
// 函数功能: 写校准因子(N)响应
// 入口参数: @*cdata：要写入的数据指针
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
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
// 函数名称: void ACK_CMD_J(void)
// 函数功能: 读校准因子(J)响应
// 入口参数: @无
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
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
// 函数名称: void ACK_CMD_V(void)
// 函数功能: 读剂量命令(V)响应
// 入口参数: @无
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
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
        // 增加Co/Cs比
        dr.DoseRate *= FloatToSmall(SysRunState.stParam.CoCsRate);
    }
    #endif

	memcpy((void*)&gs_Dose,(void*)&dr,sizeof(STU_DOSERATE));
	SendData('V',(uint8_t*)&gs_Dose,sizeof(STU_DOSERATE));
}


//========================================================================
// 函数名称: void ACK_CMD_P(void)
// 函数功能: 读计数命令(P)响应
// 入口参数: @无
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
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
// 函数名称: void ACK_CMD_B(u8 *cdata)
// 函数功能: 写报警参数命令(B)响应
// 入口参数: @*cdata：要写入的数据指针
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
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
// 函数名称: void ACK_CMD_F(void)
// 函数功能: 读报警参数命令(F)响应
// 入口参数: @无
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
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
功能：测温
输入：无
输出：无
*******************************************************************************/
void ACK_CMD_T(void)
{
	SendData('T',NULL,0);
}

/*******************************************************************************
功能：量程查询或者设置
输入：
输出：无
*******************************************************************************/
/*
void ACK_CMD_Q(uint8_t range)
{
	if((range == 'H')||(range == 'L')||(range == 'A'))
	{
	  	if(SysRunState.SensorType == WHOLE)//只有全量程才能手动切换量程
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
功能：读版本（M）响应
输入：无
输出：无
*******************************************************************************/
void ACK_CMD_M(void)
{	
 	SendData('M',(uint8_t*)SysRunState.stParam.VerSion,6);
}

//========================================================================
// 函数名称: WORD CheckSum(BYTE *buf, WORD len)
// 函数功能: 校验和函数
// 入口参数: @*buf：数据；@len：长度
// 函数返回: 校验结果
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
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
  	EA = 0;//禁止中断
  	WritePara();
  	EA = 1;//允许中断
}
