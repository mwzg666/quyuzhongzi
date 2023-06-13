#include "io.h"
#include "rtci2c.h"
//#include "AppConfig.h"


/*******************************************************************************
��������:��ʼ��IIC��IO�ڣ����÷���Ϊ���
����:��
���:��
*******************************************************************************/
void RTCI2C_Init(void)
{   
  P4DIR |= (BIT1+BIT2);
}


/*******************************************************************************
��������:����IIC�Ŀ�ʼ�ź�
����:��
���:��
*******************************************************************************/
void RTCI2C_Start()
{
	IOWrite(P4,RTCI2C_SDA_MASK,RTCI2C_SDA_MASK);
	IOWrite(P4,RTCI2C_SCL_MASK,RTCI2C_SCL_MASK);
	delay_us(5);
	IOWrite(P4,RTCI2C_SDA_MASK,0);
	delay_us(5);
	IOWrite(P4,RTCI2C_SCL_MASK,0);
	delay_us(5);
}

/*******************************************************************************
��������:����IIC�Ľ����ź�
����:��
���:��
*******************************************************************************/
void RTCI2C_Stop()
{
	IOWrite(P4,RTCI2C_SCL_MASK,0);
	IOWrite(P4,RTCI2C_SDA_MASK,0);	
	delay_us(5);
	IOWrite(P4,RTCI2C_SCL_MASK,RTCI2C_SCL_MASK);
	delay_us(5);
	IOWrite(P4,RTCI2C_SDA_MASK,RTCI2C_SDA_MASK);
	delay_us(5);
}

/*******************************************************************************
��������:IIC���͵��ֽ�
����:�����͵�����
���:�ɹ�����1
*******************************************************************************/
unsigned char RTCI2C_Write_Byte(char out_data)
{
	unsigned char cnt;
	for(cnt=0;cnt<8;cnt++)
	{
		IOWrite(P4,RTCI2C_SCL_MASK,0);
		delay_us(5);
		if(out_data&(0x80>>cnt))
			IOWrite(P4,RTCI2C_SDA_MASK,RTCI2C_SDA_MASK);
		else
			IOWrite(P4,RTCI2C_SDA_MASK,0);
		
		IOWrite(P4,RTCI2C_SCL_MASK,RTCI2C_SCL_MASK);
  	  	delay_us(5);
	}

	IOWrite(P4, RTCI2C_SCL_MASK,0);//this code is necesary
	IODireIn(P4,RTCI2C_SDA_MASK); //change direction into input
	delay_us(5);
	IOWrite(P4,RTCI2C_SCL_MASK,RTCI2C_SCL_MASK); //release sda bus
	delay_us(5);	
		
	cnt=IORead(P4);
	IOWrite(P4,RTCI2C_SCL_MASK,0);	
	delay_us(5);
	
	IODireOut(P4, RTCI2C_SDA_MASK);//ADD BY KEVIN
        /*
	if(io_data & RTCI2C_SDA_MASK)
	{
		printf("ack error!\n");
	}		
	else
    	{
		printf("ack ok!\n");
    	}
	*/
	return 1;
}


/*******************************************************************************
��������:IIC���յ��ֽ�
����:������ACK��NACK�ź�
���:���ض���������
*******************************************************************************/
unsigned char RTCI2C_Read_Byte(char ack)
{
	unsigned char rd_data=0,cnt=0;
	unsigned char io_data=0,data;
	
        
        /*
         release SDA bus
       */
        IOWrite(P4,RTCI2C_SDA_MASK,RTCI2C_SDA_MASK);
	IODireIn(P4, RTCI2C_SDA_MASK);	
	delay_us(5);
        
	for(cnt=0;cnt<8;cnt++)
    	{
		IOWrite(P4,RTCI2C_SCL_MASK,0);
		delay_us(20);
		IOWrite(P4,RTCI2C_SCL_MASK,RTCI2C_SCL_MASK);
		delay_us(20);
		io_data=IORead(P4);
		rd_data<<=1;
                
		if(io_data&(RTCI2C_SDA_MASK))
	   	  rd_data|=1;
    	}
	
        //ACK  mcp4725
	IOWrite(P4,RTCI2C_SCL_MASK,0);	
	delay_us(5);
	IODireOut(P4, RTCI2C_SDA_MASK);
	IOWrite(P4,RTCI2C_SDA_MASK,ack);
	delay_us(5);        
	IOWrite(P4,RTCI2C_SCL_MASK,RTCI2C_SCL_MASK);	
	delay_us(5);
	IOWrite(P4,RTCI2C_SCL_MASK,0);
	delay_us(5);
        
        /*
        //�ͷ�SDA
        IOWrite(P4,RTCI2C_SDA_MASK,RTCI2C_SDA_MASK);
	IODireIn(P4, RTCI2C_SDA_MASK);	
	delay_us(5);
        */
        
	data=rd_data;
	return data;
}
