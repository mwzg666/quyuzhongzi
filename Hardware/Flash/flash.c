#include "main.h"
#include "flash.h"

#define IAP_STANDBY()   IAP_CMD = 0     //IAP���������ֹ��
#define IAP_READ()      IAP_CMD = 1     //IAP��������
#define IAP_WRITE()     IAP_CMD = 2     //IAPд������
#define IAP_ERASE()     IAP_CMD = 3     //IAP��������

#define IAP_ENABLE()    IAP_CONTR = IAP_EN; IAP_TPS = MAIN_Fosc / 1000000
#define IAP_DISABLE()   IAP_CONTR = 0; IAP_CMD = 0; IAP_TRIG = 0; IAP_ADDRE = 0xff; IAP_ADDRH = 0xff; IAP_ADDRL = 0xff

#define IAP_EN          (1<<7)
#define IAP_SWBS        (1<<6)
#define IAP_SWRST       (1<<5)
#define IAP_CMD_FAIL    (1<<4)

//========================================================================
// ��������: void checkApp(void)
// ��������: ���Ӧ�ó����Ƿ���Ч����ַ0x4400���Ƿ���0xa55a),����д��0xa55a
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void checkApp(void)
{ 
	uint8_t buf[2];
  	EEPROM_read((u32)APP_CORRECT_ADDR, (u8 *)buf, 2);
	if((buf[0] != 0xa5)&&(buf[1] != 0x5a))
	{
	  	buf[0] = 0xa5;
		buf[1] = 0x5a;

		EEPROM_write((u32)APP_CORRECT_ADDR,(u8 *)buf, 2);	
	}
}

//========================================================================
// ����: void DisableEEPROM(void)
// ����: ��ֹEEPROM.
// ����: none.
// ����: none.
// �汾: V1.0, 2014-6-30
//========================================================================
void DisableEEPROM(void)        //��ֹ����EEPROM
{
    IAP_CONTR = 0;              //�ر� IAP ����
    IAP_CMD = 0;                //�������Ĵ���
    IAP_TRIG = 0;               //��������Ĵ���
    IAP_ADDRE = 0xff;           //����ַ���õ��� IAP ����
    IAP_ADDRH = 0xff;           //����ַ���õ��� IAP ����
    IAP_ADDRL = 0xff;
}

//========================================================================
// ����: void EEPROM_Trig(void)
// ����: ����EEPROM����.
// ����: none.
// ����: none.
// �汾: V1.0, 2014-6-30
//========================================================================
void EEPROM_Trig(void)
{
    F0 = EA;                //����ȫ���ж�
    EA = 0;                 //��ֹ�ж�, ���ⴥ��������Ч
    IAP_TRIG = 0x5A;
    IAP_TRIG = 0xA5;        //����5AH������A5H��IAP�����Ĵ�����ÿ�ζ���Ҫ���
                            //����A5H��IAP������������������
                            //CPU�ȴ�IAP��ɺ󣬲Ż����ִ�г���
    _nop_();                //����STC32G�Ƕ༶��ˮ�ߵ�ָ��ϵͳ��������������4��NOP����֤IAP_DATA���������׼��
    _nop_();
    _nop_();
    _nop_();
    EA = F0;                //�ָ�ȫ���ж�
}

//========================================================================
// ����: void EEPROM_SectorErase(u32 EE_address)
// ����: ����һ������.
// ����: EE_address:  Ҫ������EEPROM�������е�һ���ֽڵ�ַ.
// ����: none.
// �汾: V1.0, 2014-6-30
//========================================================================
void EEPROM_SectorErase(u32 EE_address)
{
    IAP_ENABLE();                           //���õȴ�ʱ�䣬����IAP��������һ�ξ͹�
    IAP_ERASE();                            //�����, ������������������ı�ʱ����������������
                                            //ֻ������������û���ֽڲ�����512�ֽ�/������
                                            //����������һ���ֽڵ�ַ����������ַ��
    IAP_ADDRE = (u8)(EE_address >> 16);     //��������ַ���ֽڣ���ַ��Ҫ�ı�ʱ���������͵�ַ��
    IAP_ADDRH = (u8)(EE_address >> 8);      //��������ַ���ֽڣ���ַ��Ҫ�ı�ʱ���������͵�ַ��
    IAP_ADDRL = (u8)EE_address;             //��������ַ���ֽڣ���ַ��Ҫ�ı�ʱ���������͵�ַ��
    EEPROM_Trig();                          //����EEPROM����
    DisableEEPROM();                        //��ֹEEPROM����
}

//========================================================================
// ����: void EEPROM_read_n(u32 EE_address,u8 *DataAddress,u8 lenth)
// ����: ��N���ֽں���.
// ����: EE_address:  Ҫ������EEPROM���׵�ַ.
//       DataAddress: Ҫ�������ݵ�ָ��.
//       length:      Ҫ�����ĳ���
// ����: 0: д����ȷ.  1: д�볤��Ϊ0����.  2: д�����ݴ���.
// �汾: V1.0, 2014-6-30
//========================================================================
void EEPROM_read(u32 EE_address,u8 *DataAddress,u16 length)
{
    EA = 0;  
    IAP_ENABLE();                           //���õȴ�ʱ�䣬����IAP��������һ�ξ͹�
    IAP_READ();                             //���ֽڶ���������ı�ʱ����������������
    do
    {
        IAP_ADDRE = (u8)(EE_address >> 16);//�͵�ַ���ֽڣ���ַ��Ҫ�ı�ʱ���������͵�ַ��
        IAP_ADDRH = (u8)(EE_address >> 8);  //�͵�ַ���ֽڣ���ַ��Ҫ�ı�ʱ���������͵�ַ��
        IAP_ADDRL = (u8)EE_address;         //�͵�ַ���ֽڣ���ַ��Ҫ�ı�ʱ���������͵�ַ��
        EEPROM_Trig();                      //����EEPROM����
        *DataAddress = IAP_DATA;            //��������������
        EE_address++;
        DataAddress++;
    }while(--length);

    DisableEEPROM();
    EA = 1;  
}


//========================================================================
// ����: u8 EEPROM_write_n(u32 EE_address,u8 *DataAddress,u8 length)
// ����: дN���ֽں���.
// ����: EE_address:  Ҫд���EEPROM���׵�ַ.
//       DataAddress: Ҫд�����ݵ�ָ��.
//       length:      Ҫд��ĳ���
// ����: 0: д����ȷ.  1: д�볤��Ϊ0����.  2: д�����ݴ���.
// �汾: V1.0, 2014-6-30
//========================================================================
u8 EEPROM_write(u32 EE_address,u8 *DataAddress,u16 length)
{
   u8 ret = true;
    u8  i;
    u16 j;
    u8  *p;
    
    if(length == 0) return 1;               //����Ϊ0����

    IAP_ENABLE();                           //���õȴ�ʱ�䣬����IAP��������һ�ξ͹�
    i = length;
    j = EE_address;
    p = DataAddress;
    IAP_WRITE();                            //�����, ���ֽ�д����
    do
    {
        IAP_ADDRE = (u8)(EE_address >> 16);//�͵�ַ���ֽڣ���ַ��Ҫ�ı�ʱ���������͵�ַ��
        IAP_ADDRH = (u8)(EE_address >> 8);  //�͵�ַ���ֽڣ���ַ��Ҫ�ı�ʱ���������͵�ַ��
        IAP_ADDRL = (u8)EE_address;         //�͵�ַ���ֽڣ���ַ��Ҫ�ı�ʱ���������͵�ַ��
        IAP_DATA  = *DataAddress;           //�����ݵ�IAP_DATA��ֻ�����ݸı�ʱ����������
        EEPROM_Trig();                      //����EEPROM����
        EE_address++;                       //��һ����ַ
        DataAddress++;                      //��һ������
    }while(--length);                       //ֱ������

    EE_address = j;
    length = i;
    DataAddress = p;
    i = 0;
    IAP_READ();                             //��N���ֽڲ��Ƚ�
    do
    {
        IAP_ADDRE = (u8)(EE_address >> 16);//�͵�ַ���ֽڣ���ַ��Ҫ�ı�ʱ���������͵�ַ��
        IAP_ADDRH = (u8)(EE_address >> 8);  //�͵�ַ���ֽڣ���ַ��Ҫ�ı�ʱ���������͵�ַ��
        IAP_ADDRL = (u8)EE_address;         //�͵�ַ���ֽڣ���ַ��Ҫ�ı�ʱ���������͵�ַ��
        EEPROM_Trig();                      //����EEPROM����
        if(*DataAddress != IAP_DATA)        //������������Դ���ݱȽ�
        {
            ret = false;
            break;
        }
        EE_address++;
        DataAddress++;
    }while(--length);

    DisableEEPROM();
    return ret;
}


