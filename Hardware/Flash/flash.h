#ifndef FLASH_H_
#define FLASH_H_

//===============================��������=============================
#define FLASH_PARA_START_ADDR   0x1800          //INFO A start address,used to storage para
#define FLASH_PARA_END_ADDR     0x19ff          //INFO A end address,used to storage para

#define FLASH_DATA_START_ADDR 0x010000          //�ۼƼ����洢��ַ
#define FLASH_DATA_END_ADDR     0x016400     	//��10000��ʼ��50��block���ܹ�25600�ֽڣ��ɴ洢6400������,��֤�洢8�꣬���80�꣬7.1Сʱѭ��һ��

#define BOOTLOADER_START_ADDR   0x1A400         // 4K for bootloader

#define FLASH_UPDATE_ADDR       0xE9FE

#define APP_CORRECT_ADDR	    0x4400		    //Ӧ�ó�����Ч����ַ


//===============================�ṹ�嶨��===========================




//===============================��������=============================



//===============================��������=============================
void checkApp(void);
void EEPROM_read(u32 EE_address,u8 *DataAddress,u16 length);
u8 EEPROM_write(u32 EE_address,u8 *DataAddress,u16 length);
void EEPROM_SectorErase(u32 EE_address);


#endif