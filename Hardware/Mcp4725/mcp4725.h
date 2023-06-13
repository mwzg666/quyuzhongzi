#ifndef MCP4725_H_
#define MCP4725_H_

//===============================��������=============================
#define MCP4725_S1_ADDR 0x60        //alphy  vth mcp4725 address
#define MCP4725_S2_ADDR 0x61        //beita vth high mcp4725 address
#define MCP4725_S3_ADDR 0X62        //beita vth low mcp4725 address
#define MCP4725_S4_ADDR 0X63        //high voltage mcp4725 address
#define MCP4725_REF_VOL 3300        //mcp4725�Ĳο���ѹ,��ֵ����仯�ˣ���Ҫ�޸�MCP4725_OutVol��������ĳ���ֵ
                                    //�޸�ֵΪ4096/MCP4725_REF_VOL��ֵ
#define MCP4725_PD_MODE 0x00



//===============================�ṹ�嶨��===========================




//===============================��������=============================
extern BYTE ThresAddr[];


//===============================��������=============================
void MCP4725_OutVol(BYTE addr,char *voltage);

#if 0
unsigned int MCP4725_Read(BYTE addr);
#endif

#endif
