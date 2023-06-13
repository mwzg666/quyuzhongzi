#ifndef SENSOR_H
#define SENSOR_H

#include "main.h"

//===============================��������=============================




//===============================�ṹ�嶨��===========================




//===============================��������=============================
extern u8 CPSFlag;


//===============================��������=============================
void SensorInit(void);
void SensorMeasureBegin(void);
void CaptureSensorPluseCounter(void);
float Get_Low_Smooth_Counter(void);
float Get_High_Smooth_Counter(void);
float Get_Low_Counter(void);
float Get_High_Counter(void);
void Get_Bendi_Value(char time);
float Netro_Calibration(u32 time,u32 refdata,u8 channel);
u16 CalcAlarmState(LP_SYSTEM_STTAE *me);
void RefreshParam(void);
u8 GetSensorType(void);

#endif