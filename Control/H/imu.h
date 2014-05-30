#ifndef _IMU_H_
#define _IMU_H_
#include "stm32f10x.h"

void IMU_DataPrepare(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) ;

void IMU_Output(void);
void GET_EXPRAD(void);

#endif

