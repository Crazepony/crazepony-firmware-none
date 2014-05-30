#ifndef __config_H
#define __config_H


#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0)

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#define   true 1
#define   false 0 
#define   bool  uint8_t

#define MPU6050_READRATE			1000	//6050读取频率
#define MPU6050_READTIME			0.001	//6050读取时间间隔
#define EE_6050_ACC_X_OFFSET_ADDR	0
#define EE_6050_ACC_Y_OFFSET_ADDR	1
#define EE_6050_ACC_Z_OFFSET_ADDR	2
#define EE_6050_GYRO_X_OFFSET_ADDR	3
#define EE_6050_GYRO_Y_OFFSET_ADDR	4
#define EE_6050_GYRO_Z_OFFSET_ADDR	5




#include "stm32f10x.h"
#include "stm32f10x_pwr.h"
#include "delay.h"
#include "Led.h"
#include "extern_variable.h"
#include "MPU6050.h"
#include "NRF24L01.h"
#include "Moto.h"
#include "Tim.h"
#include "IIC.h"
#include "sys_fun.h"
#include "SPI.h"
#include "control.h"
#include "stmflash.h"
#include "imu.h"
#include "math.h"
#include "stdio.h"
#include "UART1.h"
#include "stm32f10x_usart.h"
#include "ReceiveData.h"
#include "Battery.h"




#endif

//------------------End of File----------------------------
