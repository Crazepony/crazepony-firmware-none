#ifndef _SYS_CONFIG_H
#define _SYS_CONFIG_H
//
#include "stm32f10x.h"

#define AUTO_MW
#define YAW_CORRECT
#define IMU_SW		//姿态解算使用软件解算，不再使用MPU6050的硬件解算单元DMP
#define HIGH_FREQ_CTRL
#define NEW_RC
#define lostRC_Landing
//#define STOP_MOTOR_FOREVER

#define UART_DEBUG	//开启改宏，则可以使用串口助手打印调试。否则使用Crazepony上位机

 
enum {SRC_PC,SRC_APP};
extern uint8_t btSrc;
//
//typedef struct SystemConfig_tt
//{
//	;
//}SystemConfig_t;


#endif

