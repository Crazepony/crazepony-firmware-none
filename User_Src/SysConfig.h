#ifndef _SYS_CONFIG_H
#define _SYS_CONFIG_H
//
#include "stm32f10x.h"

#define AUTO_MW
#define YAW_CORRECT
#define IMU_SW
#define HIGH_FREQ_CTRL
#define NEW_RC
#define lostRC_Landing
 
enum {SRC_PC,SRC_APP};
extern uint8_t btSrc;
//
//typedef struct SystemConfig_tt
//{
//	;
//}SystemConfig_t;


#endif

