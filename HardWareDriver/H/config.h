#ifndef __config_H
#define __config_H



///////////////////////////////////////

#include "stm32f10x.h"
#include "stm32f10x_pwr.h"
#include "delay.h"
#include "Led.h"
#include "BT.h"
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
#include "math.h"
#include "stdio.h"
#include "UART1.h"
#include "stm32f10x_usart.h"
#include "ReceiveData.h"
#include "Battery.h"
#include "DMP.h"
#include "Battery.h"
#include "HMC5883L.h"


#define SQR(x)  ((x) * (x))

///////////////////////////////////////////////////////////////////////////////
// Misc Type Definitions
///////////////////////////////////////////////////////////////////////////////

typedef union {
    int16_t value;
    uint8_t bytes[2];
} int16andUint8_t;

typedef union {
    int32_t value;
    uint8_t bytes[4];
} int32andUint8_t;

typedef union {
    uint16_t value;
     uint8_t bytes[2];
} uint16andUint8_t;

typedef union {
	uint32_t value;
	 uint8_t bytes[4];
} uint32andUint8_t;

#include "ms5611_I2C.h"


#endif

//------------------End of File----------------------------
