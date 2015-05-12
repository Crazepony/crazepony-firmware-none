#ifndef __MS5611_H
#define __MS5611_H

#include "IIC.h"
#include "delay.h"


// addresses of the device CSB =0 
#ifdef DEBUG_HW_I2C
#define MS5611_ADDR         0x77// 0xEE     // default I2C address
#else
#define MS5611_ADDR         0xEE     // default I2C address
#endif

// registers of the device
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00  //Conversion time 0.6ms  Resolution 0.065mbar
#define MS561101BA_OSR_512 0x02  //Conversion time 1.2ms  Resolution 0.042mbar
#define MS561101BA_OSR_1024 0x04 //Conversion time 2.3ms  Resolution 0.027mbar
#define MS561101BA_OSR_2048 0x06 //Conversion time 4.6ms  Resolution 0.018mbar
#define MS561101BA_OSR_4096 0x08 //Conversion time 9.1ms  Resolution 0.012mbar

#define MS561101BA_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.

//Other
#define MSLP                    101325          // Mean Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar)

//  Temperature in 1C
//  Pressure    in 0.01mbar = Pa
//  Altitude    in meter
//  VerticalSpeed in m/s
extern volatile float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude,MS5611_VerticalSpeed;
extern uint8_t Baro_ALT_Updated ; //气压计高度更新完成标志。
extern uint8_t paOffsetInited;
void MS5611_Init(void);
void MS5611_Thread(void);
void MS5611_ThreadNew(void) ;
uint8_t  WaitBaroInitOffset(void);

#endif

//------------------End of File----------------------------
