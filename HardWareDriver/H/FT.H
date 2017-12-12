#ifndef _FT_H_
#define _FT_H_
#include <stm32f10x.h>
#include "control.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "config.h"

#define FMTISensorAdd_I2C			 	(218)

void Initial_FMTI_Sensor(void);
void updateFBM320(void);

//float Rel_Altitude(int32_t Press);

static int quickMedianFilter3(int32_t * v);
static uint32_t recalculateBarometerTotal(uint32_t pressureTotal, int32_t newPressureReading);
static int applyBarometerMedianFilter(int32_t newPressureReading);
//static int Abs_Altitude(int32_t Press);
static void FMTISensor_Write_U8(uint8_t add, uint8_t cmd);
static uint8_t FMTISensor_Read_U8(uint8_t add);
static uint32_t FMTISensor_ReadADC_U32(void);
static void FMTISensor_ReadCoefficient(void);
static void FMTISensor_Calculate(int32_t UP, int32_t UT);

extern int relPressData;
extern u8 okFbm320;

#endif
