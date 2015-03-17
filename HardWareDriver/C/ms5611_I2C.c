/*
  October 2012

  aq32Plus Rev -

  Copyright (c) 2012 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the AQ32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/
///////////////////////////////////////////////////////////////////////////////

#include "config.h"

///////////////////////////////////////////////////////////////////////////////

#define _MS5611_DEBUG_

#ifdef _MS5611_DEBUG_

#define DBG_PRINT(fmt, args...) 	\
    do{\
				printf(""fmt"\r\n",##args);\
    }while(0)
#else

#define DBG_PRINT(fmt, args...)

#endif


///////////////////////////////////////

//#define OSR  256  // 0.60 mSec conversion time (1666.67 Hz)
//#define OSR  512  // 1.17 mSec conversion time ( 854.70 Hz)
//#define OSR 1024  // 2.28 mSec conversion time ( 357.14 Hz)
//#define OSR 2048  // 4.54 mSec conversion time ( 220.26 Hz)
#define OSR 4096  // 9.04 mSec conversion time ( 110.62 Hz)

///////////////////////////////////////

I2C_TypeDef *ms5611I2C;
uint8_t     ms5611Address;

uint16andUint8_t c1, c2, c3, c4, c5, c6;

uint32andUint8_t d1;

uint32_t d1Value;

uint32andUint8_t d2;

uint32_t d2Value;

int32_t dT;

int32_t ms5611Temperature;

uint8_t newPressureReading = false;

uint8_t newTemperatureReading = false;

///////////////////////////////////////////////////////////////////////////////
// Read Temperature Request Pressure
///////////////////////////////////////////////////////////////////////////////

void readTemperatureRequestPressure(void)
{
    uint8_t data[3];

    IICreadBytes( ms5611Address, 0x00, 3, data);    // Request temperature read

    d2.bytes[2] = data[0];
    d2.bytes[1] = data[1];
    d2.bytes[0] = data[2];
	
		DBG_PRINT("d2: 0x%2x%2x%2x\n",d2.bytes[0],d2.bytes[1],d2.bytes[2]);

    #if   (OSR ==  256)
				IICwriteOneByte( ms5611Address, 0x40);  // Request pressure conversion
		#elif (OSR ==  512)
				IICwriteOneByte( ms5611Address, 0x42);
		#elif (OSR == 1024)
				IICwriteOneByte( ms5611Address, 0x44);
		#elif (OSR == 2048)
				IICwriteOneByte( ms5611Address, 0x46);
		#elif (OSR == 4096)
				IICwriteOneByte( ms5611Address, 0x48);
    #endif
}

///////////////////////////////////////////////////////////////////////////////
// ReadPressureRequestPressure
///////////////////////////////////////////////////////////////////////////////

void readPressureRequestPressure(void)
{
    uint8_t data[3];

    IICreadBytes( ms5611Address, 0x00, 3, data);    // Request pressure read

    d1.bytes[2] = data[0];
    d1.bytes[1] = data[1];
    d1.bytes[0] = data[2];
	
		DBG_PRINT("d1: 0x%2x%2x%2x\n",d1.bytes[0],d1.bytes[1],d1.bytes[2]);

    #if(OSR ==  256)
				IICwriteOneByte( ms5611Address, 0x40);  // Request pressure conversion
		#elif (OSR ==  512)
			IICwriteOneByte( ms5611Address, 0x42);
		#elif (OSR == 1024)
				IICwriteOneByte( ms5611Address, 0x44);
		#elif (OSR == 2048)
				IICwriteOneByte( ms5611Address, 0x46);
		#elif (OSR == 4096)
				IICwriteOneByte( ms5611Address, 0x48);
    #endif
}

///////////////////////////////////////////////////////////////////////////////
// Read Pressure Request Temperature
///////////////////////////////////////////////////////////////////////////////

void readPressureRequestTemperature(void)
{
    uint8_t data[3];

    IICreadBytes( ms5611Address, 0x00, 3, data);    // Request pressure read

    d1.bytes[2] = data[0];
    d1.bytes[1] = data[1];
    d1.bytes[0] = data[2];
	
		DBG_PRINT("d1: 0x%2x%2x%2x\n",d1.bytes[0],d1.bytes[1],d1.bytes[2]);

    #if   (OSR ==  256)
	    IICwriteOneByte( ms5611Address, 0x50);   // Request temperature converison
	#elif (OSR ==  512)
	    IICwriteOneByte( ms5611Address, 0x52);
	#elif (OSR == 1024)
	    IICwriteOneByte( ms5611Address, 0x54);
	#elif (OSR == 2048)
	    IICwriteOneByte( ms5611Address, 0x56);
	#elif (OSR == 4096)
	    IICwriteOneByte( ms5611Address, 0x58);
    #endif
}

///////////////////////////////////////////////////////////////////////////////
// Calculate Temperature
///////////////////////////////////////////////////////////////////////////////

void calculateTemperature(void)
{
    dT                = (int32_t)d2Value - ((int32_t)c5.value << 8);
    ms5611Temperature = 2000 + (int32_t)(((int64_t)dT * c6.value) >> 23);
	
		DBG_PRINT("TEMP: %d",ms5611Temperature);
}

///////////////////////////////////////////////////////////////////////////////
// Calculate Pressure Altitude
///////////////////////////////////////////////////////////////////////////////

void calculatePressureAltitude(void)
{
	float pressureAlt50Hz;	//最后计算得到的高度值

	int64_t offset;
	int64_t offset2 = 0;

	int64_t sensitivity;
	int64_t sensitivity2 = 0;

	int64_t f;

	int32_t p;

	int32_t ms5611Temp2  = 0;

	offset      = ((int64_t)c2.value << 16) + (((int64_t)c4.value * dT) >> 7);
	sensitivity = ((int64_t)c1.value << 15) + (((int64_t)c3.value * dT) >> 8);

	if (ms5611Temperature < 2000)
	{
		ms5611Temp2  = SQR(dT) >> 31;

		f	 		 = SQR(ms5611Temperature - 2000);
		offset2      = 5 * f >> 1;
		sensitivity2 = 5 * f >> 2;

		if (ms5611Temperature < -1500)
		{
			f 			  = SQR(ms5611Temperature + 1500);
			offset2      +=  7 * f;
			sensitivity2 += 11 * f >> 1;
		}

		ms5611Temperature -= ms5611Temp2;

		offset -= offset2;
		sensitivity -= sensitivity2;
	}

	p = (((d1Value * sensitivity) >> 21) - offset) >> 15;
	
	DBG_PRINT("PRESS : %d",p);

	pressureAlt50Hz = 44330.0f * (1.0f - pow((float)p / 101325.0f, 1.0f / 5.255f));

	DBG_PRINT("calculate Pressure Altitude : %f\n",pressureAlt50Hz);
}

///////////////////////////////////////////////////////////////////////////////
// Pressure Initialization
///////////////////////////////////////////////////////////////////////////////

void initPressure(void)
{
	
    uint8_t data[2];

    ms5611I2C = I2C1;
    ms5611Address = 0xEE;	 //0XEE为MS5611的八位地址，0X77为MS5611的七位地址，IIC读写函数输入参数为八位地址

		DBG_PRINT("init pressure MS5611\n");
	
    IICwriteOneByte( ms5611Address, 0x1E);      // Reset Device

    delay_ms(10);

    IICreadBytes( ms5611Address, 0xA2, 2, data);    // Read Calibration Data C1
    c1.bytes[1] = data[0];
    c1.bytes[0] = data[1];

    IICreadBytes( ms5611Address, 0xA4, 2, data);    // Read Calibration Data C2
    c2.bytes[1] = data[0];
    c2.bytes[0] = data[1];

    IICreadBytes( ms5611Address, 0xA6, 2, data);    // Read Calibration Data C3
		c3.bytes[1] = data[0];
    c3.bytes[0] = data[1];

    IICreadBytes( ms5611Address, 0xA8, 2, data);    // Read Calibration Data C4
		c4.bytes[1] = data[0];
    c4.bytes[0] = data[1];

    IICreadBytes( ms5611Address, 0xAA, 2, data);    // Read Calibration Data C5
		c5.bytes[1] = data[0];
    c5.bytes[0] = data[1];

    IICreadBytes( ms5611Address, 0xAC, 2, data);    // Read Calibration Data C6
		c6.bytes[1] = data[0];
    c6.bytes[0] = data[1];
		
		DBG_PRINT("c1:0x%2x%2x\n",c1.bytes[0],c1.bytes[1]);
		DBG_PRINT("c2:0x%2x%2x\n",c2.bytes[0],c2.bytes[1]);
		DBG_PRINT("c3:0x%2x%2x\n",c3.bytes[0],c3.bytes[1]);
		DBG_PRINT("c4:0x%2x%2x\n",c4.bytes[0],c4.bytes[1]);
		DBG_PRINT("c5:0x%2x%2x\n",c5.bytes[0],c5.bytes[1]);
		DBG_PRINT("c6:0x%2x%2x\n",c6.bytes[0],c6.bytes[1]);

    #if   (OSR ==  256)
				IICwriteOneByte( ms5611Address, 0x50);  // Request temperature conversion
		#elif (OSR ==  512)
				IICwriteOneByte( ms5611Address, 0x52);
		#elif (OSR == 1024)
				IICwriteOneByte( ms5611Address, 0x54);
		#elif (OSR == 2048)
				IICwriteOneByte( ms5611Address, 0x56);
		#elif (OSR == 4096)
				IICwriteOneByte( ms5611Address, 0x58);
    #endif


    delay_ms(10);

    readTemperatureRequestPressure();
    delay_ms(10);

    readPressureRequestTemperature();
    delay_ms(10);

    d1Value = d1.value;
    d2Value = d2.value;

    calculateTemperature();
    calculatePressureAltitude();

		DBG_PRINT("init pressure MS5611 finished\n");
}

///////////////////////////////////////////////////////////////////////////////
