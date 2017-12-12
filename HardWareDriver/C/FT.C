#include "FT.H"
#include "MS5611.h"

int relPressData = 0;

static int baroPressureSum = 0;
uint32_t timeCount = 0;
u8 okFbm320 = 0;

struct FMTI_Sensor
{
    int32_t UP;
    int32_t UT;
    int32_t RP;
    int32_t RT;
    int32_t Reff_P;
    uint16_t C0, C1, C2, C3, C6, C8, C9, C10, C11, C12;
    uint32_t C4, C5, C7;
} FBM320;



void updateFBM320(void)
{
	static long time;
	static u8 status = 0;
	switch(status)
	{
		case 0:
			FMTISensor_Write_U8(0xF4, 0x2E);
			time = micros();
			status = 1;
		break;
		case 1:
			if(micros() - time > 2200)
			{
				FBM320.UT = FMTISensor_ReadADC_U32();
				FMTISensor_Write_U8(0xF4, 0xF4);
				time = micros();
				status = 2;
			}
		break;
		case 2:
			if(micros() - time > 9800)
			{
				FBM320.UP = FMTISensor_ReadADC_U32();	
				FMTISensor_Write_U8(0xF4, 0x2E);
				time = micros();
				FMTISensor_Calculate(FBM320.UP, FBM320.UT);
				//printf("RP:%d\r\n",FBM320.RP);
				if(timeCount < 5)
				{
					FBM320.Reff_P = FBM320.RP;
					paOffsetInited = 1;
				}
				baroPressureSum = recalculateBarometerTotal(baroPressureSum,FBM320.RP);
				if(timeCount > 5)
				{
					relPressData = baroPressureSum/5 - FBM320.Reff_P;
					//relPressData = FBM320.RP - FBM320.Reff_P;
				//relPressData = ((relPressData*0.1) + ((baroPressureSum/20 - FBM320.Reff_P)*0.9)+0.49);
				//printf("Press:%d\r\n",relPressData);
				}
				Baro_ALT_Updated = 0xff;
				timeCount++;
				if(timeCount > 200)
				{
					timeCount = 6;
					if(relPressData > 0)
					{
						FBM320.Reff_P++;
					}else
					{
						FBM320.Reff_P--;
					}
				}				
				status = 1;
			}
		break;
	}
}

void Initial_FMTI_Sensor()														//Initial FBM320, used SPI or I2C bus protocol
{
    delay_ms(15);																					//FBM320 power on settling time, 15 ms
    FMTISensor_ReadCoefficient();															//Read FBM320 coefficient
}
void FMTISensor_Write_U8(uint8_t add, uint8_t cmd)	//FMTI sensor write unsigned char function
{
    IICwriteByte(FMTISensorAdd_I2C, add, cmd);
}
uint8_t FMTISensor_Read_U8(uint8_t add)						//FMTI sensor read unsigned char function
{
    return I2C_ReadOneByte(FMTISensorAdd_I2C, add);
}
uint32_t FMTISensor_ReadADC_U32()										//FMTI sensor read ADC value function
{
    return	((uint32_t)I2C_ReadOneByte(FMTISensorAdd_I2C, 0xF6) << 16) | ((uint16_t)I2C_ReadOneByte(FMTISensorAdd_I2C, 0xF7) << 8) | I2C_ReadOneByte(FMTISensorAdd_I2C, 0xF8);
}
void FMTISensor_ReadCoefficient()										//FMTI sensor receive R0~R9 and calibrate coefficient C0~C12
{
    uint8_t i;
    uint16_t R[10]= {0};

    for(i=0; i<9; i++)
    {
        R[i] = ((uint8_t)FMTISensor_Read_U8(0xAA + (i*2)) << 8) | FMTISensor_Read_U8(0xAB + (i*2));
    }
    R[9] = ((uint8_t)FMTISensor_Read_U8(0xA4) << 8) | FMTISensor_Read_U8(0xF1);


    /*	Use R0~R9 calculate C0~C12 of FBM320-02	*/
    FBM320.C0 = R[0] >> 4;
    FBM320.C1 = ((R[1] & 0xFF00) >> 5) | (R[2] & 7);
    FBM320.C2 = ((R[1] & 0xFF) << 1) | (R[4] & 1);
    FBM320.C3 = R[2] >> 3;
    FBM320.C4 = ((uint32_t)R[3] << 2) | (R[0] & 3);
    FBM320.C5 = R[4] >> 1;
    FBM320.C6 = R[5] >> 3;
    FBM320.C7 = ((uint32_t)R[6] << 3) | (R[5] & 7);
    FBM320.C8 = R[7] >> 3;
    FBM320.C9 = R[8] >> 2;
    FBM320.C10 = ((R[9] & 0xFF00) >> 6) | (R[8] & 3);
    FBM320.C11 = R[9] & 0xFF;
    FBM320.C12 = ((R[0] & 0x0C) << 1) | (R[7] & 7);


}
void FMTISensor_Calculate(int32_t UP, int32_t UT)										//FMTI sensor calculate Real pressure & temperautre
{
    int32_t DT, DT2, X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32, CF, PP1, PP2, PP3, PP4;

    DT	=	((UT - 8388608) >> 4) + (FBM320.C0 << 4);
    X01	=	(FBM320.C1 + 4459) * DT >> 1;
    X02	=	((((FBM320.C2 - 256) * DT) >> 14) * DT) >> 4;
    X03	=	(((((FBM320.C3 * DT) >> 18) * DT) >> 18) * DT);
    FBM320.RT	=	((2500 << 15) - X01 - X02 - X03) >> 15;

    DT2	=	(X01 + X02 + X03) >> 12;

    X11	=	((FBM320.C5 - 4443) * DT2);
    X12	=	(((FBM320.C6 * DT2) >> 16) * DT2) >> 2;
    X13	=	((X11 + X12) >> 10) + ((FBM320.C4 + 120586) << 4);

    X21	=	((FBM320.C8 + 7180) * DT2) >> 10;
    X22	=	(((FBM320.C9 * DT2) >> 17) * DT2) >> 12;
    X23 = (X22 >= X21) ? (X22 - X21) : (X21 - X22);

    X24	=	(X23 >> 11) * (FBM320.C7 + 166426);
    X25	=	((X23 & 0x7FF) * (FBM320.C7 + 166426)) >> 11;
    X26 = (X21 >= X22) ? (((0 - X24 - X25) >> 11) + FBM320.C7 + 166426) : (((X24 + X25) >> 11) + FBM320.C7 + 166426);

    PP1	=	((UP - 8388608) - X13) >> 3;
    PP2	=	(X26 >> 11) * PP1;
    PP3	=	((X26 & 0x7FF) * PP1) >> 11;
    PP4	=	(PP2 + PP3) >> 10;

    CF	=	(2097152 + FBM320.C12 * DT2) >> 3;
    X31	=	(((CF * FBM320.C10) >> 17) * PP4) >> 2;
    X32	=	(((((CF * FBM320.C11) >> 15) * PP4) >> 18) * PP4);
    FBM320.RP	=	((X31 + X32) >> 15) + PP4 + 99880;

}

float Rel_Altitude(int32_t Press)										//Calculate relative altitude
{
	float BaroAlt_tmp;
	static float alt = 0;
	static u8 flag = 0;
	BaroAlt_tmp = 44330 * (1 - powf((float)Press / (float)FBM320.Reff_P, 0.190295));
	if(flag)
	{
		alt = alt*0.8 + BaroAlt_tmp*0.2;		
	}else
	{
		alt = BaroAlt_tmp;
		flag = 1;		
	}
	return alt;
	
}

int32_t applyBarometerMedianFilter(int32_t newPressureReading)
{
    static int32_t barometerFilterSamples[3];
    static int currentFilterSampleIndex = 0;
    static u8 medianFilterReady = 0;
    int nextSampleIndex;
    
    nextSampleIndex = (currentFilterSampleIndex + 1);
    if (nextSampleIndex == 3) {
        nextSampleIndex = 0;
        medianFilterReady = 1;
    }

    barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
    currentFilterSampleIndex = nextSampleIndex;
    
    if (medianFilterReady)
        return quickMedianFilter3(barometerFilterSamples);
    else
        return newPressureReading;
}
int32_t quickMedianFilter3(int32_t * v)
{
	if(v[0] > v[1])
	{
		if(v[0] > v[2])
		{
			return (v[1]>v[2]?v[1]:v[2]);
		}		
	}
	else
	{
		if(v[0] < v[2])
		{
			return (v[1]>v[2]?v[2]:v[1]);
		}
	}
	return v[0];
}

uint32_t recalculateBarometerTotal(uint32_t pressureTotal, int32_t newPressureReading)
{
    static int32_t barometerSamples[48];
    static int currentSampleIndex = 0;
    int nextSampleIndex;

    // store current pressure in barometerSamples
    nextSampleIndex = (currentSampleIndex + 1);
    if (nextSampleIndex == 6) {
        nextSampleIndex = 0;
    }
    barometerSamples[currentSampleIndex] = applyBarometerMedianFilter(newPressureReading);

    // recalculate pressure total
    // Note, the pressure total is made up of baroSampleCount - 1 samples - See PRESSURE_SAMPLE_COUNT
    pressureTotal += barometerSamples[currentSampleIndex];
    pressureTotal -= barometerSamples[nextSampleIndex];

    currentSampleIndex = nextSampleIndex;

    return pressureTotal;
}

int32_t Abs_Altitude(int32_t Press)																	//Calculate absolute altitude
{
	int8_t P0;			
	int16_t hs1, dP0;			
	int32_t h0, hs0, HP1, HP2;			
					
	if(Press >= 103000)
	{	
		P0	=	103;
		h0	=	-138507;
		hs0	=	-21007;
		hs1	=	311;
	}	
	else if(Press >= 98000)
	{	
		P0	=	98;
		h0	=	280531;
		hs0	=	-21869;
		hs1	=	338;
	}	
	else if(Press >= 93000)
	{	
		P0	=	93;
		h0	=	717253;
		hs0	=	-22813;
		hs1	=	370;
	}	
				
	else if(Press >= 88000)
	{	
		P0	=	88;
		h0	=	1173421;
		hs0	=	-23854;
		hs1	=	407;
	}	
	else if(Press >= 83000)
	{	
		P0	=	83;
		h0	=	1651084;
		hs0	=	-25007;
		hs1	=	450;
	}	
	else if(Press >= 78000)
	{	
		P0	=	78;
		h0	=	2152645;
		hs0	=	-26292;
		hs1	=	501;
	}	
	else if(Press >= 73000)
	{	
		P0	=	73;
		h0	=	2680954;
		hs0	=	-27735;
		hs1	=	560;
	}	
	else if(Press >= 68000)
	{	
		P0	=	68;
		h0	=	3239426;
		hs0	=	-29366;
		hs1	=	632;
	}	
	else if(Press >= 63000)
	{	
		P0	=	63;
		h0	=	3832204;
		hs0	=	-31229;
		hs1	=	719;
	}	
	else if(Press >= 58000)
	{	
		P0	=	58;
		h0	=	4464387;
		hs0	=	-33377;
		hs1	=	826;
	}	
	else if(Press >= 53000)
	{	
		P0	=	53;
		h0	=	5142359;
		hs0	=	-35885;
		hs1	=	960;
	}		
	else if(Press >= 48000)
	{	
		P0	=	48;
		h0	=	5874268;
		hs0	=	-38855;
		hs1	=	1131;
	}	
	else if(Press >= 43000)
	{	
		P0	=	43;
		h0	=	6670762;
		hs0	=	-42434;
		hs1	=	1354;
	}	
	else if(Press >= 38000)
	{	
		P0	=	38;
		h0	=	7546157;
		hs0	=	-46841;
		hs1	=	1654;
	}	
	else if(Press >= 33000)
	{	
		P0	=	33;
		h0	=	8520395;
		hs0	=	-52412;
		hs1	=	2072;
	}	
	else
	{	
		P0	=	28;
		h0	=	9622536;
		hs0	=	-59704;
		hs1	=	2682;
	}
					
	dP0	=	Press - P0 * 1000;
				
	HP1	=	(hs0 * dP0) >> 2;
	HP2	=	(((hs1 * dP0) >> 10)* dP0) >> 4;			

	return	((h0 << 6) + HP1 + HP2) >> 6;										//Return absolute altitude
}

