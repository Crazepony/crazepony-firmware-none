#ifndef _IMU_H_
#define _IMU_H_
#include "config.h"
#include <math.h>


#define M_PI_F 3.1415926
#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C		1.225f			/* kg/m^3		*/
#define CONSTANTS_AIR_GAS_CONST				287.1f 			/* J/(kg * K)		*/
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS			-273.15f		/* ?C			*/
#define CONSTANTS_RADIUS_OF_EARTH			6371000			/* meters (m)		*/

//
extern float accFilted[3];
extern float DCMbg[3][3];

void LowPassFilter2p_set_cutoff_frequency(float sample_freq, float cutoff_freq);
float LowPassFilter2p_apply(float sample);

void quat2DCM(float DCM[3][3],float q[4]);

#endif

