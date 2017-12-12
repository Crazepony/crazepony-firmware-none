

#include "AltitudeHold.h"
#include "CommApp.h"
#include "imu.h"

int32_t setVelocity = 0;
uint8_t velocityControl = 0;
int32_t errorVelocityI = 0;
int32_t altHoldThrottleAdjustment = 0;
int32_t AltHold;
int32_t vario = 0;                      // variometer in cm/s
int32_t BaroAlt = 0;
int aa = 0;
static u8 fastChangeFlag = 0;
static int16_t initialRawThrottleHold;
static int16_t initialThrottleHold;
static int32_t EstAlt;                // in cm
int16_t relThrottle;
u8 calibrationEndFlag = 0;
// 40hz update rate (20hz LPF on acc)
#define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25)

#define DEGREES_80_IN_DECIDEGREES 800

static void applyMultirotorAltHold(void)
{
    static uint8_t isAltHoldChanged = 0;
    //printf("throttle:%d,%d\r\n",rcData[THROTTLE],initialRawThrottleHold);
    // multirotor alt hold
    if (fastChangeFlag) {
        // rapid alt changes
        if (fabs(rcData[THROTTLE] - initialRawThrottleHold) > 40) {
            errorVelocityI = 0;
            isAltHoldChanged = 1;
            relThrottle += (rcData[THROTTLE] > 1000) ? -40 : 40;
        } else {
            if (isAltHoldChanged) {
                AltHold = EstAlt;
                isAltHoldChanged = 0;
            }
            if(initialThrottleHold + altHoldThrottleAdjustment < 1000)
            {
                relThrottle = 1000;
            }
            else if(initialThrottleHold + altHoldThrottleAdjustment > 2000)
            {
                relThrottle = 2000;
            }
            else
            {
                relThrottle = initialThrottleHold + altHoldThrottleAdjustment;
            }
        }
    } else {
        // slow alt changes, mostly used for aerial photography
        if (fabs(rcData[THROTTLE] - initialRawThrottleHold) > 40) {
            // set velocity proportional to stick movement +100 throttle gives ~ +50 cm/s
            setVelocity = (rcData[THROTTLE] - initialRawThrottleHold) / 2;
            velocityControl = 1;
            isAltHoldChanged = 1;
        } else if (isAltHoldChanged) {
            AltHold = EstAlt;
            velocityControl = 0;
            isAltHoldChanged = 0;
        }
        if(initialThrottleHold + altHoldThrottleAdjustment < 1000)
        {
            relThrottle = 1000;
        }
        else if(initialThrottleHold + altHoldThrottleAdjustment > 2000)
        {
            relThrottle = 2000;
        }
        else
        {
            relThrottle = initialThrottleHold + altHoldThrottleAdjustment;
        }
    }
    //printf("relTh:%d\r\n",relThrottle);
}

void applyAltHold(void)
{

    applyMultirotorAltHold();

}

void updateAltHoldState(void)
{
    //static u8 baroFlag = 0;
    u8 baroFlag = 0;
    if (!baroFlag) {
//        baroFlag = 1;
//        AltHold = EstAlt;
//        initialRawThrottleHold = rcData[THROTTLE];
//        initialThrottleHold = relThrottle;
        //errorVelocityI = 0;
//        altHoldThrottleAdjustment = 0;
        printf("mother egg!");
    }

}


u8 isThrustFacingDownwards(void)
{
    return fabs(imu.pitch) < DEGREES_80_IN_DECIDEGREES && fabs(imu.roll) < DEGREES_80_IN_DECIDEGREES;
}

int32_t applyDeadband(int32_t value, int32_t deadband)
{
    if (fabs(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

int32_t calculateAltHoldThrottleAdjustment(int32_t vel_tmp, float accZ_tmp, float accZ_old)
{
    int32_t result = 0;
    int32_t error;
    int32_t setVel;

    if (!isThrustFacingDownwards()) {
        return result;
    }

    if(rcData[THROTTLE] > 1460 && rcData[THROTTLE] < 1540)
    {
        AltHold = EstAlt;
    }

    // Altitude P-Controller
    if (!velocityControl) {
        if(AltHold - EstAlt < -500)
        {
            error = -500;
        }
        else if(AltHold - EstAlt > 500)
        {
            error = 500;
        }
        else
        {
            error = AltHold - EstAlt;
        }
        error = applyDeadband(error, 10); // remove small P parameter to reduce noise near zero position
        // limit velocity to +/- 3 m/s
        if((alt_PID.P * error) < -300)
        {
            setVel = -300;
        }
        else if((alt_PID.P * error) > 300)
        {
            setVel = 300;
        }
        else
        {
            setVel = (alt_PID.P * error);
        }

    } else {
        setVel = setVelocity;
    }
    // Velocity PID-Controller
    //printf("setvel:%d\r\n",setVel);
    //printf("setVel:%d, vel_tmp:%d\r\n",setVel, vel_tmp);
    // P
    error = setVel - vel_tmp;
    if((alt_vel_PID.P * error) < -300)
    {
        result = -300;
    }
    else if((alt_vel_PID.P * error) > 300)
    {
        result = 300;
    }
    else
    {
        result = (alt_vel_PID.P * error);
    }
//	printf("error:%d\r\n",error);
//    // I
//    errorVelocityI += (alt_vel_PID.I * error);
//	printf("errorI:%d\r\n",errorVelocityI);
//	if(errorVelocityI < -(200))
//	{
//		errorVelocityI = -(200);
//	}
//	else if(errorVelocityI > (200))
//	{
//		errorVelocityI = (200);
//	}
//    result += errorVelocityI;     // I in range +/-200
    //printf("accZ:%f, old:%f\r\n",accZ_tmp,accZ_old);
    // D
    if(alt_vel_PID.D * (accZ_tmp + accZ_old) < -150)
    {
        result = -150;
    }
    else if(alt_vel_PID.D * (accZ_tmp + accZ_old) > 150)
    {
        result = 150;
    }
    result -= alt_vel_PID.D * (accZ_tmp + accZ_old);

    return result;
}

void calculateEstimatedAltitude(uint32_t currentTime)
{
    float baroVel;
    float vel;
    float alt;
    float accZ_tmp;
    float highError = 0;
    float velError = 0;
    static float alt_old = 0.0f;
    static float accAlt_old = 0.0f;
    static float accZ_old = 0.0f;
    static float accAlt = 0.0f;
    static float lastBaroAlt;
    static float highError_i = 0.0f;
    static float velError_old = 0.0f;

    BaroAlt = -relPressData*5.5;


    //if(calibrationEndFlag)
//	{
//		vel = nav.vz;
//		alt = nav.z;
//		accZ_tmp = nav.az;
//	}
    if(alt < 0.1 && alt > -0.1)
        alt = 0;

    printf("alt:%f\r\n",alt);
    printf("vel:%f\r\n",vel);
    printf("acc:%f\r\n",accZ_tmp);
    accAlt += alt;
    accAlt = accAlt*0.85 + BaroAlt*0.15;

    printf("accAlt:%f\r\n",accAlt);

    highError = accAlt - accAlt_old;
    if(highError < 0.7 && highError > -0.7)
        highError = 0;
    printf("Herror:%f\r\n",highError);
    accAlt_old = accAlt;


//	highError_i += highError*alt_PID.I;

//	baroVel = (BaroAlt - lastBaroAlt);
//	vel = (alt - alt_old);
//
//	velError = (highError*alt_PID.P + highError_i) - (baroVel*0.15 + vel * 0.85);
//
//    lastBaroAlt = BaroAlt;
//	alt_old = alt;
//	//printf("barovel:%d\r\n",baroVel);
//    //baroVel = constrain(baroVel, -1500, 1500);  // constrain baro velocity +/- 1500cm/s
//    baroVel = applyDeadband(baroVel, 10);       // to reduce noise near zero

//    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
//    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay


//    // set vario
//    vario = applyDeadband(vel_tmp, 5);

//    altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment(vel_tmp, accZ_tmp, accZ_old);
//	relThrottle = altHoldThrottleAdjustment;
//    accZ_old = accZ_tmp;
    //printf("altHoldThrottle:%d\r\n",altHoldThrottleAdjustment);
}

int32_t altitudeHoldGetEstimatedAltitude(void)
{
    return EstAlt;
}
