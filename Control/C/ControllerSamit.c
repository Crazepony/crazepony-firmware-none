#include "stm32f10x.h"
#include "ControllerSamit.h"

//----------------------------PID ---------------------------------------//
//void pidInit2(PidObject* pid, const float desired, const float kp,
//             const float ki, const float kd, const float dt)
//{
//  pid->error     = 0;
//  pid->prevError = 0;
//  pid->integ     = 0;
//  pid->deriv     = 0;
//  pid->desired = desired;
//  pid->kp = kp;
//  pid->ki = ki;
//  pid->kd = kd;
//  pid->iLimit    = DEFAULT_PID_INTEGRATION_LIMIT;
//  pid->iLimitLow = -DEFAULT_PID_INTEGRATION_LIMIT;
//  pid->dt        = dt;
//}

float pidUpdate(PidObject* pid, const float measured, const bool updateError)
{
    float output;

    if (updateError)
    {
        pid->error = pid->desired - measured;
    }

    pid->integ += pid->error * pid->dt;
    if (pid->integ > pid->iLimit)
    {
        pid->integ = pid->iLimit;
    }
    else if (pid->integ < pid->iLimitLow)
    {
        pid->integ = pid->iLimitLow;
    }

    pid->deriv = (pid->error - pid->prevError) / pid->dt;

    pid->outP = pid->kp * pid->error;
    pid->outI = pid->ki * pid->integ;
    pid->outD = pid->kd * pid->deriv;

    output = pid->outP + pid->outI + pid->outD;

    pid->prevError = pid->error;

    return output;
}


void pidReset(PidObject* pid)
{
    pid->error     = 0;
    pid->prevError = 0;
    pid->integ     = 0;
    pid->deriv     = 0;
}

//----------------------------------------Atti Controller------------------------//

void AttiCtrlInit()
{

}


