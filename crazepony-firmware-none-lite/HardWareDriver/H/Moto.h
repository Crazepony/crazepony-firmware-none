#ifndef _MOTO_H_
#define _MOTO_H_
#include "extern_variable.h"

#define Moto_PwmMax 999

void MotorPwmFlash(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM);
void MotorInit(void);

#endif

