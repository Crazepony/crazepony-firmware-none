#ifndef _Battery_H_
#define _Battery_H_
#include "stm32f10x.h"


void BatteryCheckInit(void);	
u16 Get_Adc_Average(u8 ch,u8 times);             
                
#endif
                
        



