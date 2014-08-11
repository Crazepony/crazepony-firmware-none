#ifndef _Led_H_
#define _Led_H_
#include "stm32f10x.h"

#define LedA_on    GPIO_SetBits(GPIOA, GPIO_Pin_11)
#define LedA_off   GPIO_ResetBits(GPIOA, GPIO_Pin_11)


#define LedB_on    GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define LedB_off   GPIO_ResetBits(GPIOA, GPIO_Pin_8)


#define LedC_on    GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define LedC_off   GPIO_ResetBits(GPIOB, GPIO_Pin_1)


#define LedD_on    GPIO_SetBits(GPIOB, GPIO_Pin_3)
#define LedD_off   GPIO_ResetBits(GPIOB, GPIO_Pin_3)


void LedInit(void);   //Led初始化函数外部声明



#endif

