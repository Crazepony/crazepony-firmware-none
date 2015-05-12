#ifndef _tim_H_
#define _tim_H_
#include "stm32f10x.h"

void TIM4_Init(char clock,int Preiod);//用于监测系统
void TIM3_Init(char clock,int Preiod);//定时器3的初始化
void TimerNVIC_Configuration(void);//定时器中断向量表配置

extern volatile uint16_t anyCnt,anyCnt2,loop100HzCnt,loop200HzCnt;
extern uint8_t  loop200HzFlag,loop500HzFlag,loop50HzFlag,loop600HzFlag,loop100HzFlag,loop10HzFlag,loop20HzFlag;

#endif

