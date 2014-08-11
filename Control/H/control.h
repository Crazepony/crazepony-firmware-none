#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f10x.h"


//定义PID参数
typedef struct PID
{
    float P,
          POUT,
          I,
          IOUT,
          D,
          DOUT,
          IMAX,
          SetPoint,
          NowPoint,
          LastError,
          PrerError;

}PID;

void Controler(void);
void PID_INIT(void);
void PID_Calculate(void);
char  ParameterWrite(void);
void  ParameterRead(void);

extern u16 PIDWriteBuf[3];//写入flash的临时数字，由NRF24L01_RXDATA[i]赋值 
extern u16 PRWriteBuf[2];//写入flash的临时数字，由NRF24L01_RXDATA[i]赋值 
extern u16 PowerCouter[3];//开机次数统计值
extern PID  PID_RP;//定义一个PIID结构体
extern float Yaw_Init;
#endif


