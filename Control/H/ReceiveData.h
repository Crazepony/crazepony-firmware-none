#ifndef _ReceiveData_H_
#define _ReceiveData_H_
#include "stm32f10x.h"


//RC遥控
typedef struct int16_rcget
{
    float ROOL;
    float PITCH;
    float THROTTLE;
    float YAW;
}RC_GETDATA;


extern RC_GETDATA RC_DATA,RC_DATA_RAW;//经过处理的RC数据
extern uint8_t FLY_ENABLE;//飞行使能端  7/-5    14/15

void ReceiveDataFormNRF(void);
void ReceiveDataFormUART(void);
void Send_PIDToPC(void);
void Send_AtitudeToPC(void);
//extern int  Rool_error_init;     //如果飞机起飞朝左偏，Rool_error_init朝正向增大修改;朝右偏，Rool_error_init朝负向增大修改
//extern int  Pitch_error_init;     //如果飞机起飞朝前偏，Pitch_error_init朝负向增大修改;朝吼偏，Pitch_error_init朝正向增大修改
void NRFmatching(void);


#endif

