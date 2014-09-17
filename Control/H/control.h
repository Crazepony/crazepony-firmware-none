#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f10x.h"


// //定义PID参数
// typedef struct PID
// {
//     float P,
//           POUT,
//           I,
//           IOUT,
//           D,
//           DOUT,
//           IMAX,
//           SetPoint,
//           NowPoint,
//           LastError,
//           PrerError;

// }PID;


// PID结构体
typedef struct
{
    float P;
    float I;
    float D;
    float Desired;
    float Error;
    float PreError;
    float PrePreError;
    float Increment;
    double Integ;
    float Deriv;
    float Output;
 
}PID_Typedef;


void Controler(void);
void PID_INIT(void);
void PID_Calculate(void);
char  ParameterWrite(void);
void  ParameterRead(void);

extern u16 PIDWriteBuf[3];//写入flash的临时数字，由NRF24L01_RXDATA[i]赋值 

extern PID_Typedef pitch_angle_PID;	//角度环的PID
extern PID_Typedef pitch_rate_PID;		//角速率环的PID

extern PID_Typedef roll_angle_PID;
extern PID_Typedef roll_rate_PID;

extern PID_Typedef yaw_angle_PID;
extern PID_Typedef yaw_rate_PID;


#endif


