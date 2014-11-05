#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f10x.h"

#define Debug  //调试与否的条件编译



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


//写入Flash参数结构体
typedef struct
{
  u16 WriteBuf[10];       //写入flash的临时数组
  u16 ReadBuf[10];        //读取Flash的临时数组
  
}Parameter_Typedef;


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


extern Parameter_Typedef PIDParameter;//实例化一个PID的Flash参数
extern Parameter_Typedef BTParameter; //实例化一个蓝牙Flash参数


extern float gyroxGloble;
extern float gyroyGloble;
#endif


