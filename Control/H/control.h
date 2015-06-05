#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f10x.h"


//#define Debug  //调试与否的条件编译

//待速转
#define SLOW_THRO 200
//定义飞机最大倾斜角度
#define  Angle_Max  40.0
#define  YAW_RATE_MAX  180.0f/M_PI_F		//deg/s  
//纠正姿态误差，可以用来抵抗重心偏移等带来的初始不平衡
//#define  Rool_error_init   7      //如果飞机起飞朝左偏，Rool_error_init朝正向增大修改;朝右偏，Rool_error_init朝负向增大修改
//#define  Pitch_error_init  -5      //如果飞机起飞朝前偏，Pitch_error_init朝负向增大修改;朝后偏，Pitch_error_init朝正向增大修改
//定高部分
#define LAND_SPEED						1.2f		//m/s^2
#define ALT_VEL_MAX 					4.0f
#define THR_MIN								0.42f		//min thrust ，根据机重和最小降速而定，用于下降速度过大时，油门过小，导致失衡。再增加fuzzy control ，在油门小时用更大的姿态参数


#define HOVER_THRU	         -0.63  //-0.5f  //悬停


enum {CLIMB_RATE=0,MANUAL,LANDING};
extern uint8_t altCtrlMode;
extern float hoverThrust;
extern uint8_t zIntReset;
extern uint8_t offLandFlag;
extern float altLand;
extern uint8_t isAltLimit;
extern float thrustZSp,thrustZInt;

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
    float Integ;
		float iLimit;
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

void CtrlAttiAng(void);
void CtrlAttiRate(void);
void CtrlAlti(void);
void CtrlAltiVel(void);
void CtrlMotor(void);
void CtrlTest(void);
void CtrlAttiRateNew(void);
void CtrlAttiNew(void);

void SetHeadFree(uint8_t on);

extern u16 PIDWriteBuf[3];//写入flash的临时数字，由NRF24L01_RXDATA[i]赋值 

extern PID_Typedef pitch_angle_PID;	  //pitch角度环的PID
extern PID_Typedef pitch_rate_PID;		//pitch角速率环的PID

extern PID_Typedef roll_angle_PID;    //roll角度环的PID
extern PID_Typedef roll_rate_PID;     //roll角速率环的PID

extern PID_Typedef yaw_angle_PID;     //yaw角度环的PID  
extern PID_Typedef yaw_rate_PID;      //yaw的角速率环的PID  

extern PID_Typedef	alt_PID;
extern PID_Typedef alt_vel_PID;


extern float gyroxGloble;
extern float gyroyGloble;

extern volatile unsigned char motorLock;
#endif


