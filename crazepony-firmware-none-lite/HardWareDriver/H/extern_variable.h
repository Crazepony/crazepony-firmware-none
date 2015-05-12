#ifndef _EXTERN_VARIABLE_H_
#define _EXTERN_VARIABLE_H_
#include "NRF24L01.h"
 
//
enum{BT=0,NFRC,PC,APP};

#define RC_SRC	BT	//BT or NFRC
//#define BT_SRC  PC	//APP or PC
#define BT_SRC_PC			//both ok
#define BT_SRC_APP

extern  uint8_t accUpdated;
 
 
//系统
extern uint8_t SystemReady_OK;					//系统初始化完成标志
extern uint8_t FLY_ENABLE;					

//飞行开关
extern uint8_t IIC_ERROR_CNT;					//iic错误计数器,每次tim3中断加1,如果读取成功置0
extern uint8_t	I2C2_BUSY;
volatile extern uint32_t	TIM3_IRQCNT;			//TIM3中断计数器
volatile extern uint32_t	TIM2_IRQCNT;			//TIM3中断计数器
volatile extern uint8_t 	MPU6050_I2CData_Ready;		//mpu6050读取完成标志,=1表示读取完成

         
//传感器
typedef struct int16_xyz
{
    int16_t X;
    int16_t Y;
    int16_t Z;
}S_INT16_XYZ;


typedef union 
{
    int16_t D[3];
    S_INT16_XYZ V;
    
}U_INT16_XYZ;




//IMU
typedef struct float_xyz
{
    float X;
    float Y;
    float Z;
    
}S_FLOAT_XYZ;


typedef union 
{
    float D[3];
    S_FLOAT_XYZ V;	
    
}U_FLOAT_XYZ;
   

typedef struct float_angle
{
    float Roll;
    float Pitch;
    float Yaw;
}S_FLOAT_ANGLE;
    
             
extern S_FLOAT_XYZ ACC_F,GYRO_F;	//当次转换结果ACC单位为G,GYRO单位为度/秒
extern S_FLOAT_XYZ GYRO_I[3];		//陀螺仪积分

extern S_FLOAT_XYZ DIF_ACC;		//差分加速度
extern S_FLOAT_XYZ EXP_ANGLE;		//期望角度
extern S_FLOAT_XYZ DIF_ANGLE;		//期望角度与实际角度差
extern S_FLOAT_ANGLE Q_ANGLE;		//四元数计算出的角度
extern S_INT16_XYZ ACC_AVG,GYRO_AVG;		//滑动窗口滤波后的ACC平均值和处理后的gyro值
extern S_FLOAT_ANGLE  Q_ANGLE;	          
                
#endif
                
        



