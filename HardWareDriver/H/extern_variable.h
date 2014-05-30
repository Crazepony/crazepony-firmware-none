#ifndef _EXTERN_VARIABLE_H_
#define _EXTERN_VARIABLE_H_
#include "NRF24L01.h"
 
 
 
//系统
extern uint8_t SystemReady_OK;					//系统初始化完成标志
extern uint8_t FLY_ENABLE;						//飞行开关
extern uint8_t GYRO_OFFSET_OK;					//稳态误差计算完成标志
extern uint8_t ACC_OFFSET_OK;
extern uint8_t IIC_ERROR_CNT;					//iic错误计数器,每次tim3中断加1,如果读取成功置0
extern uint8_t	I2C2_BUSY;
volatile extern uint32_t	TIM3_IRQCNT;			//TIM3中断计数器
volatile extern uint32_t	TIM2_IRQCNT;			//TIM3中断计数器
volatile extern uint8_t 	MPU6050_I2CData_Ready;		//mpu6050读取完成标志,=1表示读取完成
extern uint32_t While1_Lasttime;			             	//存储while循环的时间


//2.4G发送接收数组
extern uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];		//nrf接收的数据
extern uint8_t NRF24L01_TXDATA[TX_PLOAD_WIDTH];		//nrf需要发送的数据


//RC遥控
typedef struct int16_rcget
{
    float ROOL;
    float PITCH;
    int THROTTLE;
    int YAW;
}RC_GETDATA;


extern RC_GETDATA RC_DATA;//经过处理的RC数据
                
                
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


extern S_INT16_XYZ 	MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		//最新一次读取值
extern S_INT16_XYZ 	GYRO_OFFSET,ACC_OFFSET;			        //零漂


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
extern uint32_t	IMU_CYCTIME;	//两次读取systick时的计数值差值
                
                
#endif
                
        



