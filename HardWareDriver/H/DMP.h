#ifndef __DMP_H
#define __DMP_H
#include "UART1.h"

#define   ONE_G  9.80665f //related to acc calculate 

#define DMP_CALC_PRD 7	//ms
#define DMP_ACC_SCALE 8192.0f	//4G , 31276/4=8192
#define DMP_GYRO_SCALE 16.4f	//2000deg/s , 31276/2000=16.4f 
 
// MotionApps 2.0 DMP implementation,
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20

#define MPU6050_DMP_CODE_SIZE       1929    // dmpMemory[]
#define MPU6050_DMP_CONFIG_SIZE     192     // dmpConfig[]
#define MPU6050_DMP_UPDATES_SIZE    47      // dmpUpdates[]


/* ================================================================================================ *
 | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
 * ================================================================================================ */

struct DMP_FIFO_map{
int16_t qw;		 // DMP输出的四元数值
int16_t null0;
int16_t qx;
int16_t null1;
int16_t qy;
int16_t null2;
int16_t qz;
int16_t null3;
int16_t GYROx;	// 陀螺仪 X轴 角速度 ADC值
int16_t null4;
int16_t GYROy;  // 陀螺仪 Y轴 角速度 ADC值
int16_t null5;
int16_t GYROz;	// 陀螺仪 Z轴 角速度 ADC值
int16_t null6;
int16_t ACCx;   // 加速度计 X轴 ADC值
int16_t null7;
int16_t ACCy;	  // 加速度计 Y轴 ADC值
int16_t null8;
int16_t ACCz;	  // 加速度计 Z轴 ADC值
int16_t null9;
int16_t null10;

//以下数据由 DMP_Routing 更新。
float  dmp_pitch;  //DMP算出来的俯仰角	单位：度
float  dmp_roll;    //DMP滚转角		   单位：度
float  dmp_yaw;		//DMP 航向角，由于没有磁力计参与，航向角会飘  单位：度
float  dmp_gyrox;	// 陀螺仪 X轴 角速度   单位：度每秒
float  dmp_gyroy;   // 陀螺仪 Y轴 角速度   单位：度每秒
float  dmp_gyroz;   // 陀螺仪 Z轴 角速度   单位：度每秒
float  dmp_accx;	// 加速度计 X轴   单位：g  [9.8 m/S^2]
float  dmp_accy;	// 加速度计 Y轴   单位：g  [9.8 m/S^2]
float  dmp_accz;	// 加速度计 Z轴   单位：g  [9.8 m/S^2]
};

//------------------------------------------------------------------
extern struct DMP_FIFO_map DMP_DATA;  //数据引出				
extern  float q[4];
//当希望读取 陀螺仪的X轴输出时，变量是 DMP_DATA.dmp_gyrox		   -
//当希望读取 陀螺仪的Y轴输出时，变量是 DMP_DATA.dmp_gyroy		   -
//当希望读取 加速度计的X轴输出时，变量是 DMP_DATA.dmp_accx		   -
//载体俯仰角的 变量是 DMP_DATA.dmp_pitch						   -
//其他数据 参考 struct DMP_FIFO_map 结构体的定义				   -
//编写者：lisn3188												   -
//网址：www.chiplab7.net										   -
//作者E-mail：lisn3188@163.com									   -
//------------------------------------------------------------------


//DMP API子程序
uint8_t MPU6050_DMP_Initialize(void); //DMP初始化
void DMPCalibrate(void);
void DMP_Routing(void);	 //DMP 线程，主要用于读取和处理DMP的结果   [需要定时调用]
void DMP_getYawPitchRoll(void);  //读取载体的姿态角

#endif

//------------------End of File----------------------------
