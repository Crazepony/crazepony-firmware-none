
 /*    
  *      ____                      _____                  +---+
  *     / ___\                     / __ \                 | R |
  *    / /                        / /_/ /                 +---+
  *   / /   ________  ____  ___  / ____/___  ____  __   __
  *  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
  * / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
  * \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
  *                                                 / /
  *                                            ____/ /
  *                                           /_____/
  *                                       
  *  Crazyfile control firmware                                        
  *  Copyright (C) 2011-2014 Crazepony-II                                        
  *
  *  This program is free software: you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License as published by
  *  the Free Software Foundation, in version 3.
  *
  *  This program is distributed in the hope that it will be useful,
  *  but WITHOUT ANY WARRANTY; without even the implied warranty of
  *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  *  GNU General Public License for more details.
  * 
  * You should have received a copy of the GNU General Public License
  * along with this program. If not, see <http://www.gnu.org/licenses/>.
  *
  *
  * debug.c - Debugging utility functions
  *
  */
#include "imu.h"
#include "moto.h"
#include "ReceiveData.h"
#include "extern_variable.h"
#include "math.h"
#include "led.h"


#define pi              __PI
#define angletorad      0.01745329252
#define radtoangle      57.295779513
#define Gyr_Gain 	0.015267                                  
#define ACC_Gain 	0.0011963                                
#define FILTER_NUM	10   //滑动窗口滤波窗口大小



#define Kp 10.0f                        
#define Ki 0.008f                         
#define halfT 0.001f                 




float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   
float exInt = 0, eyInt = 0, ezInt = 0; 


S_FLOAT_ANGLE  Q_ANGLE;	

uint8_t FLY_ENABLE=0;
uint8_t ParameterSet=0;
uint8_t Imu_DataReady=0;	//IMU数据准备好标志
float IMU_CYCTIME_F;		//两次计算的时间间隔,单位为秒
S_FLOAT_XYZ ACC_F,GYRO_F;	
S_FLOAT_XYZ GYRO_I[3];		      //陀螺仪积分
S_INT16_XYZ ACC_AVG,GYRO_AVG;   //滑动窗口滤波后的ACC平均值和处理后的gyro值

S_FLOAT_XYZ DIF_ACC;		//差分加速度
S_FLOAT_XYZ EXP_ANGLE;	//期望角度	
S_FLOAT_XYZ DIF_ANGLE;	//实际与期望相差的角度	

int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];	

 

//函数名：IMU_DataPrepare()
//输入：无
//输出: 无
//描述：欧拉角变换前的准备
//作者：马骏
//备注：没考上研，心情不好
void IMU_DataPrepare(void)
{
    static uint8_t filter_cnt=0;
    int32_t temp1=0,temp2=0,temp3=0;
    uint8_t i;
    
    IMU_CYCTIME_F = (float)IMU_CYCTIME / 9000000;
    ACC_F.X = MPU6050_ACC_LAST.X * ACC_Gain;//本次传感器数据转换成单位格式
    ACC_F.Y = MPU6050_ACC_LAST.Y * ACC_Gain;
    ACC_F.Z = MPU6050_ACC_LAST.Z * ACC_Gain;
    GYRO_F.X = MPU6050_GYRO_LAST.X * Gyr_Gain;//为了和acc一致
    GYRO_F.Y = MPU6050_GYRO_LAST.Y * Gyr_Gain;
    GYRO_F.Z = MPU6050_GYRO_LAST.Z * Gyr_Gain;

    ACC_X_BUF[filter_cnt] = MPU6050_ACC_LAST.X;//更新滑动窗口数组
    ACC_Y_BUF[filter_cnt] = MPU6050_ACC_LAST.Y;
    ACC_Z_BUF[filter_cnt] = MPU6050_ACC_LAST.Z;
    for(i=0;i<FILTER_NUM;i++)
    {
        temp1 += ACC_X_BUF[i];
        temp2 += ACC_Y_BUF[i];
        temp3 += ACC_Z_BUF[i];
    }
    ACC_AVG.X = temp1 / FILTER_NUM;   //均值
    ACC_AVG.Y = temp2 / FILTER_NUM;
    ACC_AVG.Z = temp3 / FILTER_NUM;
    filter_cnt++;
    if(filter_cnt==FILTER_NUM)	filter_cnt=0;

    GYRO_I[2].X = GYRO_I[1].X;				
    GYRO_I[2].Y = GYRO_I[1].Y;
    GYRO_I[2].Z = GYRO_I[1].Z;
    GYRO_I[1].X = GYRO_I[0].X;
    GYRO_I[1].Y = GYRO_I[0].Y;
    GYRO_I[1].Z = GYRO_I[0].Z;
    GYRO_I[0].X += GYRO_F.X * IMU_CYCTIME_F;
    GYRO_I[0].Y += GYRO_F.Y * IMU_CYCTIME_F;
    GYRO_I[0].Z += GYRO_F.Z * IMU_CYCTIME_F;

    Imu_DataReady = 1;
 
}

//函数名：IMU_TEST()
//输入：无
//输出: 无
//描述：将之前准备的欧拉角变换的中间量转换为弧度制式，再输出pitch和roll角，供后续算法调用
//作者：马骏
//备注：没考上研，心情不好
void IMU_Output(void)
{
    if(Imu_DataReady)
    {
        Imu_DataReady = 0;
        IMUupdate(GYRO_F.X*angletorad,GYRO_F.Y*angletorad,GYRO_F.Z*angletorad,ACC_AVG.X,ACC_AVG.Y,ACC_AVG.Z);	//*angletorad转成弧度
    }
}
//函数名：GET_EXPRAD()
//输入：无
//输出: 无
//描述：为PID控制做期望准备
//作者：马骏
//备注：计算期望角度,不加控制时期望角度为0,0
void GET_EXPRAD(void)			
{
    EXP_ANGLE.X = (float)(RC_DATA.ROOL);
    EXP_ANGLE.Y = (float)(RC_DATA.PITCH);
    EXP_ANGLE.Z = (float)(RC_DATA.YAW);
    
    DIF_ANGLE.X = EXP_ANGLE.X - Q_ANGLE.Roll;
    DIF_ANGLE.X = DIF_ANGLE.X*5;
  
    DIF_ANGLE.Y = EXP_ANGLE.Y - Q_ANGLE.Pitch;
    DIF_ANGLE.Y = DIF_ANGLE.Y*5;
    DIF_ACC.Z =   (ACC_AVG.Z/100)-76;//Z轴加速度差分值，保证飞机起飞不会很冲
    
    //纠正平衡点的误差
//     DIF_ANGLE.X = DIF_ANGLE.X ;
//     DIF_ANGLE.Y = DIF_ANGLE.Y ;
    
}

//函数名：IMUupdate()
//输入：无
//输出: 无
//描述：四元素的融合，输出横滚和俯仰角 Q_ANGLE.Pitch  Q_ANGLE.Roll
//作者：马骏
//备注：没考上研，心情不好
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;
    float vx, vy, vz;// wx, wy, wz;
    float ex, ey, ez;
  
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q1q1 = q1*q1;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    norm = sqrt(ax*ax + ay*ay + az*az);      
    ax = ax /norm;
    ay = ay / norm;
    az = az / norm;
             
    vx = 2*(q1q3 - q0q2);												
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3 ;
  
    ex = (ay*vz - az*vy) ;                           					
    ey = (az*vx - ax*vz) ;
    ez = (ax*vy - ay*vx) ;
  
    exInt = exInt + ex * Ki;								 
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;
  
    gx = gx + Kp*ex + exInt;					   							
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;				   							
                                             
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
  
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
   
    //四元素到欧拉角结果输出，算出rool和pitch
    Q_ANGLE.Pitch  = radtoangle * asin(-2 * q1 * q3 + 2 * q0* q2);   // pitch
    Q_ANGLE.Roll = radtoangle * atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1);    // roll

  
}

