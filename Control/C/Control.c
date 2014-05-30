
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

#include "control.h"
#include "moto.h"
#include "sys_fun.h"
#include "mpu6050.h"
#include "imu.h"
#include "extern_variable.h"
#include "led.h"
#include "stmflash.h"
#include "ReceiveData.h"


PID  PID_Motor;//定义一个PID结构体

//函数名：CONTROL()
//输入：无
//输出: 无
//描述：飞机控制函数主体，被定时器调用
//作者：马骏
//备注：没考上研，心情不好
void Controler(void)
{
    static uint8_t Control_Counter = 0;	//=2时控制一次,频率500HZ
    Control_Counter ++;
    IMU_CYCTIME = GET_NOWTIME(&While1_Lasttime);	//更新两次读取时间间隔
    MPU6050_Dataanl();//6050数据分析
    MPU6050_READ();	  //将6050数据减去稳态误差后放入相应变量,并启动下一次传输
    IMU_DataPrepare();

    if(Control_Counter==2)
    {
        Control_Counter = 0;   
        Nrf_Irq();   //接收控制目标参数
        PID_Calculate();   //=2时控制一次,频率500HZ	
    }
       
    
}

//函数名：PID_Calculate()
//输入：无
//输出: 无
//描述：飞机的自稳PID实现函数
//作者：马骏
//备注：没考上研，心情不好
void PID_Calculate(void)
{
    static float Thr=0,Rool=0,Pitch=0,Yaw=0;
    long Motor[4];   //定义电机PWM数组，分别对应M1-M4
    IMU_Output();	   //角度融合
    GET_EXPRAD();    //得到期望角度
 
    //ROOL
    Rool = PID_Motor.P * DIF_ANGLE.X;
    Rool -= PID_Motor.D * GYRO_F.X; 
    //Rool = Rool*10;
   //PITCH 
    Pitch = PID_Motor.P * DIF_ANGLE.Y;
    Pitch -= PID_Motor.D * GYRO_F.Y;
    //Pitch = Pitch*10;
   //基础动力
    Thr = 0.001*RC_DATA.THROTTLE*RC_DATA.THROTTLE;  //RC_DATA.THROTTLE为0到1000,
                                                     //将摇杆油门曲线转换为下凹的抛物线
    Thr -=30*DIF_ACC.Z;      //对Z轴用一次负反馈控制
   // YAW
   
    GYRO_I[0].Z += EXP_ANGLE.Z/250;
    Yaw = -40*GYRO_I[0].Z;
      

    //将输出值融合到四个电机 
    Motor[2] = (int16_t)(Thr - Pitch -Rool- Yaw );    //右  
    Motor[0] = (int16_t)(Thr + Pitch +Rool- Yaw );    //左
    Motor[3] = (int16_t)(Thr - Pitch +Rool+ Yaw );    //上 
    Motor[1] = (int16_t)(Thr + Pitch -Rool+ Yaw );    //下    

    if((FLY_ENABLE==0xA5))    {MotorPwmFlash(Motor[0],Motor[1],Motor[2],Motor[3]);}
    else                      {MotorPwmFlash(0,0,0,0);GYRO_I[0].Z=0;GYRO_I[1].Z=0;GYRO_I[2].Z=0;}//避免飞机落地重启时突然打转 


  
}


int PIDParameterAdd=0;//PID参数写入首地址为0,占3个字节
int ErrorParameterAdd=10;//初始俯仰横滚误差写入地址为10,占2个字节
u16 PIDWriteBuf[3];       //写入flash的临时数组  PID参数
u16 PRWriteBuf[2];        //写入flash的临时数组  俯仰误差

//函数名：ParameterWrite()
//输入：无
//输出：当收到地址29的字节为0xA5时，返回1，否则返回0
//描述：飞机开机后，当检测到写入参数模式时，写参数用
//作者：马骏
//备注：没考上研，心情不好
char  ParameterWrite()
{

  //PID参数写入飞控flash.
if(NRF24L01_RXDATA[29]==0xA5)
{
        PIDWriteBuf[0]=NRF24L01_RXDATA[0];
        PIDWriteBuf[1]=NRF24L01_RXDATA[1];
        PIDWriteBuf[2]=NRF24L01_RXDATA[2];//写PID参数
        PRWriteBuf[0] =NRF24L01_RXDATA[3];
        PRWriteBuf[1] =NRF24L01_RXDATA[4];//写俯仰误差，为了便于处理负数，整体偏移了100
  
        STMFLASH_Write(STM32_FLASH_BASE+STM32_FLASH_OFFEST+PIDParameterAdd,PIDWriteBuf,3); //PID 参数写入
        STMFLASH_Write(STM32_FLASH_BASE+STM32_FLASH_OFFEST+ErrorParameterAdd,PRWriteBuf,2); //俯仰误差 参数写入，在缓冲数组的第三个字节开始
     
return 1;
}

return 0;
}



/*********************************
条件编译PID和初始零漂参数的来源
*********************************/


//#define ParameterReadFromFlash


//PID初始化参数读取数组缓存，全局数组
u16 PIDreadBuf[3]; 
//函数名：PID_INIT()
//输入：无
//输出: 无
//描述：PID参数初始化
//作者：马骏
//备注：没考上研，心情不好
void PID_INIT(void)
{
    
    //PID_RP.P--->PIDreadBuf[0]//
    //PID_RP.I--->PIDreadBuf[1]//
    //PID_RP.D--->PIDreadBuf[2]// 初始化时从内部flash读取设定值，方便调试  不用下程序就能调节PID参数
#ifdef ParameterReadFromFlash
  
    PID_Motor.P = PIDreadBuf[0]/10.0;//比例增益
    PID_Motor.I = PIDreadBuf[1]/10.0;//积分增益
    PID_Motor.D = PIDreadBuf[2]/10.0;//微分增益  //通过上位机写入到飞机的片内flash，开机初始化时读取。目前只能我这边能用此功能
  
#else
    PID_Motor.P = 0.4;//比例增益
    PID_Motor.I = 0;//积分增益
    PID_Motor.D = 1.5;//微分增益  改组参数是测试过程中最好优秀的一组
#endif
  
    PID_Motor.POUT = 0;
    PID_Motor.IOUT = 0;
    PID_Motor.DOUT = 0;
  
    PID_Motor.IMAX = 300;
    PID_Motor.LastError = 0;
    PID_Motor.PrerError = 0;
}

//函数名：ParameterRead()
//输入：无
//输出：无
//描述：初始化时，读取上位机最后一次设定的参数
//作者：马骏
//备注：没考上研，心情不好
void  ParameterRead()
{
  u16 PitchRoolBuf[2];
  STMFLASH_Read(STM32_FLASH_BASE+STM32_FLASH_OFFEST+PIDParameterAdd,PIDreadBuf,3);
  STMFLASH_Read(STM32_FLASH_BASE+STM32_FLASH_OFFEST+ErrorParameterAdd,PitchRoolBuf,2);

#ifdef ParameterReadFromFlash
  
  Pitch_error_init= PitchRoolBuf[0];
  Rool_error_init = PitchRoolBuf[1];//通过上位机写入到飞机的片内flash，开机初始化时读取。目前只能我这边能用此功能
  
#else
  
    Pitch_error_init= 0;  //如果飞机起飞朝前偏，Pitch_error_init朝负向增大修改;朝吼偏，Pitch_error_init朝正向增大修改
    Rool_error_init = 1;//如果飞机起飞朝左偏，Rool_error_init朝正向增大修改;朝右偏，Rool_error_init朝负向增大修改,单位 ：度

#endif

}



