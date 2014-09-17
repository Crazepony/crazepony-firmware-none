/*    
      ____                      _____                  +---+
     / ___\                     / __ \                 | R |
    / /                        / /_/ /                 +---+
   / /   ________  ____  ___  / ____/___  ____  __   __
  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
 / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
 \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
                                                 / /
                                            ____/ /
                                           /_____/
*/
/* Control.c file
编写者：小马  (Camel)
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.PID参数初始化
2.控制函数

------------------------------------
*/
#include "control.h"
#include "moto.h"
#include "math.h"
#include "sys_fun.h"
#include "mpu6050.h"
#include "imu.h"
#include "extern_variable.h"
#include "led.h"
#include "stmflash.h"
#include "ReceiveData.h"
#include "DMP.h"
#include "Battery.h"
#include "stdio.h"


//----PID结构体实例化----
PID_Typedef pitch_angle_PID;	//角度环的PID
PID_Typedef pitch_rate_PID;		//角速率环的PID

PID_Typedef roll_angle_PID;
PID_Typedef roll_rate_PID;

PID_Typedef yaw_angle_PID;
PID_Typedef yaw_rate_PID;



S_FLOAT_XYZ DIF_ACC;		//实际去期望相差的加速度
S_FLOAT_XYZ EXP_ANGLE;	//期望角度	
S_FLOAT_XYZ DIF_ANGLE;	//实际与期望相差的角度	



//函数名：Controler()
//输入：无
//输出: 无
//描述：飞机控制函数主体，被定时器调用
//作者：马骏
//备注：没考上研，心情不好
void Controler(void)
{     
    static char Counter_Cnt=0;
    Counter_Cnt++;
    DMP_Routing();	        //DMP 线程  所有的数据都在这里更新
    DMP_getYawPitchRoll();  //读取 姿态角
  
    /*******************向上位机发送姿态信息，如果要在PC上位机看实时姿态，需要把这里解注释****************/
    /*******************PC姿态显示，跟串口debug不能共存****************/
    Send_AtitudeToPC();     
    
    if(Counter_Cnt==5)
    {
    Counter_Cnt=0;
    Nrf_Irq();           //从2.4G接收控制目标参数
    //ReceiveDataFormUART();//从蓝牙透传模块接收控制目标参数，和2.4G接收控制只能选其一
    PID_Calculate();     //=2时控制一次,频率500HZ	
    }
}







//-----------位置式PID-----------
void PID_Postion_Cal(PID_Typedef * PID,float target,float measure,int32_t dertT)
{
	//-----------位置式PID-----------
	//误差=期望值-测量值
	PID->Error=target-measure;
	
	PID->Integ+=(double)PID->Error*dertT/1000000.0;

	PID->Deriv=PID->Error-PID->PreError;
	
	PID->Output=PID->P*PID->Error+PID->I*PID->Integ+PID->D*PID->Deriv;
	
	PID->PreError=PID->Error;
	
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
//     static float  g_init = DMP_DATA.dmp_accz;        //当地重力加速度变量

    long Motor[4];   //定义电机PWM数组，分别对应M1-M4
  /*********************************************************
     计算期望姿态与实际姿态的差值
    *********************************************************/
    EXP_ANGLE.X = (float)(RC_DATA.ROOL);
    EXP_ANGLE.Y = (float)(RC_DATA.PITCH);
    EXP_ANGLE.Z = (float)(RC_DATA.YAW);

    DIF_ANGLE.X = EXP_ANGLE.X - Q_ANGLE.Roll;
    DIF_ANGLE.X = DIF_ANGLE.X;
    
    DIF_ANGLE.Y = EXP_ANGLE.Y - Q_ANGLE.Pitch;
    DIF_ANGLE.Y = DIF_ANGLE.Y;

    DIF_ACC.Z =  DMP_DATA.dmp_accz - g;     //Z 轴加速度实际与静止时的差值，g为当地重力加速度,初始化时采样
  
  
  
    /*********************************************************
     PID核心算法部分
    *********************************************************/
  //------------俯仰控制------------
    //参数整定原则为先内后外，故在整定内环时将外环的PID均设为0
    //外环控 制。输入为角度,输出为角速度。PID->Output作为内环的输入。
    PID_Postion_Cal(&pitch_angle_PID,EXP_ANGLE.Y,Q_ANGLE.Pitch,0);
    
    //内环控制，输入为角速度，输出为PWM增量
    PID_Postion_Cal(&pitch_rate_PID,pitch_angle_PID.Output,DMP_DATA.GYROy,0);
    //参数整定原则为先内后外，故在整定内环时将外环的PID均设为0
    
    
    //外环控 制。输入为角度,输出为角速度。PID->Output作为内环的输入。
    PID_Postion_Cal(&roll_angle_PID,EXP_ANGLE.X,Q_ANGLE.Roll,0);
    
    //内环控制，输入为角速度，输出为PWM增量
    PID_Postion_Cal(&roll_rate_PID,roll_angle_PID.Output,DMP_DATA.GYROx,0);
    //参数整定原则为先内后外，故在整定内环时将外环的PID均设为0
    

    //外环控 制。输入为角度,输出为角速度。PID->Output作为内环的输入。
    PID_Postion_Cal(&yaw_angle_PID,EXP_ANGLE.Z,Q_ANGLE.Yaw,0);
    
    //内环控制，输入为角速度，输出为PWM增量
    PID_Postion_Cal(&yaw_rate_PID,-2*EXP_ANGLE.Z,DMP_DATA.GYROz,0);
    //参数整定原则为先内后外，故在整定内环时将外环的PID均设为0
    
    
    //基础油门动力
    //Thr = 0.001*RC_DATA.THROTTLE*RC_DATA.THROTTLE;   //RC_DATA.THROTTLE为0到1000,将摇杆油门曲线转换为下凹的抛物线
    Thr = RC_DATA.THROTTLE;
    Thr -=100*DIF_ACC.Z;                             //对Z轴用一次负反馈控制
   
   
    Pitch = pitch_rate_PID.Output;
    Rool  = roll_rate_PID.Output;
    Yaw   = yaw_rate_PID.Output; 
    
     
   //将输出值融合到四个电机 
    Motor[2] = (int16_t)(Thr - Pitch -Rool- Yaw );    //M3  
    Motor[0] = (int16_t)(Thr + Pitch +Rool- Yaw );    //M1
    Motor[3] = (int16_t)(Thr - Pitch +Rool+ Yaw );    //M4 
    Motor[1] = (int16_t)(Thr + Pitch -Rool+ Yaw );    //M2    
    
    if((FLY_ENABLE==0xA5))MotorPwmFlash(Motor[0],Motor[1],Motor[2],Motor[3]);   
    else                  MotorPwmFlash(0,0,0,0);//避免飞机落地重启时突然打转 
    if(NRF24L01_RXDATA[10]==0xA5) MotorPwmFlash(5,5,Motor[2],Motor[3]); //一键操作，翻滚返航等，测试功能，不要用
    
    
    
    
     
}


int PIDParameterAdd=0;//PID参数写入首地址为0,占3个字节


u16 PIDWriteBuf[3];       //写入flash的临时数组  PID参数
u16 PIDreadBuf[3];        //

//函数名：ParameterWrite()
//输入：无
//输出：当收到地址29的字节为0xA5时，返回1，否则返回0
//描述：飞机开机后，当检测到写入参数模式时，写参数用
//作者：马骏
//备注：没考上研，心情不好
char  ParameterWrite()
{

        PIDWriteBuf[0]=16;
        PIDWriteBuf[1]=0;
        PIDWriteBuf[2]=8;//写PID参数  
        STMFLASH_Write(STM32_FLASH_BASE+STM32_FLASH_OFFEST+PIDParameterAdd,PIDWriteBuf,3); //PID 参数写入
  
return 0;
}



/*********************************
条件编译PID和初始零漂参数的来源
*********************************/

//#define ParameterReadFromFlash

//函数名：PID_INIT()
//输入：无
//输出: 无
//描述：PID参数初始化
//作者：马骏
//备注：没考上研，心情不好
void PID_INIT(void) 
{
     
     pitch_angle_PID.P = 8;
     pitch_angle_PID.I = 0;
     pitch_angle_PID.D = 0;

     pitch_rate_PID.P  = 0.2; 
     pitch_rate_PID.I  = 0; 
     pitch_rate_PID.D  = 1; 
////////////////////////////////////////////
     roll_angle_PID.P = 8;
     roll_angle_PID.I = 0;
     roll_angle_PID.D = 0;

     roll_rate_PID.P  = 0.2;
     roll_rate_PID.I  = 0; 
     roll_rate_PID.D  = 1; 
///////////////////////////////////////////
     yaw_angle_PID.P = 1;
     yaw_angle_PID.I = 0;
     yaw_angle_PID.D = 0;
  
     yaw_rate_PID.P  = 20;
     yaw_rate_PID.I  = 0; 
     yaw_rate_PID.D  = 0; 

     printf("PID初始化完成...\r\n");
}

//函数名：ParameterRead()
//输入：无
//输出：无
//描述：初始化时，读取上位机最后一次设定的参数
//作者：马骏
//备注：没考上研，心情不好
void  ParameterRead()
{                                                                             
  STMFLASH_Read(STM32_FLASH_BASE+STM32_FLASH_OFFEST+PIDParameterAdd,PIDreadBuf,3);      
  printf("从FLASH中读取参数...\r\n");

}



