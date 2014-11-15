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
#include "BT.h"

//----PID结构体实例化----
PID_Typedef pitch_angle_PID;	//角度环的PID
PID_Typedef pitch_rate_PID;		//角速率环的PID

PID_Typedef roll_angle_PID;
PID_Typedef roll_rate_PID;

PID_Typedef yaw_angle_PID;
PID_Typedef yaw_rate_PID;

float gyroxGloble = 0;
float gyroyGloble = 0;


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
    char i;
    Counter_Cnt++;
    DMP_Routing();	        //DMP 线程  所有的数据都在这里更新
    DMP_getYawPitchRoll();  //读取 姿态角
    /*******************向上位机发送姿态信息，如果要在PC上位机看实时姿态,宏开关控制***************/
    #ifndef Debug
    Send_AtitudeToPC();     
    #else
    #endif 
  
    if(Counter_Cnt==5)
    {
    Counter_Cnt=0;
      
    for(i=0;i<RX_PLOAD_WIDTH;i++)
    NRF24L01_RXDATA[i] = 0;    
      
    //Nrf_Irq();           //从2.4G接收控制目标参数

    ReceiveDataFormUART();//从蓝牙透传模块接收控制目标参数，和2.4G接收控制只能选其一
    PID_Calculate();     //=2时控制一次,频率200HZ	
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
float YawLock = 0;
uint8_t YawLockState  = 0;
int YawLockRange = 10; //锁定航向，偏航杆允许在中位波动的范围
void PID_Calculate(void)
{  
    static float Thr=0,Rool=0,Pitch=0,Yaw=0;
    static int PIDcounter=0;
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

    DIF_ACC.Z =  DMP_DATA.dmp_accz - g;    
  
    //偏航杆位判定
    if(EXP_ANGLE.Z <YawLockRange && EXP_ANGLE.Z>-YawLockRange)
    {
        if(YawLockState == 0x00)
        {
          YawLock = Q_ANGLE.Yaw;
          YawLockState = 0xff;
        }
    }
    else YawLockState = 0x00;
    
    
    
    PIDcounter++;
    if(PIDcounter == 2)
    {
            PIDcounter = 0;
          /*********************************************************
           PID核心算法部分
          *********************************************************/
          //------------俯仰控制------------
          //参数整定原则为先内后外，故在整定内环时将外环的PID均设为0
          //外环控 制。输入为角度,输出为角速度。PID->Output作为内环的输入。
          PID_Postion_Cal(&pitch_angle_PID,EXP_ANGLE.Y,Q_ANGLE.Pitch,0);   
          //外环控 制。输入为角度,输出为角速度。PID->Output作为内环的输入。
          PID_Postion_Cal(&roll_angle_PID,EXP_ANGLE.X,Q_ANGLE.Roll,0);
    }
    
    
    if(YawLockState == 0x00) YawLock = Q_ANGLE.Yaw; //偏航锁定状态判定  
    else  EXP_ANGLE.Z = 0;
    //外环控 制。输入为角度,输出为角速度。PID->Output作为内环的输入。
    PID_Postion_Cal(&yaw_angle_PID,YawLock,Q_ANGLE.Yaw,0);
    
    
    
    //内环控制，输入为角速度，输出为PWM增量
    PID_Postion_Cal(&pitch_rate_PID,pitch_angle_PID.Output,gyroyGloble,0);
    //参数整定原则为先内后外，故在整定内环时将外环的PID均设为0
    //内环控制，输入为角速度，输出为PWM增量
    PID_Postion_Cal(&roll_rate_PID,roll_angle_PID.Output,gyroxGloble,0);
    //参数整定原则为先内后外，故在整定内环时将外环的PID均设为0
    //内环控制，输入为角速度，输出为PWM增量
    PID_Postion_Cal(&yaw_rate_PID,-yaw_angle_PID.Output - EXP_ANGLE.Z,DMP_DATA.GYROz,0);
    //参数整定原则为先内后外，故在整定内环时将外环的PID均设为0
    
    
    
    //基础油门动力
    //Thr = 0.001*RC_DATA.THROTTLE*RC_DATA.THROTTLE;   //RC_DATA.THROTTLE为0到1000,将摇杆油门曲线转换为下凹的抛物线
    Thr = RC_DATA.THROTTLE;
    Thr -=  80*DIF_ACC.Z;                             
    
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


#define PIDParameterAdd   0    //PID参数写入首地址为
#define BTParameterAdd    32   //蓝牙参数写入Flash地址为

Parameter_Typedef PIDParameter;//实例化一个PID的Flash参数
Parameter_Typedef BTParameter; //实例化一个蓝牙Flash参数

//函数名：ParameterWrite()
//输入：无
//输出：当收到地址29的字节为0xA5时，返回1，否则返回0
//描述：飞机开机后，当检测到写入参数模式时，写参数用
//作者：马骏
//备注：没考上研，心情不好
char  ParameterWrite()
{
//         PIDParameter.WriteBuf[0] = 23;
//         PIDParameter.WriteBuf[1] = 1;
//         PIDParameter.WriteBuf[2] = 4;
//   
//         BTParameter.WriteBuf[1]  = 0;
//         BTParameter.WriteBuf[2]  = 0;
  
        //STMFLASH_Write(STM32_FLASH_BASE+STM32_FLASH_OFFEST+PIDParameterAdd,PIDParameter.WriteBuf,3); //PID 参数写入Flash
        STMFLASH_Write(STM32_FLASH_BASE+STM32_FLASH_OFFEST+BTParameterAdd,BTParameter.WriteBuf,3);  //蓝牙配置参数写入Flash
       
return 0;
}

//函数名：ParameterRead()
//输入：无
//输出：无
//描述：初始化时，读取上位机最后一次设定的参数
//作者：马骏
//备注：没考上研，心情不好
void  ParameterRead()
{      
  //STMFLASH_Read(STM32_FLASH_BASE+STM32_FLASH_OFFEST+PIDParameterAdd,PIDParameter.ReadBuf,3);
  STMFLASH_Read(STM32_FLASH_BASE+STM32_FLASH_OFFEST+BTParameterAdd,BTParameter.ReadBuf,3);
  
  printf("从FLASH中读取参数...\r\n");

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
     pitch_angle_PID.P = 3.5;
     pitch_angle_PID.I = 0;
     pitch_angle_PID.D = 0;

     pitch_rate_PID.P  = 0.5; 
     pitch_rate_PID.I  = 0; 
     pitch_rate_PID.D  = 1.5; 
////////////////////////////////////////////
     roll_angle_PID.P = 3.5;
     roll_angle_PID.I = 0;
     roll_angle_PID.D = 0;

     roll_rate_PID.P  = 0.5;
     roll_rate_PID.I  = 0; 
     roll_rate_PID.D  = 1.5; 
///////////////////////////////////////////
     yaw_angle_PID.P = 1;
     yaw_angle_PID.I = 0;
     yaw_angle_PID.D = 0;
  
     yaw_rate_PID.P  = 15;
     yaw_rate_PID.I  = 0; 
     yaw_rate_PID.D  = 0; 

     printf("PID初始化完成...\r\n");
  
}





