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
ReceiveData.c file
编写者：小马  (Camel)
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.接收函数文件，包括接收2.4G数据，UART1的数据流
2.解析数据包，分配给对应的控制量
------------------------------------
*/

  
#include "ReceiveData.h"
#include "imu.h"
#include "moto.h"
#include "led.h"
#include "MPU6050.h"
#include "extern_variable.h"
#include "UART1.h"


//定义飞机最大倾斜角度
#define  Angle_Max  50.0

uint8_t FLY_ENABLE=0;//飞行使能端
//纠正姿态误差，可以用来抵抗重心偏移等带来的初始不平衡
int  Rool_error_init;      //如果飞机起飞朝左偏，Rool_error_init朝正向增大修改;朝右偏，Rool_error_init朝负向增大修改
int  Pitch_error_init;     //如果飞机起飞朝前偏，Pitch_error_init朝负向增大修改;朝吼偏，Pitch_error_init朝正向增大修改

RC_GETDATA   RC_DATA;	//经过处理的RC数据


//函数名：ReceiveDataFormNRF()
//输入：无
//输出: 无
//描述：将收到的2.4G遥控数据赋值给对应的变量
//作者：马骏
//备注：没考上研，心情不好
void ReceiveDataFormNRF(void)
{
    //PITCH
    RC_DATA.PITCH=NRF24L01_RXDATA[2]-50;//减50做负数传输
    RC_DATA.PITCH = (RC_DATA.PITCH/50.0)*Angle_Max+Pitch_error_init;
    RC_DATA.PITCH=(RC_DATA.PITCH > Angle_Max)  ? (Angle_Max):(RC_DATA.PITCH);
    RC_DATA.PITCH=(RC_DATA.PITCH < -Angle_Max) ? (-Angle_Max):(RC_DATA.PITCH);
    //ROOL
    RC_DATA.ROOL=NRF24L01_RXDATA[3]-50;//减50做负数传输
    RC_DATA.ROOL = (RC_DATA.ROOL/50.0)*Angle_Max+Rool_error_init; 
    RC_DATA.ROOL=(RC_DATA.ROOL > Angle_Max)  ? (Angle_Max):(RC_DATA.ROOL);
    RC_DATA.ROOL=(RC_DATA.ROOL < -Angle_Max) ? (-Angle_Max):(RC_DATA.ROOL);

    //YAW
    RC_DATA.YAW = 5+NRF24L01_RXDATA[4]-50;
    //RC_DATA.YAW = 0;                      //YAW角控制与否
    RC_DATA.YAW = (RC_DATA.YAW/50.0)*Angle_Max;
    RC_DATA.YAW=(RC_DATA.YAW > Angle_Max)  ? (Angle_Max):(RC_DATA.YAW);
    RC_DATA.YAW=(RC_DATA.YAW < -Angle_Max) ? (-Angle_Max):(RC_DATA.YAW);

    RC_DATA.THROTTLE=NRF24L01_RXDATA[0]+(NRF24L01_RXDATA[1]<<8);
    FLY_ENABLE = NRF24L01_RXDATA[31];   //0xA5或0，决定是否使能飞行，由遥控器决定
}


//函数名：ReceiveDataFormUART()
//输入：无
//输出: 无
//描述：将收到的串口遥控数据赋值给对应的变量
//作者：马骏
//备注：没考上研，心情不好
void ReceiveDataFormUART(void)
{

  
  if(rx_buffer[0]=='a'&&rx_buffer[1]=='s'&&rx_buffer[2]=='d')
  {

//         UART1_Put_Char(rx_buffer[0]);
//         UART1_Put_Char(rx_buffer[1]);
//         UART1_Put_Char(rx_buffer[2]);
  LedA_on;LedB_on;LedC_on;LedD_on;
  }
  else {LedA_off;LedB_off;LedC_off;LedD_off;}
  


}








