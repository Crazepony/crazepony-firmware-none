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
#include "control.h"
#include "stmflash.h"
#include "dmp.h"


//定义飞机最大倾斜角度
#define  Angle_Max  30.0f

uint8_t FLY_ENABLE=0;//飞行使能端  7/-5    12/15
//纠正姿态误差，可以用来抵抗重心偏移等带来的初始不平衡
#define  Rool_error_init   0       //如果飞机起飞朝左偏，Rool_error_init朝正向增大修改;朝右偏，Rool_error_init朝负向增大修改
#define  Pitch_error_init  0      //如果飞机起飞朝前偏，Pitch_error_init朝负向增大修改;朝后偏，Pitch_error_init朝正向增大修改

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
    RC_DATA.YAW = NRF24L01_RXDATA[4]-50;
    //RC_DATA.YAW = 0;                      //YAW角控制与否
    RC_DATA.YAW = (RC_DATA.YAW/50.0)*Angle_Max;
    RC_DATA.YAW=(RC_DATA.YAW > Angle_Max)  ? (Angle_Max):(RC_DATA.YAW);
    RC_DATA.YAW=(RC_DATA.YAW < -Angle_Max) ? (-Angle_Max):(RC_DATA.YAW);

    RC_DATA.THROTTLE=NRF24L01_RXDATA[0]+(NRF24L01_RXDATA[1]<<8);
    FLY_ENABLE = NRF24L01_RXDATA[31];   //0xA5或0，决定是否使能飞行，由遥控器决定

}


#define pkHeader1 0xAA
#define pkHeader2 0xBB 
//函数名：ReceiveDataFormUART()
//输入：无
//输出: 无
//描述：将收到的串口遥控数据赋值给对应的变量
//作者：马骏
//备注：没考上研，心情不好
void ReceiveDataFormUART(void)
{  
  Crazepony_get_uartpack();
}

#define parseState1 0x01
#define parseState2 0x02
#define parsethr1   0x03
#define parsethr2   0x04
#define parsepitch  0x05
#define parseroll   0x06
#define parseReservedByte   0x07
#define parseEnable   0x08
//解包
void parse_package(u8 pkdata)
{
  static u8 parseState = parseState1;
  static u8 payloadcnt = 0;
  switch(parseState)
  {
    case parseState1:
         payloadcnt ++;
         if(pkdata == pkHeader1) parseState = parseState2;
         else parseState = parseState1;
      break;
    case parseState2:
         payloadcnt ++;
         if(pkdata == pkHeader2) parseState = parsethr1;
         else parseState = parseState1;
      break;
    case parsethr1:
         payloadcnt ++;
         RC_DATA.THROTTLE = pkdata;
         parseState = parsethr2;
      break;
    case parsethr2:
         payloadcnt ++;
         RC_DATA.THROTTLE = RC_DATA.THROTTLE + (pkdata<<8);
         parseState = parsepitch;
      break;
    case parsepitch:
         payloadcnt ++;
         RC_DATA.PITCH=pkdata-50;//减50做负数传输
         RC_DATA.PITCH = (RC_DATA.PITCH/50.0)*Angle_Max+Pitch_error_init;
         RC_DATA.PITCH=(RC_DATA.PITCH > Angle_Max)  ? (Angle_Max):(RC_DATA.PITCH);
         RC_DATA.PITCH=(RC_DATA.PITCH < -Angle_Max) ? (-Angle_Max):(RC_DATA.PITCH);
         parseState = parseroll;
      break;  
    case parseroll:
         payloadcnt ++;
         RC_DATA.PITCH=pkdata-50;//减50做负数传输
         RC_DATA.PITCH = (RC_DATA.PITCH/50.0)*Angle_Max+Pitch_error_init;
         RC_DATA.PITCH=(RC_DATA.PITCH > Angle_Max)  ? (Angle_Max):(RC_DATA.PITCH);
         RC_DATA.PITCH=(RC_DATA.PITCH < -Angle_Max) ? (-Angle_Max):(RC_DATA.PITCH);
         parseState = parseReservedByte;
      break;  
    case parseReservedByte:
         payloadcnt ++;
         if((RX_PLOAD_WIDTH - 1) == payloadcnt) parseState = parseEnable;
      break;
    case parseEnable:
         payloadcnt ++;
         if(1 == pkdata) FLY_ENABLE=0xA5;
         else FLY_ENABLE=0;
         if((RX_PLOAD_WIDTH) == payloadcnt) 
           {
             payloadcnt = 0;
             parseState = parseState1;
           }
      break;
     default :break;
    
  } 
}

void Crazepony_get_uartpack(void)
{
  static int rx_cnt;
  u8 rx_da;
  rx_cnt = UartBuf_Cnt(&UartRxbuf);
  while(rx_cnt -- )
  {
    rx_da = UartBuf_RD(&UartRxbuf);
    parse_package(rx_da);
  }
}




/*
void Send_SaveAckToPC()
{
	int _temp;
	uint8_t sum=0;
	UART1_Put_Char(0x88);
	UART1_Put_Char(0xAE);
	UART1_Put_Char(0xcc);
//发送姿态数据：
	_temp = (int)(PID_Motor.P*10);
	sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
	_temp = (int)(PID_Motor.I*10);
	sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
  _temp = (int)(PID_Motor.D*10);
	sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
//发送电机数据：
	_temp =TIM2->CCR1/10;
	sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
	_temp =TIM2->CCR2/10;
	sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
	_temp =TIM2->CCR3/10;
	sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
	_temp =TIM2->CCR4/10;
	sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));

	Uart1_Put_Char(sum);
}*/

// void Send_PIDToPC()
// {
// 	int _temp;
// 	uint8_t sum=0;
// 	UART1_Put_Char(0x88);
// 	UART1_Put_Char(0xAD);
// 	UART1_Put_Char(0x1A);
// //发送姿态数据：
// 	_temp = (int)(PID_Motor.P*10);
// 	sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
// 	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
// 	_temp = (int)(PID_Motor.I*10);
// 	sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
// 	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
//   _temp = (int)(PID_Motor.D*10);
// 	sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
// 	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
// //发送电机数据：
// 	_temp =TIM2->CCR1/10;
// 	sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
// 	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
// 	_temp =TIM2->CCR2/10;
// 	sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
// 	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
// 	_temp =TIM2->CCR3/10;
// 	sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
// 	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
// 	_temp =TIM2->CCR4/10;
// 	sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
// 	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));

//	Uart1_Put_Char(sum);  
// }


void Send_AtitudeToPC()
{
	int _temp;
	uint8_t sum=0;
	UART1_Put_Char(0x88);
	UART1_Put_Char(0xAF);
	UART1_Put_Char(0x1c);
//发送姿态数据：
	_temp = (int)(Q_ANGLE.Pitch+200);
	sum += Uart1_Put_Char((unsigned char)(_temp>>8));
	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
	_temp = (int)(Q_ANGLE.Roll+200);
	sum += Uart1_Put_Char((unsigned char)(_temp>>8));
	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
  _temp = (int)(Q_ANGLE.Yaw+200);
	sum += Uart1_Put_Char((unsigned char)(_temp>>8));
	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
//发送电机数据：
	_temp =TIM2->CCR1/10;
	sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
	sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
  _temp =TIM2->CCR2/10;
  sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
  sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
  _temp = TIM2->CCR3/10;
  sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
  sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
  _temp =TIM2->CCR4/10;
  sum += Uart1_Put_Char((unsigned char)(_temp&0xff00)>>8);
  sum += Uart1_Put_Char((unsigned char)(_temp&0x00ff));
	Uart1_Put_Char(sum);
}
