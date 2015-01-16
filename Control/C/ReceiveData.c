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
#include "stm32f10x_it.h"
#include "SysConfig.h"
#include "CommApp.h"


uint8_t 		FLY_ENABLE=0;//飞行使能端  7/-5    14/15

RC_GETDATA  RC_DATA;//={0,0,0,0},RC_DATA_RAW={0,0,0,0};	// RC_DATA是处理后的期望四通

extern uint32_t lastGetRCTime;

//函数名：ReceiveDataFormNRF()
//输入：无
//输出: 无
//描述：将收到的2.4G遥控数据赋值给对应的变量
//作者：马骏
//备注：没考上研，心情不好
void ReceiveDataFormNRF(void)
{
	
	
 if((NRF24L01_RXDATA[0] == '$')&&(NRF24L01_RXDATA[1] == 'M')&&(NRF24L01_RXDATA[2] == '<'))
	 {
		 switch(NRF24L01_RXDATA[4])
		 {
			 case MSP_SET_4CON:
												 rcData[THROTTLE]=NRF24L01_RXDATA[5]+(NRF24L01_RXDATA[6]<<8);//UdataBuf[6]<<8 | UdataBuf[5];
												 rcData[YAW]=NRF24L01_RXDATA[7]   +  (NRF24L01_RXDATA[8]<<8);   //UdataBuf[8]<<8 | UdataBuf[7];
												 rcData[PITCH]=NRF24L01_RXDATA[9] + (NRF24L01_RXDATA[10]<<8);  //UdataBuf[10]<<8 | UdataBuf[9];
												 rcData[ROLL]=NRF24L01_RXDATA[11] + (NRF24L01_RXDATA[12]<<8); //UdataBuf[12]<<8 | UdataBuf[11];
			 break;
			  case MSP_ARM_IT://MSP_ARM_IT
						armState =REQ_ARM;
				break;
			 case MSP_DISARM_IT:
					armState =REQ_DISARM;
			 break;
			 case MSP_ACC_CALI:
					imuCaliFlag = 1;
			 break;
		 }
		 
	 }	
		lastGetRCTime=millis();		//ms
}


