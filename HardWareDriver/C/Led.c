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
led.c file
编写者：小马  (Camel)
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.飞机四个臂上led IO口初始化
2.初始化默认关闭所有LED灯
------------------------------------
*/

#include "Led.h"
#include "UART1.h"
#include "config.h"
#include "imu.h"
#include "FailSafe.h"

LED_t LEDCtrl;
//接口显存
LEDBuf_t LEDBuf;

extern int LostRCFlag;


/********************************************
              Led初始化函数
功能：
1.配置Led接口IO输出方向
2.关闭所有Led(开机默认方式)
描述：
Led接口：
Led1-->PA11
Led2-->PA8
Led3-->PB1
Led4-->PB3
对应IO=1，灯亮
********************************************/
void LedInit(void)
{
    RCC->APB2ENR|=1<<2;    //使能PORTA时钟	
    RCC->APB2ENR|=1<<3;    //使能PORTB时钟	

    RCC->APB2ENR|=1<<0;      //使能复用时钟	   
    GPIOB->CRL&=0XFFFF0F0F;  //PB1,3推挽输出
    GPIOB->CRL|=0X00003030;
    GPIOB->ODR|=5<<1;        //PB1,3上拉
  
    GPIOA->CRH&=0XFFFF0FF0;  //PA8,11推挽输出
    GPIOA->CRH|=0X00003003;
    GPIOA->ODR|=9<<0;        //PA1,11上拉
  
    AFIO->MAPR|=2<<24;      //关闭JATG,千万不能将SWD也关闭，否则芯片作废，亲测!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    LedA_off;LedB_off;LedC_off;LedD_off;
}

//底层更新 ，10Hz
void LEDReflash(void)
{
 
		if(LEDBuf.bits.A)
			LedA_on;
		else
			LedA_off;
		
		if(LEDBuf.bits.B)
			LedB_on;
		else
			LedB_off;
		
		if(LEDBuf.bits.C)
			LedC_on;
		else
			LedC_off;
		
		if(LEDBuf.bits.D)
			LedD_on;
		else
			LedD_off;
		
// 		
// 		if(LEDBuf.bits.D)
// 			LedD_on;
// 		else
// 			LedD_off;
}

//事件驱动层
void LEDFSM(void)
{
	//闪烁状态由几个系统的标志决定,优先级依次按判断顺序上升
	LEDCtrl.event=E_READY;

	if(!imu.ready)		//开机imu准备
		LEDCtrl.event=E_CALI;
	
	if(1 == LostRCFlag)
			LEDCtrl.event=E_LOST_RC;	

	if(!imu.caliPass)
		LEDCtrl.event=E_CALI_FAIL;
	
	if(Battery.alarm)
		LEDCtrl.event=E_BAT_LOW;
	
	if(imuCaliFlag)
		LEDCtrl.event=E_CALI;
	
	if((Battery.chargeSta))			//battery charge check
		LEDCtrl.event = E_BatChg;
	
	if(LANDING == altCtrlMode){
		LEDCtrl.event = E_AUTO_LANDED;
	}
		
	switch(LEDCtrl.event)
	{
		case E_READY:
				if(++LEDCtrl.cnt >= 3)		//0 1 2 in loop, 0 on ,else off
					LEDCtrl.cnt=0;
				if(LEDCtrl.cnt==0)
						LEDBuf.byte =LA|LB;
				else
						LEDBuf.byte =0;
			break;
		case E_CALI:
				LEDBuf.byte=LA|LB;
			break;
		case E_BAT_LOW:
				if(++LEDCtrl.cnt >= 3)		//0 1  in loop
					LEDCtrl.cnt=0;
				if(LEDCtrl.cnt==0)
						LEDBuf.byte =0x0f;
				else
						LEDBuf.byte =0;
			break;
		case E_CALI_FAIL:
				if(++LEDCtrl.cnt >= 4)
					LEDCtrl.cnt=0;
				if(LEDCtrl.cnt<2)
						LEDBuf.byte =LC|LD;
				else
						LEDBuf.byte =LA|LB;
			break;
		case E_LOST_RC:
				if(++LEDCtrl.cnt >= 4)
					LEDCtrl.cnt=0;
				LEDBuf.byte= 1<<LEDCtrl.cnt ;
			break;
		case E_AUTO_LANDED:
				 LEDBuf.byte=0x0f;
			break;
		
		case E_BatChg:
				 LEDBuf.byte=0x00;
			break;
		
	}
	
	LEDReflash();
}


