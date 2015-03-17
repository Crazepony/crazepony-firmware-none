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
 /* main.c file
编写者：小马  (Camel)
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
提供精确的延时API  有微秒级 和毫秒级延时
------------------------------------
*/
 
 
#include "delay.h"
#include "UART1.h"
#include "config.h"
#include "stm32f10x_it.h"

static u8  fac_us=0;//us延时倍乘数
static u16 fac_ms=0;//ms延时倍乘数

//初始化延迟函数
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
/***********************************************
函数名：delay_init(u8 SYSCLK)
功能：初始化延时函数
输入参数：SYSCLK
输出:无
描述：由于该延时是由定时器中断计数完成的，所以需要对相应的寄存器和时钟赋值
备注：输出参数为系统时钟
***********************************************/
void delay_init(u8 SYSCLK)
{
	SysTick->CTRL&=0xfffffffb;//bit2清空,选择外部时钟  HCLK/8
	fac_us=SYSCLK/8;		    
	fac_ms=(u16)fac_us*1000;
  printf("延时函数初始化完成...\r\n");
}				
				    
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
/**************************实现函数********************************************
*函数原型:		void delay_ms(u16 nms)
*功　　能:		毫秒级延时  延时nms  nms<=1864 
*******************************************************************************/
/*void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL=0x01 ;          //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	  
}   */
void delay_ms(uint16_t nms)
{
		uint32_t t0=micros();
		while(micros() - t0 < nms * 1000);
			
}

//延时nus
//nus为要延时的us数.
/**************************实现函数********************************************
*函数原型:		void delay_us(u32 nus)
*功　　能:		微秒级延时  延时nus  nms<=1864 
*******************************************************************************/		    								   
/*
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	 
}*/

void delay_us(u32 nus)
{
		uint32_t t0=micros();
		while(micros() - t0 < nus);
			
}

//粗略延时
void Delay(unsigned long delay_time)
{
   long i;
   
   for(i=0; i<delay_time; i++);
 
}


//------------------End of File----------------------------
