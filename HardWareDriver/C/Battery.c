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
Battery.c file
编写者：小马  (Camel)
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.电池检测AD初始化
2.供低压检测用，提供片内温度传感器驱动
------------------------------------
*/

#include "Battery.h"
#include "UART1.h"
#include "stdio.h"
#include "CommApp.h"
#include "ReceiveData.h"

//实例化一个电压信息结构体
Bat_Typedef Battery;


//初始化电池检测ADC
//开启ADC1的通道8	
//BatteryCheck---->PB0
void BatteryCheckInit()
{
  
 //先初PB0为模拟输入
  RCC->APB2ENR|=1<<3;    //使能PORTB口时钟 
  GPIOB->CRL&=0XFFFFFFF0;//PB0	anolog输入
	//通道8	 
	RCC->APB2ENR|=1<<9;    //ADC1时钟使能	  
	RCC->APB2RSTR|=1<<9;   //ADC1复位
	RCC->APB2RSTR&=~(1<<9);//复位结束	    
	RCC->CFGR&=~(3<<14);   //分频因子清零	
	//SYSCLK/DIV2=12M ADC时钟设置为12M,ADC最大时钟不能超过14M!
	//否则将导致ADC准确度下降! 
	RCC->CFGR|=2<<14;      	 
	ADC1->CR1&=0XF0FFFF;   //工作模式清零
	ADC1->CR1|=0<<16;      //独立工作模式  
	ADC1->CR1&=~(1<<8);    //非扫描模式	  
	ADC1->CR2&=~(1<<1);    //单次转换模式
	ADC1->CR2&=~(7<<17);	   
	ADC1->CR2|=7<<17;	     //软件控制转换  
	ADC1->CR2|=1<<20;      //使用用外部触发(SWSTART)!!!	必须使用一个事件来触发
	ADC1->CR2&=~(1<<11);   //右对齐	 
	ADC1->CR2|=1<<23;      //使能温度传感器

	ADC1->SQR1&=~(0XF<<20);
	ADC1->SQR1&=0<<20;     //1个转换在规则序列中 也就是只转换规则序列1 			   
	//设置通道1的采样时间
	ADC1->SMPR2&=~(7<<3);  //通道1采样时间清空	  
 	ADC1->SMPR2|=7<<3;     //通道1  239.5周期,提高采样时间可以提高精确度	 

 	ADC1->SMPR1&=~(7<<18);  //清除通道16原来的设置	 
	ADC1->SMPR1|=7<<18;     //通道16  239.5周期,提高采样时间可以提高精确度	 

	ADC1->CR2|=1<<0;	   //开启AD转换器	 
	ADC1->CR2|=1<<3;       //使能复位校准  
	while(ADC1->CR2&1<<3); //等待校准结束 			 
  //该位由软件设置并由硬件清除。在校准寄存器被初始化后该位将被清除。 		 
	ADC1->CR2|=1<<2;        //开启AD校准	   
	while(ADC1->CR2&1<<2);  //等待校准结束
	//该位由软件设置以开始校准，并在校准结束时由硬件清除  
  
  
  
  
  
  Battery.BatReal = 3.95;//单位为v 电池实际电压  校准电压时修改
  Battery.ADinput = 1.98;//单位为v R15和R17连接处电压 校准电压时修改
  Battery.ADRef   = 3.26;//单位为v 单片机供电电压   校准电压时修改
  Battery.Bat_K   = Battery.BatReal/Battery.ADinput;//计算电压计算系数
  Battery.BatteryADmin = 2000;//电压门限AD值
  
  
  
  
  printf("电压监测AD初始完成...\r\n");
  
}





//获得ADC值
//ch:通道值 0~16
//返回值:转换结果
u16 Get_Adc(u8 ch)   
{
	//设置转换序列	  		 
	ADC1->SQR3&=0XFFFFFFE0;//规则序列1 通道ch
	ADC1->SQR3|=ch;		  			    
	ADC1->CR2|=1<<22;       //启动规则转换通道 
	while(!(ADC1->SR&1<<1));//等待转换结束	 	   
	return ADC1->DR;		    //返回adc值	
}

//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
	}
	return temp_val/times;
} 

//得到ADC采样内部温度传感器的温度值
//返回值3位温度值 XXX*0.1C	 
int Get_Temp(void)
{				 
	u16 temp_val=0;
	u8 t;
	float temperate;   
	for(t=0;t<20;t++)//读20次,取平均值
	{
		temp_val+=Get_Adc(16);//温度传感器为通道16
	}
	temp_val/=20;
	temperate=(float)temp_val*(3.3/4096);//得到温度传感器的电压值
	temperate=(1.43-temperate)/0.0043+25;//计算出当前温度值	 
	temperate*=10;//扩大十倍,使用小数点后一位
	return (int)temperate;	 
}


//返回电池电压AD值
int GetBatteryAD()
{
 return Get_Adc_Average(8,5);
}

#include "BT.h"

//检测电池电压
void BatteryCheck(void)
{
		Battery.BatteryAD  = GetBatteryAD();            //电池电压检测  
		Battery.BatteryVal = Battery.Bat_K * (Battery.BatteryAD/4096.0) * Battery.ADRef;//实际电压 值计算	
	  if(FLY_ENABLE)
		{
			if(Battery.BatteryAD <= Battery.BatteryADmin)
			{
					Battery.alarm=1;
			}
			else
					Battery.alarm=0;
		}
		else
		{
			if((Battery.BatteryVal < BAT_ALARM_VAL)&&(Battery.BatteryVal > BAT_CHG_VAL))	//低于3.7v 且大于充电检测电压 BAT_CHG_VAL
				Battery.alarm=1;
			else
				Battery.alarm=0;
		}
		
		if(Battery.BatteryVal < BAT_CHG_VAL) //on charge
		{
			Battery.chargeSta = 1; 
			BT_off();
		}
		else 					
			Battery.chargeSta = 0;

}



