
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
  * main.c
  *
  */


#include "config.h"        //包含所有的驱动头文件

//#define  USART_REC_LEN      5        //接收缓冲元素最大个数


uint8_t SystemReady_OK=0;	      	//系统初始化完成标志
uint32_t While1_Lasttime=0;     //存储while循环的时间




/********************************************
              开机LED的各种状态
********************************************/
void PowerOn()
{
  char i;            //循环变量
    
  while(NRF24L01_RXDATA[30]!=0xA5)//保证收到一个完整的数据包32个字节,再继续下面的程序
    {
        Nrf_Irq();
        LedA_on;LedB_on;LedC_on;LedD_on;Delay(900000);LedA_off;LedB_off;LedC_off;LedD_off;Delay(900000*3);
    }
    while((NRF24L01_RXDATA[30]==0xA5)&&(!ParameterWrite())&&NRF24L01_RXDATA[28]==0xA5)//从上位机读取需要写入的参数，方便上位机调试时使用，不用每次都下程序改参数
    {   
        Nrf_Irq();
        LedA_on;LedB_on;LedC_on;LedD_on;Delay(900000*3);LedA_off;LedB_off;LedC_off;LedD_off;Delay(900000);
    }
    for(i=0;i<4;i++)//循环闪烁4次
    {
    LedA_on;LedB_off;LedC_off;LedD_off;
    Delay(900000);
    LedA_off;LedB_on;LedC_off;LedD_off;
    Delay(900000);
    LedA_off;LedB_off;LedC_on;LedD_off;
    Delay(900000);
    LedA_off;LedB_off;LedC_off;LedD_on;
    Delay(900000);
    }
    while(NRF24L01_RXDATA[27]!=0xA5)//保证收到一个完整的数据包32个字节,再继续下面的程序,等待解锁
    {
        Nrf_Irq();
        LedA_on;LedB_on;LedC_on;LedD_on;Delay(1);LedA_off;LedB_off;LedC_off;LedD_off;Delay(200);
    }
    for(i=0;i<3;i++)//解锁成功，快速闪烁3次提示
    {
    LedA_on;LedB_on;LedC_on;LedD_on;
    Delay(900000);
    LedA_off;LedB_off;LedC_off;LedD_off;
    Delay(900000);
    }
}

/********************************************
           使用内部DCO配置系统时钟
功能：
1.使用内部HSI时钟二分频（4MHz）作为PLL输入
2.PLL倍频系数PLLMUL<=13
3.输入参数：PLLMUL，PLL倍频系数
********************************************/
void SystemClock(char PLLMUL)
{
    RCC->CR|=1<<0;              //内部高速时钟使能
    while(!((RCC->CR)&(1<<1))); //等待内部时钟稳定就绪
    RCC->CFGR|=(PLLMUL-2)<<18;  //PPL倍频系数
    RCC->CFGR|=0<<16;           //PPL输入时钟源,HSI二分频后作为PLL输入源=4MHz
    RCC->CR|=1<<24;             //PLL使能
    while(!((RCC->CR)&(1<<25)));//等待PLL温度
    RCC->CFGR|=2<<0;           //系统时钟源配置，PLL输出作为系统时钟
}
/********************************************
              飞控主函数入口
功能：
1.初始化各个硬件
2.初始化系统参数
3.初始化PID参数
********************************************/
int main(void)
{

    //int temp;
    SystemClock(9);   //9倍频，系统时钟为36MHz
    NVIC_INIT();	     //中断初始化
    STMFLASH_Unlock(); //内部flash解锁
    LedInit();		     //IO初始化   
    BT_off;            //蓝牙关闭
    MotorInit();	     //马达初始化
    BatteryCheckInit();//电池电压监测初始化
    //mpu6050初始化顺序：
    //1.初始化IIC总线
    //2.调用该函数MPU6050_Check()检测设备存在与否
    //3.若设备存在，则初始化该设备，否则不初始化
    i2cInit();                //IIC初始化
    while(!MPU6050_Check())   //MPU6050检测  M1,M2
    {LedA_on;LedB_on;Delay(7000000);LedA_off;LedB_off;Delay(1000000);}
    MPU6050_INIT(); //检测通过才初始化，不能先初始化再检测 
    while(!NRF24L01_INIT()) //无线模块初始化 M3,M4
    {LedC_on;LedD_on;Delay(100000);LedC_off;LedD_off;Delay(7000000);}//无线模块初始化,初始化成功则不闪灯，不成功持续闪灯
    SetRX_Mode();   //设无线模块为接收模式
    SYSTICK_INIT(); //系统计时初始化，用来抗旋转，即yaw角
    SystemReady_OK = 1;//初始化完毕用于监测系统，初始化周期控制寄存器，开始执行系统控制
    PowerOn();      //开机LED灯的各种闪烁状态
    BT_on;           //自检完成，蓝牙开
    ParameterRead();//从flash读取各个参数
    PID_INIT();     //PID参数初始化 
    TIM4_Init(36,1000);	  //定时器4初始化，定时采样传感器数据，更新PID输出
    TIM3_Init(36,1000);   //定时器3初始化
    Uart1_init(36,115200); 	//串口1初始化  
    while (1);            //等待数据更新中断到来

}





