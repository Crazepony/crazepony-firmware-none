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
main.c file
编写者：小马  (Camel)
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.飞机硬件初始化
2.参数初始化
3.定时器开
4.等待中断到来
------------------------------------
*/

#include "config.h"        //包含所有的驱动头文件

/********************************************
              飞控主函数入口
功能：                                        
1.初始化各个硬件
2.初始化系统参数
3.开定时器4等待数据中断到来
4.开定时器3串口广播实时姿态以及相关信息
********************************************/
int main(void)
{
  int i;
  SystemClock_HSE(9);           //系统时钟初始化，时钟源外部晶振HSE
  //SystemClock_HSI(9);         //系统时钟初始化，时钟源内部HSI
  UART1_init(SysClock,115200); 	//串口1初始化
  NVIC_INIT();	                //中断初始化
  STMFLASH_Unlock();            //内部flash解锁
  LedInit();		                //IO初始化 
  delay_init(SysClock);         //滴答延时初始化 
  BT_PowerInit();               //蓝牙电源初始化完成，默认关闭 
  MotorInit();	                //马达初始化
  BatteryCheckInit();           //电池电压监测初始化
  IIC_Init();                   //IIC初始化
  MPU6050_DMP_Initialize();     //初始化DMP引擎
  PID_INIT();                   //PID参数初始化  
  //HMC5883L_SetUp();           //初始化磁力计HMC5883L
  ParameterRead();              //Flash参数读取
  PID_INIT();                   //PID参数初始化 
  NRF24L01_INIT();              //NRF24L01初始化
  SetRX_Mode();                 //设无线模块为接收模式
  initPressure();				//初始化气压计
  /////////////////////////
//   NRF24L01_RXDATA[30]=0xA5;
//   NRF24L01_RXDATA[27]=0xA5;//跳过解锁,调试用，跳过下面的poweron
  /////////////////////////
  PowerOn();                    //开机等待  
  //BT_ATcmdWrite();            //蓝牙写配置
  //BT_off();                   //蓝牙关闭
  //ParameterWrite();           //写参数到内部模拟eeprom
  TIM3_Init(SysClock,1000);	    //定时器3初始化，调试串口输出
  TIM4_Init(SysClock,1000);	    //定时器4初始化，定时采样传感器数据，更新PID输出，定时器定时基石为1us，PID更新周期为4ms，所以姿态更新频率 为250Hz    
                                
  while (1)                    //等待数据更新中断到来
  {    

    
//        
// //测试环形缓冲数组用，可以无视或者直接注释掉。不注释也不影响操作
//     printf("\r\n收到数据[%d] = %d\r\n",i,rx_buffer[i++]);
//     printf("读指针 = %d\r\n",UartRxbuf.Rd_Indx & UartRxbuf.Mask);
//     printf("写指针 = %d\r\n",UartRxbuf.Wd_Indx & UartRxbuf.Mask);
//     printf("可用%d字节\r\n",UartBuf_Cnt(&UartRxbuf));
//     if(i>UartRxbuf.Mask){
//     i=0;
//     //UartBuf_RD(&UartRxbuf);
//     printf("*********************\r\n");
//     }
/////////////////////////////////////////////////////////////////
  }
}

