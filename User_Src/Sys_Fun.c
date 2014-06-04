#include "config.h"
#include "extern_variable.h"



char SysClock;       //申请存储系统时钟变量，单位MHz

/********************************************
           系统中断优先级配置
功能：
1.各个中断优先级配置函数统一封装为：中断优先级配置初始化
********************************************/
void NVIC_INIT(void)
{
    TimerNVIC_Configuration();//定时器中断配置
    UART1NVIC_Configuration();//串口1中断配置
}

/********************************************
           使用内部DCO配置系统时钟
功能：
1.使用内部HSI时钟二分频（4MHz）作为PLL输入
2.PLL倍频系数PLLMUL<=13
3.输入参数：PLLMUL，PLL倍频系数
********************************************/
char SystemClock(char PLLMUL)
{
    RCC->CR|=1<<0;              //内部高速时钟使能
    while(!((RCC->CR)&(1<<1))); //等待内部时钟稳定就绪
    RCC->CFGR|=(PLLMUL-2)<<18;  //PPL倍频系数
    RCC->CFGR|=0<<16;           //PPL输入时钟源,HSI二分频后作为PLL输入源=4MHz
    RCC->CR|=1<<24;             //PLL使能
    while(!((RCC->CR)&(1<<25)));//等待PLL温度
    RCC->CFGR|=2<<0;           //系统时钟源配置，PLL输出作为系统时钟
    SysClock=4*PLLMUL;         //返回系统时钟，单位MHz
    return SysClock;
}
/********************************************
              开机LED的各种状态
********************************************/
void PowerOn()
{
  char i;            //循环变量
    
  while(NRF24L01_RXDATA[30]!=0xA5)//保证收到一个完整的数据包32个字节,再继续下面的程序
    {
        Nrf_Irq();DEBUG_PRINTLN("等待遥控接入...\r\n");
        LedA_on;LedB_on;LedC_on;LedD_on;Delay(900000);LedA_off;LedB_off;LedC_off;LedD_off;Delay(900000*3);
    }
    DEBUG_PRINTLN("已检测到遥控信号...\r\n");
    while((NRF24L01_RXDATA[30]==0xA5)&&(!ParameterWrite())&&NRF24L01_RXDATA[28]==0xA5)//从上位机读取需要写入的参数，方便上位机调试时使用，不用每次都下程序改参数
    {   
        Nrf_Irq();DEBUG_PRINTLN("等待写入参数...\r\n");
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
        Nrf_Irq();DEBUG_PRINTLN("等待解锁...\r\n");
        LedA_on;LedB_on;LedC_on;LedD_on;Delay(1);LedA_off;LedB_off;LedC_off;LedD_off;Delay(9000);
    }
    for(i=0;i<3;i++)//解锁成功，快速闪烁3次提示
    {
    LedA_on;LedB_on;LedC_on;LedD_on;
    Delay(900000);
    LedA_off;LedB_off;LedC_off;LedD_off;
    Delay(900000);
    }
    DEBUG_PRINTLN("解锁成功,进入飞行模式...\r\n");
}
