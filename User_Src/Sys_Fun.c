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
//不能在这里执行所有外设复位!否则至少引起串口不工作.
//把所有时钟寄存器复位
void MYRCC_DeInit(void)
{
    RCC->APB1RSTR = 0x00000000;//复位结束
    RCC->APB2RSTR = 0x00000000;

    RCC->AHBENR = 0x00000014;  //睡眠模式闪存和SRAM时钟使能.其他关闭.
    RCC->APB2ENR = 0x00000000; //外设时钟关闭.
    RCC->APB1ENR = 0x00000000;
    RCC->CR |= 0x00000001;     //使能内部高速时钟HSION
    RCC->CFGR &= 0xF8FF0000;   //复位SW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0]
    RCC->CR &= 0xFEF6FFFF;     //复位HSEON,CSSON,PLLON
    RCC->CR &= 0xFFFBFFFF;     //复位HSEBYP
    RCC->CFGR &= 0xFF80FFFF;   //复位PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE
    RCC->CIR = 0x00000000;     //关闭所有中断
}


/********************************************
           使用内部DCO配置系统时钟
功能：
1.使用内部HSI时钟二分频（4MHz）作为PLL输入
2.PLL倍频系数PLLMUL<=9(实际到达13时，还能正常倍频内部时钟)
3.输入参数：PLLMUL，PLL倍频系数
4.备注：官方手册上说，使用HSI时，最高到达到36M，实际可以达到52M。
********************************************/
char SystemClock_HSI(u8 PLL)
{
    RCC->CR|=1<<0;              //内部高速时钟使能
    RCC->CR|=0<<16;              //外部高速时钟关闭
    RCC->CR|=1<<18;
    while(!((RCC->CR)&(1<<1))); //等待内部时钟稳定就绪
    RCC->CFGR|=(PLL-2)<<18;     //PPL倍频系数
    RCC->CFGR|=0<<16;           //PPL输入时钟源,HSI二分频后作为PLL输入源=4MHz
    RCC->CR|=1<<24;             //PLL使能
    while(!((RCC->CR)&(1<<25)));//等待PLL稳定
    RCC->CFGR|=2<<0;            //系统时钟源配置，PLL输出作为系统时钟
    SysClock=4*PLL;             //返回系统时钟，单位MHz
    return SysClock;
}


/********************************************
           使用外部晶体作为系统时钟源
功能：
1.使用外部HSE时钟8M作为PLL输入
2.PLL倍频系数PLLMUL<=9(实际到达16时，还能正常倍频外部时钟)
3.输入参数：PLLMUL，PLL倍频系数
4.备注：官方手册上说，使用HSE作为系统时钟源时，最高可倍频到72MHz，但是实际可以倍频到128M系统还算稳定
********************************************/
//系统时钟初始化函数
//pll:选择的倍频数，从2开始，最大值为16
//时钟源为外部晶振
//备注：当机身焊接了8M晶振时，就只能使用外部8M晶振作为时钟源，
//      用内部的HSI不好使，我反正没调出来，看各位有啥办法没
char SystemClock_HSE(u8 PLL)
{
    unsigned char temp=0;
    MYRCC_DeInit();		    //复位并配置向量表
    RCC->CR|=1<<16;       //外部高速时钟使能HSEON
    while(!(RCC->CR>>17));//等待外部时钟就绪
    RCC->CFGR=0X00000400; //APB1=DIV2;APB2=DIV1;AHB=DIV1;
    PLL-=2;//抵消2个单位
    RCC->CFGR|=PLL<<18;   //设置PLL值 2~16
    RCC->CFGR|=1<<16;	    //PLLSRC ON
    FLASH->ACR|=0x32;	    //FLASH 2个延时周期
    RCC->CR|=0x01000000;  //PLLON
    while(!(RCC->CR>>25));//等待PLL锁定
    RCC->CFGR|=0x00000002;//PLL作为系统时钟
    while(temp!=0x02)     //等待PLL作为系统时钟设置成功
    {
        temp=RCC->CFGR>>2;
        temp&=0x03;
    }

    SysClock=(PLL+2)*8;
    return SysClock;
}

/********************************************
              开机LED的各种状态
********************************************/
void PowerOn()
{
    char i;            //循环变量

    for(i=0; i<4; i++) //循环闪烁4次
    {
        LedA_on;
        LedB_off;
        LedC_off;
        LedD_off;
        Delay(900000);
        LedA_off;
        LedB_on;
        LedC_off;
        LedD_off;
        Delay(900000);
        LedA_off;
        LedB_off;
        LedC_on;
        LedD_off;
        Delay(900000);
        LedA_off;
        LedB_off;
        LedC_off;
        LedD_on;
        Delay(900000);
    }

    for(i=0; i<3; i++) //解锁成功，快速闪烁3次提示
    {
        LedA_on;
        LedB_on;
        LedC_on;
        LedD_on;
        Delay(900000);
        LedA_off;
        LedB_off;
        LedC_off;
        LedD_off;
        Delay(900000);
    }
    printf("Armed success...\r\n");
}











