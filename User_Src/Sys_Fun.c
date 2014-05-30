#include "config.h"
#include "extern_variable.h"

uint32_t IMU_CYCTIME;

 
void NVIC_INIT(void)
{
    TimerNVIC_Configuration();//定时器中断配置
    U1NVIC_Configuration();  //串口1中断配置
}

/**************************实现函数********************************************
*函数原型:		
*功　　能:		
*******************************************************************************/
void SYSTICK_INIT(void)
{
    SysTick->LOAD = 0xFFFFFF;
    SysTick->VAL = 0x0;
    SysTick->CTRL = 0x01;//启用systick,8分频(9M),关中断,本寄存器第四位为重载标志,=1说明重载了,读取本寄存器第四位自动清零
    
}


uint32_t GET_NOWTIME(uint32_t * lasttime)//返回当前systick计数器值,32位
{
    uint32_t temp,temp1,temp2;
    
    temp1 = SysTick->VAL;
    temp = SysTick->CTRL;
    if(temp&(1<<16)) 
            temp2 = *lasttime + 0xffffff - temp1;//发生重载
    else
            temp2 = *lasttime - temp1;
    *lasttime = temp1;
    if(temp2>100000)	return 0;
    return temp2;
}



