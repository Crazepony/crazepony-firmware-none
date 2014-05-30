
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
  * debug.c - Debugging utility functions
  *
  */
#include "tim.h"
#include "config.h"



#define Debug  //调试与否的条件编译


int LedCounter;//LED闪烁计数值


//控制入口
void TIM4_IRQHandler(void)		//1ms中断一次,用于程序读取6050等
{
    if( TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET ) 
    {     
        if(SystemReady_OK)  //整个系统初始化后才开始运行
        {
          Controler(); //控制函数
          LedCounter++;
          if(BatteryAD>BatteryADmin)//当电池电压在设定值之上时，正常模式
          {
          if(LedCounter==50){ LedA_off;LedB_off;}   //遥控端使能后，闪灯提示        
          else if(LedCounter==1000){LedCounter=0;LedA_on;LedB_on;}
          }
          else //电池电压低时，闪灯提示
          {
          if(LedCounter==50){ LedA_off;LedB_off;LedC_off;LedD_off;}   //遥控端使能后，闪灯提示        
          else if(LedCounter==100){LedCounter=0;LedA_on;LedB_on;LedC_on;LedD_on;}
          }
          
          if(LedCounter>=1001)LedCounter=0;
        
        }
        TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);   //清除中断标志   
    }
}



int DebugCounter;

void TIM3_IRQHandler(void)		//1ms中断一次,用于程序读取6050等
{
    if( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET ) 
    {     
        if(SystemReady_OK)  //整个系统初始化后才开始运行
        {
  #ifdef Debug
          
           DebugCounter++;
           BatteryAD=GetBatteryAD();//电池电压检测
          if( DebugCounter==1000){
            DebugCounter=0;
            printf(" ******************************************************************\r\n");
            printf(" *       ____                      _____                  +---+   *\r\n");
            printf(" *      / ___\\                     / __ \\                 | R |   *\r\n");
            printf(" *     / /                        / /_/ /                 +---+   *\r\n");
            printf(" *    / /   ________  ____  ___  / ____/___  ____  __   __        *\r\n");
            printf(" *   / /  / ___/ __ `/_  / / _ \\/ /   / __ \\/ _  \\/ /  / /        *\r\n");
            printf(" *  / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /         *\r\n");
            printf(" *  \\___/_/   \\__,_/ /___/\\___/_/    \\___ /_/ /_/____  /          *\r\n");
            printf(" *                                                  / /           *\r\n");
            printf(" *                                             ____/ /            *\r\n");
            printf(" *                                            /_____/             *\r\n");
            printf(" ******************************************************************\r\n");
            printf("\r\n");
            printf(" Crazepony-II System Runing...\r\n"); 
            printf("\r\n");
            printf("\r\n--->MPU6050 DMP Test<---\r\n");
            printf(" 偏航角---> %f°\r\n",(float)Q_ANGLE.Yaw);
            printf(" 俯仰角---> %f°\r\n",(float)Q_ANGLE.Pitch);
            printf(" 横滚角---> %f°\r\n",(float) Q_ANGLE.Roll);
            printf(" ==================\r\n");
            printf(" X轴期望角度---> %f°\r\n",(float)EXP_ANGLE.X);
            printf(" Y轴期望角度---> %f°\r\n",(float)EXP_ANGLE.Y);
            printf(" ==================\r\n");
            printf(" X轴差分角度---> %f°\r\n",(float)DIF_ANGLE.X);
            printf(" Y轴差分角度---> %f°\r\n",(float)DIF_ANGLE.Y);
            
            printf(" Z轴加速度---> %f\r\n",(float) DMP_DATA.ACCz);
            printf(" ==================\r\n");
            printf(" X轴DMP角速度---> %f\r\n",(float) DMP_DATA.GYROx);
            printf(" Y轴DMP角速度---> %f\r\n",(float) DMP_DATA.GYROy);
            printf(" Z轴DMP角速度---> %f\r\n",(float) DMP_DATA.GYROz);
            printf("==================\r\n");
            printf(" 电池电压---> %d\r\n",(int) BatteryAD);
            printf("==================\r\n");
            
          
            }
#else      
             
#endif
        }
        TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);   //清除中断标志   
    }
}



//定时器4初始化：用来中断处理PID
void TIM4_Init(char clock,int Preiod)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  //打开时钟
    
    TIM_DeInit(TIM4);

    TIM_TimeBaseStructure.TIM_Period = Preiod;
    TIM_TimeBaseStructure.TIM_Prescaler = clock-1;//定时1ms
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM4,TIM_FLAG_Update);

    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM4,ENABLE);
    DEBUG_PRINTLN("定时器4初始化完成...\r\n");
    
}	


//定时器3初始化
void TIM3_Init(char clock,int Preiod)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  //打开时钟
    
    TIM_DeInit(TIM3);

    TIM_TimeBaseStructure.TIM_Period = Preiod;
    TIM_TimeBaseStructure.TIM_Prescaler = clock-1;//定时1ms
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM3,TIM_FLAG_Update);

    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM3,ENABLE);
    DEBUG_PRINTLN("定时器3初始化完成...\r\n");
}		



void TimerNVIC_Configuration()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* NVIC_PriorityGroup 2 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    //TIM3
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //TIM4
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

} 

