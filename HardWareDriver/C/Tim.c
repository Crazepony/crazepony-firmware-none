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
Tim.c file
编写者：小马  (Camel)
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.初始化定时器3和定时器4
2.定时器3-->串口打印各种参数
3.定时器4-->姿态解算以及PID输出，属于关键中断，将定时器4的主优先级以及从优先级设为最高很有必要
------------------------------------
*/
#include "tim.h"
#include "config.h"



int LedCounter;//LED闪烁计数值
float Compass_HMC[3];

//控制入口
void TIM4_IRQHandler(void)		//1ms中断一次,用于程序读取6050等
{
    if( TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET ) 
    {     
          Controler(); //控制函数
                  
          //HMC58X3_mgetValues(&Compass_HMC[0]);       
          LedCounter++;//led闪烁计数值
          if(Battery.BatteryAD > Battery.BatteryADmin)//当电池电压在设定值之上时，正常模式
          {
              if(LedCounter==10){ LedA_off;LedB_off;}   //遥控端使能后，闪灯提示        
              else if(LedCounter==30){LedCounter=0;LedA_on;LedB_on;}
          }
          else //电池电压低时，闪灯提示
          {
              if(LedCounter==10){ LedA_off;LedB_off;LedC_off;LedD_off;}   //遥控端使能后，闪灯提示        
              else if(LedCounter==20){LedCounter=0;LedA_on;LedB_on;LedC_on;LedD_on;}
          }
          if(LedCounter>=31)LedCounter=0;

          
          
          TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);   //清除中断标志   
    }
}



int DebugCounter;             //打印信息输出时间间隔计数值


void TIM3_IRQHandler(void)		//打印中断服务程序
{
    if( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET ) 
    {     
       

           
           Battery.BatteryAD  = GetBatteryAD();            //电池电压检测  
           Battery.BatteryVal = Battery.Bat_K * (Battery.BatteryAD/4096.0) * Battery.ADRef;//实际电压 值计算
#ifdef Debug
      DebugCounter++;
      if( DebugCounter==500)
            {
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
            printf(" Crazepony-II报告：系统正在运行...\r\n"); 
            printf("\r\n");
            printf("\r\n--->机身实时姿态广播信息<---\r\n");
            printf("\r\n");
            printf(" 偏航角---> %5.2f°\r\n",(float)Q_ANGLE.Yaw);
            printf(" 俯仰角---> %5.2f°\r\n",(float)Q_ANGLE.Pitch);
            printf(" 横滚角---> %5.2f°\r\n",(float) Q_ANGLE.Roll);
            printf(" ==================\r\n");
            printf(" X轴期望角度---> %5.2f°\r\n",(float)EXP_ANGLE.X);
            printf(" Y轴期望角度---> %5.2f°\r\n",(float)EXP_ANGLE.Y);
            printf(" Z轴期望角度---> %5.2f°\r\n",(float)EXP_ANGLE.Z);
            
            printf(" ==================\r\n");
            printf(" Y轴误差角度---> %5.2f°\r\n",(float)DIF_ANGLE.Y);
            printf(" X轴误差角度---> %5.2f°\r\n",(float)DIF_ANGLE.X);
            printf("==================\r\n");
            printf(" X轴加速度---> %5.2fm/s2\r\n",(float) DMP_DATA.dmp_accx);
            printf(" Y轴加速度---> %5.2fm/s2\r\n",(float) DMP_DATA.dmp_accy);
            printf(" Z轴加速度---> %5.2fm/s2\r\n",(float) DMP_DATA.dmp_accz);
            
            printf(" ==================\r\n");
            printf(" X轴角速度---> %5.2f °/s\r\n",(float) DMP_DATA.dmp_gyrox);
            printf(" Y轴角速度---> %5.2f °/s\r\n",(float) DMP_DATA.dmp_gyroy);
            printf(" Z轴角速度---> %5.2f °/s\r\n",(float) DMP_DATA.dmp_gyroz);
            printf("==================\r\n");
            printf(" 电机M1 PWM值---> %d\r\n",TIM2->CCR1);
            printf(" 电机M2 PWM值---> %d\r\n",TIM2->CCR2);
            printf(" 电机M3 PWM值---> %d\r\n",TIM2->CCR3);
            printf(" 电机M4 PWM值---> %d\r\n",TIM2->CCR4);
            printf("==================\r\n");
            printf(" 电池电压---> %3.2fv\r\n",Battery.BatteryVal);//根据采集到的AD值，计算实际电压。硬件上是对电池进行分压后给AD采集的，所以结果要乘以2
            printf("==================\r\n");
          
//             printf(" ---> %d\r\n",(int) PIDParameter.ReadBuf[0]);
//             printf(" ---> %d\r\n",(int) PIDParameter.ReadBuf[1]);
//             printf(" ---> %d\r\n",(int) PIDParameter.ReadBuf[2]);
//             
//             printf(" ---> %d\r\n",(int) BTParameter.ReadBuf[0]);
//             printf(" ---> %d\r\n",(int) BTParameter.ReadBuf[1]);
//             printf(" ---> %d\r\n",(int) BTParameter.ReadBuf[2]);
//             
            
// 
//             printf(" X磁场强度---> %5.2f °/s\r\n",(float) Compass_HMC[0]);
//             printf(" Y磁场强度---> %5.2f °/s\r\n",(float) Compass_HMC[1]);
//             printf(" Z磁场强度---> %5.2f °/s\r\n",(float) Compass_HMC[2]);
//       


        }
#else      
             
#endif
        
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
    printf("定时器4初始化完成...\r\n");
    
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
  
    printf("定时器3初始化完成...\r\n");
}		


void TimerNVIC_Configuration()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* NVIC_PriorityGroup 2 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    //TIM3
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//定时器3作为串口打印定时器，优先级低于姿态解算
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //TIM4
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//定时器4作为姿态解算，优先级高于串口打印
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

} 

