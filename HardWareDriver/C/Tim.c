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
编写者：小马  (Camel)、Nieyong
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.初始化定时器3和定时器4
2.定时器3-->串口打印各种参数（生产调试使用，默认情况下串口输出上位机数据）
3.定时器4-->飞控主循环基准定时器，属于关键中断，将定时器4的主优先级以及从优先级设为最高很有必要
------------------------------------
*/
#include "tim.h"
#include "config.h"
#include "imu.h"

#define TASK_TICK_FREQ				1000			//Hz 主任务频率

uint16_t cntBaro=0;
uint16_t cntBatChk=0;

int LedCounter;//LED闪烁计数值
float Compass_HMC[3];

uint8_t accUpdated=0;
volatile uint16_t anyCnt=0,anyCnt2=0;
uint8_t  loop500HzFlag,loop200HzFlag,loop50HzFlag,loop600HzFlag,loop100HzFlag,loop20HzFlag,loop10HzFlag;
volatile uint16_t loop500Hzcnt,loop200HzCnt,loop50HzCnt , loop600HzCnt,loop100HzCnt, loop20HzCnt , loop10HzCnt=0;


//控制入口
void TIM4_IRQHandler(void)		//1ms中断一次
{
    if( TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET ) 
    {     
					anyCnt++;
					loop200HzCnt++;
					loop100HzCnt++;

					if(++loop50HzCnt * 50 >= (1000))
					{
							loop50HzCnt=0;
							loop50HzFlag=1;
					}
					if(++loop20HzCnt * 20 >=1000 )
					{
							loop20HzCnt=0;
							loop20HzFlag=1;
					}
					if(++loop10HzCnt * 10 >=1000 )
					{
							loop10HzCnt=0;
							loop10HzFlag=1;
					}
          
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
            printf("\r\n");
            printf(" Yaw ---> %5.2f degree\r\n",(float)imu.yaw);
            printf(" Pitch---> %5.2f degree\r\n",(float)imu.pitch);
            printf(" Roll ---> %5.2f degree\r\n",(float)imu.roll);
            printf("====================================\r\n");
            printf(" Motor M1 PWM---> %d\r\n",TIM2->CCR1);
            printf(" Motor M2 PWM---> %d\r\n",TIM2->CCR2);
            printf(" Motor M3 PWM---> %d\r\n",TIM2->CCR3);
            printf(" Motor M4 PWM---> %d\r\n",TIM2->CCR4);
            printf("====================================\r\n");
						printf(" Pressure ---> %5.2f Pa\r\n",(float)MS5611_Pressure);
            printf(" Altitude ---> %5.2f M\r\n",(float)MS5611_Altitude);
            printf(" Temperature---> %5.2f C\r\n",(float)MS5611_Temperature);
						printf("====================================\r\n");
						//根据采集到的AD值，计算实际电压。硬件上是对电池进行分压后给AD采集的，所以结果要乘以2
            printf(" Battery Voltage---> %3.2fv\r\n",Battery.BatteryVal);
            printf("====================================\r\n");
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//串口打印定时器，优先级低于姿态解算
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //TIM4
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//飞控主循环基准定时器，优先级高于串口打印
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

} 

