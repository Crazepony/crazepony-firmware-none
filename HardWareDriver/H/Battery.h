#ifndef _Battery_H_
#define _Battery_H_
#include "stm32f10x.h"


#define BAT_CHK_PRD 5000	//ms
#define BAT_ALARM_VAL  3.65	  //这是电机未开启的报警电压值，开启电机之后，电池电压会下降0.3-0.4v左右
#define BAT_CHG_VAL    1.0	  // charge battery val.  unit :v
#define BAT_OVERDIS_VAL 3.15		//过放保护电压值，持续低于该电压则自动降落，over discharge protect value


//电压信息结构体
typedef struct
{
    
int    BatteryAD;             //电压AD值
float  BatteryVal;            //电压实际值
float  BatReal;               //电池的实际电压，用万用表测
float  ADRef;                 //AD参考源电压，这里是单片机供电电压，一般在3.3V左右，要实测
float  ADinput;               //AD采样输入电压--->R15和R17相连的焊盘电压
float  Bat_K;                 //计算电压值系数，用于电压校准
char   alarm;									//报警位
char   chargeSta;							//充电状态
int    overDischargeCnt;			//过放保护计数，防止误判
}Bat_Typedef;



void BatteryCheckInit(void);	
u16 Get_Adc_Average(u8 ch,u8 times);             
int GetBatteryAD(void);     
void BatteryCheck(void);
//实例化一个电压信息结构体
extern Bat_Typedef Battery;


#endif
                
        



