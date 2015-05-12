#ifndef _Battery_H_
#define _Battery_H_
#include "stm32f10x.h"


#define BAT_CHK_PRD 5000	//ms
#define BAT_ALARM_VAL  3.7	  //on ground
#define BAT_CHG_VAL    1.0	  // charge battery val.  unit :v



//电压信息结构体
typedef struct
{
    
int    BatteryAD;             //电压AD值
float  BatteryVal;            //电压实际值
float  BatReal;               //电池的实际电压，用万用表测
float  ADRef;                 //AD参考源电压，这里是单片机供电电压，一般在3.3V左右，要实测
float  ADinput;               //AD采样输入电压--->R15和R17相连的焊盘电压
float  Bat_K;                 //计算电压值系数，用于电压校准
int    BatteryADmin;          //电压门限
char   alarm;									//报警位
char   chargeSta;							//充电状态
}Bat_Typedef;



void BatteryCheckInit(void);	
u16 Get_Adc_Average(u8 ch,u8 times);             
int GetBatteryAD(void);     
void BatteryCheck(void);
//实例化一个电压信息结构体
extern Bat_Typedef Battery;


#endif
                
        



