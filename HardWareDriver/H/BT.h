#ifndef _BT_H_
#define _BT_H_
#include "stm32f10x.h"
#include "UART1.h"

#define true  1
#define false !true

#define BT_BAUD_Set     115200
#define BT_NAME 				"Crazepony"
#define BT_PIN 				  "1234"
//AT 字符串
#define BT_BAUD_AT		 "OK+NAME:" BT_NAME

#define BT_on()      {GPIO_SetBits(GPIOB, GPIO_Pin_2);printf("蓝牙电源初始开启完成...\r\n");}
#define BT_off()     {GPIO_ResetBits(GPIOB, GPIO_Pin_2);printf("蓝牙电源初始化关闭完成...\r\n");}//宏定义蓝牙开关

#define CmdreturnLength 20

#define BThavewrote        0xa5
#define BTneedwrite        0x5a

#define BT_NAMEmax  10
#define BT_PINmax   10


typedef struct
{
 u8 Name[BT_NAMEmax];
 u32 Baud;
 u8 PinCode[BT_PINmax];
}BTtype;



void BT_PowerInit(void);   //蓝牙透传电源初始化
void BT_ATcmdWrite(void);//蓝牙写参数

extern float BTstate;

#endif

