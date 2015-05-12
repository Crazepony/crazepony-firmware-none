#ifndef _Led_H_
#define _Led_H_
#include "stm32f10x.h"


//
#define LED_NUM 4
#define LA	    0x01
#define LB      0x02
#define LC      0x04
#define LD      0x08
//
#define E_READY 		   0
#define E_CALI			   1
#define E_BAT_LOW		   2
#define E_CALI_FAIL	   3
#define E_LOST_RC 	   4
#define E_AUTO_LANDED  5
#define E_BatChg       6


typedef union{
	uint8_t byte;
	struct 
	{
			uint8_t A	:1;
		  uint8_t B	:1;
			uint8_t C	:1;
		  uint8_t D	:1;
			uint8_t reserved	:4;
	}bits;
}LEDBuf_t;

typedef struct Led_tt
{
uint8_t event;
uint8_t state;
uint16_t cnt;
}LED_t;

#define LedA_on    GPIO_SetBits(GPIOA, GPIO_Pin_11)
#define LedA_off   GPIO_ResetBits(GPIOA, GPIO_Pin_11)


#define LedB_on    GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define LedB_off   GPIO_ResetBits(GPIOA, GPIO_Pin_8)


#define LedC_on    GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define LedC_off   GPIO_ResetBits(GPIOB, GPIO_Pin_1)


#define LedD_on    GPIO_SetBits(GPIOB, GPIO_Pin_3)
#define LedD_off   GPIO_ResetBits(GPIOB, GPIO_Pin_3)

#define LEDA_troggle GPIO_WriteBit(GPIOA,GPIO_Pin_11, !GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_11))
#define LEDB_troggle GPIO_WriteBit(GPIOA,GPIO_Pin_8, !GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_8))
#define LEDC_troggle GPIO_WriteBit(GPIOB,GPIO_Pin_1, !GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_1))


extern LED_t LEDCtrl;

void LedInit(void);   //Led初始化函数外部声明
void LEDFSM(void);



#endif

