#ifndef _SPI_H_
#define _SPI_H_
#include "stm32f10x.h"
/*
6.2.4G:	        CE:PA15
		CSN:PA4
		SCK:PA5
		MOSI:PA7
		MISO:PA6
		IRQ:PA8
*/
#define SPI_CE_H()   GPIO_SetBits(GPIOA, GPIO_Pin_12) 
#define SPI_CE_L()   GPIO_ResetBits(GPIOA, GPIO_Pin_12)

#define SPI_CSN_H()  GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define SPI_CSN_L()  GPIO_ResetBits(GPIOA, GPIO_Pin_4)

void SPI1_INIT(void);
u8 SPI_RW(u8 dat);

#endif

