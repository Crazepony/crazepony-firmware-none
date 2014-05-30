#ifndef __UART1_H_
#define __UART1_H_

#include "stm32f10x.h"


void U1NVIC_Configuration(void);
void Uart1_init(u32 pclk2,u32 bound);
void UART1_Put_Char(unsigned char DataToSend);
u8 UART1_Get_Char(void);
void UART1_Putc_Hex(uint8_t b);
void UART1_Putw_Hex(uint16_t w);
void UART1_Putdw_Hex(uint32_t dw);
void UART1_Putw_Dec(uint32_t w);
#endif


