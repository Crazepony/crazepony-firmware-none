#ifndef __UART1_H_
#define __UART1_H_

#include "stm32f10x.h"


// USART Receiver buffer
#define RX_BUFFER_SIZE   3
#define TX_BUFFER_SIZE   3



void DEBUG_PRINTLN(unsigned char *Str);
void UART1NVIC_Configuration(void);
void UART1_init(u32 pclk2,u32 bound);
void UART1_Put_Char(unsigned char DataToSend);
u8 UART1_Get_Char(void);
void UART1_Putc_Hex(uint8_t b);
void UART1_Putw_Hex(uint16_t w);
void UART1_Putdw_Hex(uint32_t dw);
void UART1_Putw_Dec(uint32_t w);
void UART1_Put_String(unsigned char *Str);

extern unsigned char rx_buffer[RX_BUFFER_SIZE];
#endif


