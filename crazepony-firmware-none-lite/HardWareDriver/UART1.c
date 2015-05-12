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
*/
#include "UART1.h"
#include "stdio.h"


//uart reicer flag
#define b_uart_head  0x80
#define b_rx_over    0x40


u8 U1TxBuffer[256];
u8 U1TxPackage[TX_BUFFER_SIZE];
u8 U1TxCounter=0;
u8 U1RxCounter=0;
u8 U1count=0; 
char TxPackFlag;//发送预定格式数据包标志位

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 




/**************************实现函数********************************************
*函数原型:		void U1NVIC_Configuration(void)
*功　　能:		串口1中断配置
输入参数：无
输出参数：没有	
*******************************************************************************/
void UART1NVIC_Configuration(void)
{
        NVIC_InitTypeDef NVIC_InitStructure; 
        /* Enable the USART1 Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
}



/**************************实现函数********************************************
*函数原型:		void Initial_UART1(u32 baudrate)
*功　　能:		初始化UART1
输入参数：u32 baudrate   设置RS232串口的波特率
输出参数：没有	
*******************************************************************************/
void UART1_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
  mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB2ENR|=1<<14;  //使能串口时钟 
	GPIOA->CRH&=0XFFFFF00F;//IO状态设置
	GPIOA->CRH|=0X000008B0;//IO状态设置
	RCC->APB2RSTR|=1<<14;   //复位串口1
	RCC->APB2RSTR&=~(1<<14);//停止复位	   	   
	//波特率设置
 	USART1->BRR=mantissa; // 波特率设置	 
	USART1->CR1|=0X200C;  //1位停止,无校验位.
  USART1->CR1|=1<<8;    //PE中断使能
	USART1->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
 
  UART1NVIC_Configuration();//中断配置
  printf("系统时钟频率：%dMHz \r\n",pclk2);
  printf("串口1初始化波特率：%d \r\n",bound);
 
  
}


/**************************实现函数********************************************
*函数原型:		void UART1_Put_Char(unsigned char DataToSend)
*功　　能:		RS232发送一个字节
输入参数：
		unsigned char DataToSend   要发送的字节数据
输出参数：没有	
*******************************************************************************/
void UART1_Put_Char(unsigned char DataToSend)
{
	U1TxBuffer[U1count++] = DataToSend;  
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  
}
/**************************实现函数********************************************
*函数原型:		u8 UART1_Get_Char(void)
*功　　能:		RS232接收一个字节  一直等待，直到UART1接收到一个字节的数据。
输入参数：		 没有
输出参数：     UART1接收到的数据	
*******************************************************************************/
u8 UART1_Get_Char(void)
{
	while (!(USART1->SR & USART_FLAG_RXNE));
	return(USART_ReceiveData(USART1));
}


/**************************实现函数********************************************
*函数原型:		void UART2_Put_String(unsigned char *Str)
*功　　能:		RS232发送字符串
输入参数：
		unsigned char *Str   要发送的字符串
输出参数：没有	
*******************************************************************************/
void UART1_Put_String(unsigned char *Str)
{
	//判断Str指向的数据是否有效.
	while(*Str){
	//是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
	if(*Str=='\r')UART1_Put_Char(0x0d);
		else if(*Str=='\n')UART1_Put_Char(0x0a);
			else UART1_Put_Char(*Str);
	//等待发送完成.
  	//while (!(USART1->SR & USART_FLAG_TXE));
	//指针++ 指向下一个字节.
	Str++;
	}
/*
	//判断Str指向的数据是否有效.
	while(*Str){
	//是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
	if(*Str=='\r')USART_SendData(USART1, 0x0d);
		else if(*Str=='\n')USART_SendData(USART1, 0x0a);
			else USART_SendData(USART1, *Str);
	//等待发送完成.
  	while (!(USART1->SR & USART_FLAG_TXE));
	//指针++ 指向下一个字节.
	Str++;
	}		 */
}

/**************************实现函数********************************************
*函数原型:		void UART2_Putc_Hex(uint8_t b)
*功　　能:		RS232以十六进制ASCII码的方式发送一个字节数据
				先将目标字节数据高4位转成ASCCII ，发送，再将低4位转成ASCII发送
				如:0xF2 将发送 " F2 "
输入参数：
		uint8_t b   要发送的字节
输出参数：没有	
*******************************************************************************/
void UART1_Putc_Hex(uint8_t b)
{
      /* 判断目标字节的高4位是否小于10 */
    if((b >> 4) < 0x0a)
        UART1_Put_Char((b >> 4) + '0'); //小于10  ,则相应发送0-9的ASCII
    else
        UART1_Put_Char((b >> 4) - 0x0a + 'A'); //大于等于10 则相应发送 A-F

    /* 判断目标字节的低4位 是否小于10*/
    if((b & 0x0f) < 0x0a)
        UART1_Put_Char((b & 0x0f) + '0');//小于10  ,则相应发送0-9的ASCII
    else
        UART1_Put_Char((b & 0x0f) - 0x0a + 'A');//大于等于10 则相应发送 A-F
   UART1_Put_Char(' '); //发送一个空格,以区分开两个字节
}

/**************************实现函数********************************************
*函数原型:		void UART2_Putw_Hex(uint16_t w)
*功　　能:		RS232以十六进制ASCII码的方式发送一个字的数据.就是发送一个int
				如:0x3456 将发送 " 3456 "
输入参数：
		uint16_t w   要发送的字
输出参数：没有	
*******************************************************************************/
void UART1_Putw_Hex(uint16_t w)
{
	//发送高8位数据,当成一个字节发送
    UART1_Putc_Hex((uint8_t) (w >> 8));
	//发送低8位数据,当成一个字节发送
    UART1_Putc_Hex((uint8_t) (w & 0xff));
}

/**************************实现函数********************************************
*函数原型:		void UART2_Putdw_Hex(uint32_t dw)
*功　　能:		RS232以十六进制ASCII码的方式发送32位的数据.
				如:0xF0123456 将发送 " F0123456 "
输入参数：
		uint32_t dw   要发送的32位数据值
输出参数：没有	
*******************************************************************************/
void UART1_Putdw_Hex(uint32_t dw)
{
    UART1_Putw_Hex((uint16_t) (dw >> 16));
    UART1_Putw_Hex((uint16_t) (dw & 0xffff));
}

/**************************实现函数********************************************
*函数原型:		void UART2_Putw_Dec(uint16_t w)
*功　　能:		RS232以十进制ASCII码的方式发送16位的数据.
				如:0x123 将发送它的十进制数据 " 291 "
输入参数：
		uint16_t w   要发送的16位数据值
输出参数：没有	
*******************************************************************************/
void UART1_Putw_Dec(uint32_t w)
{
    uint32_t num = 100000;
    uint8_t started = 0;

    while(num > 0)
    {
        uint8_t b = w / num;
        if(b > 0 || started || num == 1)
        {
            UART1_Put_Char('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}




unsigned char rx_buffer[RX_BUFFER_SIZE];

//------------------------------------------------------
void USART1_IRQHandler(void)
{
  
  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
  {   
    USART_SendData(USART1, U1TxBuffer[U1TxCounter++]);
    USART_ClearITPendingBit(USART1, USART_IT_TXE);  
    if(U1TxCounter == U1count){USART_ITConfig(USART1, USART_IT_TXE, DISABLE);}
  }

  else if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
  rx_buffer[U1RxCounter++]=USART_ReceiveData(USART1);
  if(U1RxCounter==RX_BUFFER_SIZE)
  {
  U1RxCounter=0;
  USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
  }
}


void DEBUG_PRINTLN(unsigned char *Str)
 {
	  UART1_Put_String(Str);  //通过USART1 发送调试信息
 }




