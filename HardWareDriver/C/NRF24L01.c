
 /*    
  *      ____                      _____                  +---+
  *     / ___\                     / __ \                 | R |
  *    / /                        / /_/ /                 +---+
  *   / /   ________  ____  ___  / ____/___  ____  __   __
  *  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
  * / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
  * \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
  *                                                 / /
  *                                            ____/ /
  *                                           /_____/
  *                                       
  *  Crazyfile control firmware                                        
  *  Copyright (C) 2011-2014 Crazepony-II                                        
  *
  *  This program is free software: you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License as published by
  *  the Free Software Foundation, in version 3.
  *
  *  This program is distributed in the hope that it will be useful,
  *  but WITHOUT ANY WARRANTY; without even the implied warranty of
  *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  *  GNU General Public License for more details.
  * 
  * You should have received a copy of the GNU General Public License
  * along with this program. If not, see <http://www.gnu.org/licenses/>.
  *
  *
  * debug.c - Debugging utility functions
  *
  */

#include "NRF24L01.h"
#include "spi.h"
#include "ReceiveData.h"
#include "delay.h"
	 
uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];//nrf24l01接收到的数据
uint8_t NRF24L01_TXDATA[RX_PLOAD_WIDTH];//nrf24l01需要发送的数据



//修改该接收和发送地址，可以供多个飞行器在同一区域飞行，数据不受干扰
u8  TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0xc3,0x10,0x10,0x11};	//本地地址
u8  RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0xc3,0x10,0x10,0x11};	//接收地址				



//写寄存器
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    SPI_CSN_L();					  
    status = SPI_RW(reg);  
    SPI_RW(value);		  /* 写数据 */
    SPI_CSN_H();					  /* 禁止该器件 */
    return 	status;
}


//读寄存器
uint8_t NRF_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    SPI_CSN_L();					 
    SPI_RW(reg);			  
    reg_val = SPI_RW(0);	  /* 读取该寄存器返回数据 */
    SPI_CSN_H();	
 
    return 	reg_val;
}


//写缓冲区
uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
    uint8_t i;
    uint8_t status;
    SPI_CSN_L();				        /* 选通器件 */
    status = SPI_RW(reg);	/* 写寄存器地址 */
    for(i=0; i<uchars; i++)
    {
        SPI_RW(pBuf[i]);		/* 写数据 */
    }
    SPI_CSN_H();						/* 禁止该器件 */
    return 	status;	
}


//读缓冲区
uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
    uint8_t i;
    uint8_t status;
    SPI_CSN_L();						/* 选通器件 */
    status = SPI_RW(reg);	/* 写寄存器地址 */
    for(i=0; i<uchars; i++)
    {
        pBuf[i] = SPI_RW(0); /* 读取返回数据 */ 	
    }
    SPI_CSN_H();						/* 禁止该器件 */
    return 	status;
}


//写数据包
void NRF_TxPacket(uint8_t * tx_buf, uint8_t len)
{	
    SPI_CE_L();		 //StandBy I模式	
    NRF_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			 // 装载数据	
    SPI_CE_H();		 //置高CE，激发数据发送
}

//初始化
char NRF24L01_INIT(void)
{

   SPI1_INIT();
   return NRF24L01_Check();
}


//接收模式
void SetRX_Mode(void)
{
    SPI_CE_L();
   
	  NRF_Write_Reg(FLUSH_RX,0xff);//清除TX FIFO寄存器			 
  	NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
   	NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    
  	NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址  	 
  	NRF_Write_Reg(NRF_WRITE_REG+RF_CH,40);	     //设置RF通信频率		  
  	NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
  	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
    
    
    SPI_CE_H();
} 


//发送模式
void SetTX_Mode(void)
{
    SPI_CE_L();
   
    NRF_Write_Reg(FLUSH_TX,0xff);										//清除TX FIFO寄存器		  
  	
    NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);		//写TX节点地址 
  	NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); 	//设置TX节点地址,主要为了使能ACK	  

  	NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
  	NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
  	NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	NRF_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //设置RF通道为40
  	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
    
    
    SPI_CE_H();
} 


//查询中断
void Nrf_Irq(void)
{
    uint8_t sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);
    if(sta & (1<<RX_DR))//接收中断
    {
        NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
         
        Get_ReceiveData();      //自己做修改

    }
 
    NRF_Write_Reg(0x27, sta);//清除nrf的中断标志位
}


//接收函数
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    							   
        //SPI2_SetSpeed(SPI_SPEED_4); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   
	sta=NRF_Read_Reg(NRFRegSTATUS);  //读取状态寄存器的值    	 
	NRF_Write_Reg(NRF_WRITE_REG+NRFRegSTATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//接收到数据
	{
		NRF_Write_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
		return 0; 
	}	   
	return 1;//没收到任何数据
}		



//判断SPI接口是否可用
u8 NRF24L01_Check(void) 
{ 
   u8 buf[5]={0xC2,0xC2,0xC2,0xC2,0xC2}; 
   u8 buf1[5]; 
   u8 i=0; 
    
   /*写入5 个字节的地址.  */ 
   NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5); 
     
   /*读出写入的地址 */ 
   NRF_Read_Buf(TX_ADDR,buf1,5); 
   
    /*比较*/ 
   for (i=0;i<5;i++) 
   { 
      if (buf1[i]!=0xC2) 
      break; 
   } 
  
   if (i==5)   return 1 ;        //MCU 与NRF 成功连接 
   else        return 0 ;        //MCU与NRF不正常连接    
} 

