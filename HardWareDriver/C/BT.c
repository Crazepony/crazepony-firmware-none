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
BT.c file
编写者：小马  (Camel)，nieyong
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.蓝牙透传模块的电源使能端BT_EN--->PB2
2.打开蓝牙电源-->BT_EN=1;
------------------------------------
*/


#include "BT.h"
#include "delay.h"
#include "UART1.h"
#include "stdio.h"
#include "string.h"
#include "sys_fun.h"
#include "control.h"
#include "Led.h"
	
BTtype CrazepoyBT;//实例化一个蓝牙结构体
float BTstate;//蓝牙是否需要写参数标志
	
	
/********************************************
              蓝牙电源初始化函数
********************************************/
void BT_PowerInit(void)
{
    RCC->APB2ENR|=1<<3;      //使能PORTB时钟	
    GPIOB->CRL&=0XFFFFF0FF;  //PB2推挽输出
    GPIOB->CRL|=0X00000300;
    GPIOB->ODR|=1<<2;        //PB2上拉
    BT_on();   

}


char Cmdreturn[CmdreturnLength];//指令的返回结果缓存

/********************************************
              往蓝牙写入一个指令包
********************************************/
void Uart1SendaBTCmd(const char *p)
{
  char i;
	
  for(i=0;i<CmdreturnLength;i++) Cmdreturn[i] = 0;//释放指令接收缓存
	LedA_on;
	delay_ms(100);//写完一条指令，延时500ms再度接收缓存
	LedA_off;
  for(i=0;i<strlen(p);i++)
  UART1_Put_Char(*(p+i));  
  delay_ms(100);//写完一条指令，延时500ms再度接收缓存
	
	
  i=0;
  while(UartBuf_Cnt(&UartRxbuf) != 0)     //当串口缓冲不为空时，将串口缓冲赋值给指令结果缓冲
  Cmdreturn[i++] = UartBuf_RD(&UartRxbuf);
}

/********************************************
         判断一个指令返回是不是等于设定值
         返回值：0-->指令与设定值不匹配
                 1-->指令与设定值匹配
********************************************/
char CmdJudgement(const char *p)
{
  char i;
  for(i=0;i<strlen(p);i++) if(Cmdreturn[i] != *(p+i)) break;
  if(i != strlen(p)) return 0;
  return 1;
}

const char ATcmdAsk[]    =		 {"AT"};
const char ATcmdAnswer[] =     {"OK"};

const char ATcmdNameAsk[] = 	 {"AT+NAME?"};
const char ATcmdNameAnswer[] =  {"OK+NAME:Crazepony"};	//{BT_BAUD_AT};//
const char ATcmdNameSet[] = 	 {"AT+NAMECrazepony"};    //设置蓝牙设备名为：Crazepony，当然可以在这里修改成 what ever you want...

const char ATcmdCodeAsk[] = 	 {"AT+PIN?"};
const char ATcmdCodeAnswer[] = {"OK+PIN:1234"};	
const char ATcmdCodeSet[] =		 {"AT+PIN1234"};          //蓝牙配对密码默认为1234

const char ATcmdRenewAsk[] = 	 {"AT+RENEW"};	//恢复出厂设置
const char ATcmdRenewAnswer[] = {"OK+RENEW"};	

const char ATcmdBaudAsk[] =		 {"AT+BAUD?"};
const char ATcmdBaudAnswer[] = {"OK+BAUD:115200"};
const char ATcmdBaudSet[] =    {"AT+BAUD4"};            //修改此处，可以修改蓝牙波特率
//HM-06模块，即蓝牙2.1模块的配置
//baud1--->1200
//baud2--->2400
//baud3--->4800
//baud4--->9600
//baud5--->19200
//baud6--->38400
//baud7--->57600
//baud8--->115200

//HM-11模块，即蓝牙4.0 BLE模块的配置，注意和HM-06的区别
//baud0--->9600
//baud4--->115200                                        


//轮询蓝牙模块所有可能的波特率，获取当前波特率
//并且配置其波特率为115200，
u32 BT_Scan_Buad(void)
{
	//蓝牙波特率率表，将9600（默认波特率）和230400（hm-06遗留bug）放到最前
	//115200（将要配置的波特率）放到最后
	static u32 bandsel[9] = {230400,9600,1200,2400,4800,19200,38400,57600,115200};
  u8 i;
	
	for(i=0;i<9;i++)
	{
		UART1_init(SysClock,bandsel[i]); 
		Uart1SendaBTCmd(ATcmdAsk);
		if(CmdJudgement(ATcmdAnswer) == true)
		{
			return bandsel[i];
		}
	}
	
	return 115200;
}

/********************************************
              写蓝牙参数函数
********************************************/
void BT_ATcmdWrite(void)
{	
	u8 i;
	static	u32 BT_CurBaud;
	static u32 bandsel[9] = {230400,9600,1200,2400,4800,19200,38400,57600,115200};
	
	Uart1SendaBTCmd(ATcmdAsk);
	if(CmdJudgement(ATcmdAnswer) == true)
	{
		//HM-11模块已经是115200波特率，无需配置
		return ;
	}
	
	printf("BT baund check and init begin.printf is useless.\r\n\r\n");
		
	BT_CurBaud = BT_Scan_Buad();
	
	
	//首先检测蓝牙模块的串口是否已经配置为115200
	if(BT_CurBaud != BT_BAUD_Set){
		
		//蓝牙模块需要修改其名字，波特率
		
		//修改蓝牙的名字为Crazepony
		Uart1SendaBTCmd(ATcmdNameSet);
		
		//修改蓝牙波特率为115200
		Uart1SendaBTCmd(ATcmdBaudSet);
		
		//LED闪烁表示原来蓝牙模块是哪个波特率
		for(i=0;i<9;i++)
		{
			LedA_on;LedB_on;LedC_on;LedD_on;
			delay_ms(1000);
			LedA_off;LedB_off;LedC_off;LedD_off;
			delay_ms(1000);
			
			if(BT_CurBaud == bandsel[i]){
				break;
			}
		}
	}else{
		//已经是115200，可以直接通信
		printf("BT module baud is 115200 okay\r\n");
	}
			
	//最终STM32的UART波特率设置回115200
	UART1_init(SysClock,BT_BAUD_Set);
	
	printf("\r\nBT baund check and init end.\r\n");
	
}

