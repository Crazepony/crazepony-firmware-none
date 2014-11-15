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
编写者：小马  (Camel)
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

char Cmdreturn[CmdreturnLength];//指令的返回结果缓存


/********************************************
              蓝牙电源初始化函数
********************************************/
void BT_PowerInit(void)
{
    RCC->APB2ENR|=1<<3;      //使能PORTB时钟	
    GPIOB->CRL&=0XFFFFF0FF;  //PB2推挽输出
    GPIOB->CRL|=0X00000300;
    GPIOB->ODR|=1<<2;        //PB2上拉
    BT_off();                //默认关闭
}

/********************************************
              往蓝牙写入一个指令包
********************************************/
void Uart1SendaBTCmd(const char *p)
{
  char i;
  for(i=0;i<CmdreturnLength;i++) Cmdreturn[i] = 0;//释放指令接收缓存
  delay_ms(800);//每两次写入蓝牙之间，要有几百毫秒级别的延时 
  for(i=0;i<strlen(p);i++)
  UART1_Put_Char(*(p+i));  
  delay_ms(800);//写完一条指令，延时500ms再度接收缓存
  i=0;
  while(UartBuf_Cnt(&UartRxbuf) != 0)   //当串口缓冲不为空时，将串口缓冲赋值给指令结果缓冲
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

const char ATcmdAsk[] = {"AT"};
const char ATcmdAnswer[] = {"OK"};

const char ATcmdNameAsk[] = {"AT+NAME?"};
const char ATcmdNameAnswer[] = {"OK+NAME:Crazepony"};
const char ATcmdNameSet[] = {"AT+NAMECrazepony"};//设置蓝牙设备名为：Crazepony，当然可以在这里修改成 what ever you want...

const char ATcmdCodeAsk[] = {"AT+PIN?"};
const char ATcmdCodeAnswer[] = {"OK+PIN:1234"};
const char ATcmdCodeSet[] = {"AT+PIN1234"};      //蓝牙配对密码默认为1234，

const char ATcmdBaudAsk[] = {"AT+BAUD?"};
const char ATcmdBaudAnswer[] = {"OK+BAUD:115200"};
const char ATcmdBaudSet[] = {"AT+BAUD8"};    
      //baud1--->1200
      //baud2--->2400
      //baud3--->4800
      //baud4--->9600
      //baud5--->19200
      //baud6--->38400
      //baud7--->57600
      //baud8--->115200


/********************************************
              写蓝牙参数函数
********************************************/
void BT_ATcmdWrite(void)
{
    BT_on();        //开蓝牙
    delay_ms(1500); //等待蓝牙稳定
    // if(BTParameter.ReadBuf[2] == false)
    // {
    printf("与蓝牙通信中...\r\n");
    UART1_init(SysClock,9600); 
    Uart1SendaBTCmd(ATcmdAsk);
    if(CmdJudgement(ATcmdAnswer) == true)//有蓝牙返回，才往下写指令
    {
        Uart1SendaBTCmd(ATcmdBaudAsk);
        if(CmdJudgement(ATcmdBaudAnswer) == false) {Uart1SendaBTCmd(ATcmdBaudSet);   }
        else ;
        Uart1SendaBTCmd(ATcmdNameAsk);
        if(CmdJudgement(ATcmdNameAnswer) == false)  {Uart1SendaBTCmd(ATcmdNameSet); }   
        else ;
        Uart1SendaBTCmd(ATcmdCodeAsk);
        if(CmdJudgement(ATcmdCodeAnswer) == false) {Uart1SendaBTCmd(ATcmdCodeSet);  }
        else ;
    }
    else  {printf("与蓝牙通信失败\r\n");}
    
    UART1_init(SysClock,115200);
    //   }
    //   else  {printf("蓝牙参数已写入...\r\n");}
    //   
}




