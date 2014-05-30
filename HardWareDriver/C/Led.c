
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
#include "Led.h"
/********************************************
              Led初始化函数
功能：
1.配置Led接口IO输出方向
2.关闭所有Led(开机默认方式)

描述：
Led接口：
Led1-->PB5
Led2-->PB4
Led3-->PB1
Led4-->PB3
BT_EN-->PB2
对应IO=1，灯亮
********************************************/
void LedInit(void)
{
    RCC->APB2ENR|=1<<3;    //使能PORTB时钟	
    RCC->APB2ENR|=1<<4;    //使能PORTC时钟	
    
    RCC->APB2ENR|=1<<0;      //使能复用时钟	   
    GPIOB->CRL&=0XFF00000F;  //PB1,2,3,4,5推挽输出
    GPIOB->CRL|=0X00333330;
    GPIOB->ODR|=31<<1;        //PB1,2,3,4,5上拉

    AFIO->MAPR|=2<<24;      //关闭JATG,千万不能将SWD也关闭，否则芯片作废，亲测!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    LedA_off;LedB_off;LedC_off;LedD_off;
   
}



