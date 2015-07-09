## 概述说明
这是Crazepony微型四轴飞行器飞控代码，未使用实时操作系统。

使用Keil 5进行编译开发，使用UTF-8编码。更详细的开发操作，见Crazepony百科的[开发指南](http://www.crazepony.com/wiki-crazepony.html#rd)。

配合Crazepony微型四轴遥控器代码[crazepony-remote-none](https://github.com/Crazepony/crazepony-remote-none)使用。

该代码默认的飞行模式是无头模式。

开启SysConfig.h文件中的宏`UART_DEBUG`，可以开启USB串口的打印调试。关闭该宏，则USB口用于和上位机通信，在串口终端上只会看到乱码。

## Overview
This is the flight control source code of Crazepony MINI quadcopter.The suffix none means no RTOS used.

The project is developed in [Keil MDK Version 5 IDE](http://www2.keil.com/mdk5/).You could just import the project to Keil 5 and compile it.The Makefile for cross toolchain in Linux is comming soon.

The default mode is Head-free.

To use USB for printf debug,define the macro `UART_DEBUG` in SysConfig.h.
