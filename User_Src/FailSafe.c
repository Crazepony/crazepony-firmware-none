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
Filename:	FailSafe.c
Author:		祥 、小马
------------------------------------
*/

#include "FailSafe.h"
#include "SysConfig.h"
#include "config.h"        //包含所有的驱动头文件
#include "imu.h"
#include "Altitude.h"
#include "CommApp.h"
#include "CommPC.h"
#include "ConfigTable.h"
#include "IMUSO3.h"
#include "control.h"

uint32_t newTime=0;
int LostRCFlag=0;

#define LAND_THRO 				500
#define LAND_THRO_ALT_VEL 200
#define LOST_RC_TIME_MAX  1000


//函数名：FailSafe(void)
//描述：失效保护函数
//失效保护的情景有：侧翻，丢失遥控信号
void FailSafe(void)
{
    uint16_t lostRCTime=0;

    //飞机侧翻，检测到过大的pitch或者roll角度，关闭电机，防止电机堵转烧坏
    //Stop the motors when copter crash down and get a huge pitch or roll value
    if(fabs(imu.pitch)>80 || fabs(imu.roll)>80 )
    {
        MotorPwmFlash(0,0,0,0);
        FLY_ENABLE=0;
    }

    //丢失遥控信号
    //disconnected from the RC
    newTime=millis();	//ms
    lostRCTime=(newTime>lastGetRCTime)?(newTime-lastGetRCTime):(65536-lastGetRCTime+newTime);
    if(lostRCTime > LOST_RC_TIME_MAX) {
        if(offLandFlag || (0 != FLY_ENABLE)) {
            //飞机已经离地offLandFlag，或者已经开启了怠速旋转FLY_ENABLE
            altCtrlMode=LANDING;
        }

        LostRCFlag = 1;
    } else {
        LostRCFlag = 0;
    }
}

//
void AutoLand(void)
{
    static uint32_t landStartTime=0;
    uint32_t landTime=0;

    if(offLandFlag)
    {
        if(landStartTime==0)
            landStartTime=millis();
        landTime=millis() - landStartTime;
        if( landTime>4000)
        {
            altCtrlMode=MANUAL;
            FLY_ENABLE=0;
            offLandFlag=0;
            landStartTime=0;
        }
    }
    else
    {
        altCtrlMode=MANUAL;
        FLY_ENABLE=0;

    }
}
//for copter launch and flight mode switch.  it's raw and simple for climb rate mode now
//TOBE IMPROVED
void FlightModeFSMSimple(void)
{
    if(FLY_ENABLE)
    {
        if(RC_DATA.THROTTLE>=600 )
        {
            if(altCtrlMode!=CLIMB_RATE)
            {
                zIntReset=1;
                thrustZSp=0;
                altCtrlMode=CLIMB_RATE;
                offLandFlag=1;
                altLand=-nav.z;		//记录起飞时的高度
                SetHeadFree(1);
            }
        }
        else
        {
            if(altCtrlMode==MANUAL)
            {
                RC_DATA.THROTTLE=SLOW_THRO;           //手动模式待机转200
            }
        }

    }
}
