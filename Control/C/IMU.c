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
#include "imu.h"
#include "moto.h"
#include "ReceiveData.h"
#include "extern_variable.h"
#include "math.h"
#include "led.h"
#include "DMP.h"


uint8_t FLY_ENABLE=0;
S_FLOAT_ANGLE  Q_ANGLE;	
S_FLOAT_XYZ DIF_ACC;		//差分加速度
S_FLOAT_XYZ EXP_ANGLE;	//期望角度	
S_FLOAT_XYZ DIF_ANGLE;	//实际与期望相差的角度	


//函数名：GET_EXPRAD()
//输入：无
//输出: 无
//描述：为PID控制做期望准备
//作者：马骏
//备注：计算期望角度,不加控制时期望角度为0,0
void GET_EXPRAD(void)			
{
    EXP_ANGLE.X = (float)(RC_DATA.ROOL);
    EXP_ANGLE.Y = (float)(RC_DATA.PITCH);
    EXP_ANGLE.Z = (float)(RC_DATA.YAW);
    
    DIF_ANGLE.X = EXP_ANGLE.X - Q_ANGLE.Roll;
    DIF_ANGLE.X = DIF_ANGLE.X*20;
  
    DIF_ANGLE.Y = EXP_ANGLE.Y - Q_ANGLE.Pitch;
    DIF_ANGLE.Y = DIF_ANGLE.Y*20;
    DIF_ACC.Z =  (DMP_DATA.ACCz/100)-77;//Z轴加速度差分值，保证飞机起飞不会很冲
  
}
