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
Filename:	imu.c
Author:		祥 、小马、nieyong
------------------------------------
*/
#include "config.h"
#include "DMP.h"
#include "imu.h"
#include "filter.h"
#include "SysConfig.h"


imu_t imu= {0};
uint8_t imuCaliFlag=0;

//函数名：IMU_Init(void)
//描述：姿态解算融合初始化函数
//现在使用软件解算，不再使用MPU6050的硬件解算单元DMP，IMU_SW在SysConfig.h中定义
void IMU_Init(void)
{
#ifdef IMU_SW		//软解需要先校陀螺
    imu.ready=0;
#else
    imu.ready=1;
#endif
    imu.caliPass=1;
    //filter rate
    LPF2pSetCutoffFreq_1(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);		//30Hz
    LPF2pSetCutoffFreq_2(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutoffFreq_3(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutoffFreq_4(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutoffFreq_5(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutoffFreq_6(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
}



//函数名：IMU_Process(void)
//描述：姿态解算融合函数
//该函数对姿态的融合是硬件解算，也就是基于DMP数据进行的，Crazepony现在不使用DMP硬件解算
//Crazepony现在使用软件解算，即函数IMUSO3Thread()
void IMU_Process(void)
{
    //read ADC
    imu.accADC[0]=DMP_DATA.ACCx;
    imu.accADC[1]=DMP_DATA.ACCy;
    imu.accADC[2]=DMP_DATA.ACCz;

    imu.gyroADC[0]=DMP_DATA.GYROx;
    imu.gyroADC[1]=DMP_DATA.GYROy;
    imu.gyroADC[2]=DMP_DATA.GYROz;
    //turn physical
    imu.accRaw[0]=DMP_DATA.dmp_accx;
    imu.accRaw[1]=DMP_DATA.dmp_accy;
    imu.accRaw[2]=DMP_DATA.dmp_accz;

    imu.gyroRaw[0]=DMP_DATA.dmp_gyrox ;
    imu.gyroRaw[1]=DMP_DATA.dmp_gyroy ;
    imu.gyroRaw[2]=DMP_DATA.dmp_gyroz ;

    //filter
    imu.accb[0]=LPF2pApply_1(imu.accRaw[0]-imu.accOffset[0]);
    imu.accb[1]=LPF2pApply_2(imu.accRaw[1]-imu.accOffset[1]);
    imu.accb[2]=LPF2pApply_3(imu.accRaw[2]-imu.accOffset[2]);

    imu.gyro[0]=LPF2pApply_4(imu.gyroRaw[0]-imu.gyroOffset[0]);
    imu.gyro[1]=LPF2pApply_5(imu.gyroRaw[1]-imu.gyroOffset[1]);
    imu.gyro[2]=LPF2pApply_6(imu.gyroRaw[2]-imu.gyroOffset[2]);

    imu.pitch=Q_ANGLE.Pitch;
    imu.roll=Q_ANGLE.Roll;
    imu.yaw=Q_ANGLE.Yaw;

    //get DCM
    eular2DCM(imu.DCMgb,imu.roll,imu.pitch,imu.yaw);

}


//should place to a level surface and keep it stop for 1~2 second
//return 1 when finish
uint8_t IMU_Calibrate(void)
{
    //3s
    static float accSum[3]= {0,0,0};
    static float gyroSum[3]= {0,0,0};
    static uint16_t cnt=0;
    static uint16_t tPrev=0;
    static uint8_t calibrating=0;
    uint8_t ret=0;
    uint8_t i=0;
    uint16_t dt=0,now=0;

    now=millis();
    dt=now-tPrev;

    if(calibrating==0)
    {
        calibrating=1;
        for(i=0; i<3; i++)
        {
            accSum[i]=0;
            gyroSum[i]=0;
            cnt=0;
            imu.ready=0;
        }

    }

    if(dt>=10)		//10ms
    {
        if(cnt<300)
        {
            for(i=0; i<3; i++)
            {
                accSum[i]+=imu.accRaw[i];
                gyroSum[i]+=imu.gyroRaw[i];
            }
            cnt++;
            tPrev=now;
        }
        else
        {
            for(i=0; i<3; i++)
            {
                imu.accOffset[i]=accSum[i]/(float)cnt;
                imu.gyroOffset[i]=gyroSum[i]/(float)cnt;
            }

            imu.accOffset[2]=imu.accOffset[2] - CONSTANTS_ONE_G;

            calibrating=0;
#ifndef IMU_SW
            imu.ready=1;
#endif
            ret=1;
            //tobe added: write to eeprom !!
        }
    }

    return ret;
}


#define SENSOR_MAX_G 8.0f		//constant g		// tobe fixed to 8g. but IMU need to  correct at the same time
#define SENSOR_MAX_W 2000.0f	//deg/s
#define ACC_SCALE  (SENSOR_MAX_G/32768.0f)
#define GYRO_SCALE  (SENSOR_MAX_W/32768.0f)
void ReadIMUSensorHandle(void)
{
    uint8_t i;

    //read raw
    MPU6050AccRead(imu.accADC);
    MPU6050GyroRead(imu.gyroADC);
    //tutn to physical
    for(i=0; i<3; i++)
    {
        imu.accRaw[i]= (float)imu.accADC[i] *ACC_SCALE * CONSTANTS_ONE_G ;
        imu.gyroRaw[i]=(float)imu.gyroADC[i] * GYRO_SCALE * M_PI_F /180.f;		//deg/s
    }

    imu.accb[0]=LPF2pApply_1(imu.accRaw[0]-imu.accOffset[0]);
    imu.accb[1]=LPF2pApply_2(imu.accRaw[1]-imu.accOffset[1]);
    imu.accb[2]=LPF2pApply_3(imu.accRaw[2]-imu.accOffset[2]);
#ifdef IMU_SW
    imu.gyro[0]=LPF2pApply_4(imu.gyroRaw[0]);
    imu.gyro[1]=LPF2pApply_5(imu.gyroRaw[1]);
    imu.gyro[2]=LPF2pApply_6(imu.gyroRaw[2]);
#else
    imu.gyro[0]=LPF2pApply_4(imu.gyroRaw[0]-imu.gyroOffset[0]);
    imu.gyro[1]=LPF2pApply_5(imu.gyroRaw[1]-imu.gyroOffset[1]);
    imu.gyro[2]=LPF2pApply_6(imu.gyroRaw[2]-imu.gyroOffset[2]);
#endif

}

//检测IMU是否ready，校准好
//需要将四轴放水平
#define ACCZ_ERR_MAX  0.05		//m/s^2
#define CHECK_TIME 5
uint8_t IMUCheck(void)
{
    uint32_t accZSum=0;
    uint8_t i;
    float accZb=0;

    for(i=0; i<CHECK_TIME; i++)
    {
        MPU6050AccRead(imu.accADC);
        accZSum += imu.accADC[2];
    }
    imu.accRaw[2]= (float)(accZSum /(float)CHECK_TIME) *ACC_SCALE * CONSTANTS_ONE_G ;
    accZb=imu.accRaw[2]-imu.accOffset[2];

    if((accZb > CONSTANTS_ONE_G-ACCZ_ERR_MAX ) && (accZb < CONSTANTS_ONE_G + ACCZ_ERR_MAX))
        imu.caliPass=1;
    else
        imu.caliPass=0;

    return imu.caliPass;

}

/*
in standard sequence , roll-pitch-yaw , x-y-z
angle in rad
get DCM for ground to body

*/

static void eular2DCM(float DCM[3][3],float roll,float pitch,float yaw)
{
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(roll * M_PI_F/180.0f);
    sinx = sinf(roll * M_PI_F/180.0f);
    cosy = cosf(pitch * M_PI_F/180.0f);
    siny = sinf(pitch * M_PI_F/180.0f);
    cosz = cosf(yaw * M_PI_F/180.0f);
    sinz = sinf(yaw * M_PI_F/180.0f);

    coszcosx = cosz * cosx;
    coszcosy = cosz * cosy;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    DCM[0][0] = coszcosy;
    DCM[0][1] = cosy * sinz;
    DCM[0][2] = -siny;
    DCM[1][0] = -sinzcosx + (coszsinx * siny);
    DCM[1][1] = coszcosx + (sinzsinx * siny);
    DCM[1][2] = sinx * cosy;
    DCM[2][0] = (sinzsinx) + (coszcosx * siny);
    DCM[2][1] = -(coszsinx) + (sinzcosx * siny);
    DCM[2][2] = cosy * cosx;

}

