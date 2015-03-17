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
Author:		祥 、小马
------------------------------------
*/
#include "config.h"
#include "DMP.h"
#include "imu.h"
#include "filter.h"
#include "SysConfig.h"


imu_t imu={0};
uint8_t imuCaliFlag=0;
// ---
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
//----base on dmp
void IMU_Process(void)
{
	uint8_t i,j;
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
	
		imu.gyroRaw[0]=DMP_DATA.dmp_gyrox ;		//
		imu.gyroRaw[1]=DMP_DATA.dmp_gyroy ;
		imu.gyroRaw[2]=DMP_DATA.dmp_gyroz ;
		//offset 
//		for(i=0;i<3;i++)
//		{
//				imu.accRaw[i]=imu.accRaw[i]-imu.accOffset[i];
//				imu.gyroRaw[i]=imu.gyroRaw[i]-imu.gyroOffset[i];
//		}
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
		eular2DCM(imu.DCMgb,imu.roll, imu.pitch,imu.yaw);
	
}


//should place to a level surface and keep it stop for 1~2 second
//return 1 when finish
uint8_t IMU_Calibrate(void)
{
	//3s 
	static float accSum[3]={0,0,0};
	static float gyroSum[3]={0,0,0};
	static uint16_t cnt=0;
	static uint16_t tPrev=0,startTime=0;
	static uint8_t calibrating=0;
	uint8_t ret=0;
	uint8_t i=0;
	uint16_t dt=0,now=0,caliTime=0;;


	now=millis();


//#ifndef IMU_SW
#if (1)
		dt=now-tPrev;	

	if(calibrating==0)
	{
			calibrating=1;
			for(i=0;i<3;i++)
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
				for(i=0;i<3;i++)
				{
					accSum[i]+=imu.accRaw[i];		
					gyroSum[i]+=imu.gyroRaw[i];
				}
				cnt++;
				tPrev=now;
			}
			else
			{
					for(i=0;i<3;i++)
					{
						imu.accOffset[i]=accSum[i]/(float)cnt;
						imu.gyroOffset[i]=gyroSum[i]/(float)cnt;
		//				accSum[i]=0;
		//				gyroSum[i]=0;
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
#else
	
	
	if(startTime==0)
	{
		startTime=now;
		imu.ready=0;
	}

	if(now>startTime)
		caliTime=now-startTime;
	else
		caliTime=65536-startTime + now;
	

		for(i=0;i<3;i++)
		{
			accSum[i]+=imu.accRaw[i];			//tobe 
	//		gyroSum[i]+=imu.gyroRaw[i];
		}
		cnt++;
		if(caliTime > ACC_CALC_TIME)
		{
				for(i=0;i<3;i++)
				{
					imu.accOffset[i]=accSum[i]/cnt;
	//				imu.gyroOffset[i]=gyroSum[i]/(float)cnt;
					accSum[i]=0;
				} 
				imu.accOffset[2]=imu.accOffset[2] - CONSTANTS_ONE_G;
				
				startTime=0;
				cnt=0;
				ret=1;
		}
	
#endif
	
	return ret;
	
}


//
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
		for(i=0;i<3;i++)
		{
				imu.accRaw[i]= (float)imu.accADC[i] *ACC_SCALE * CONSTANTS_ONE_G ;
				imu.gyroRaw[i]=(float)imu.gyroADC[i] * GYRO_SCALE * M_PI_F /180.f;		//deg/s
		}
		//cut offset 
//		for(i=0;i<3;i++)
//		{
//				imu.accRaw[i]-=imu.accOffset[i];
//				imu.gyroRaw[i]-=imu.gyroOffset[i];
//		}
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
		//low pass filter.  inertial or digital . tobe tested
//		for(i=0;i<2;i++)	//tobe fixed to digital filter
//		{
//				accFilted[i] = accFilted[i] * (1.0f - (1.0f / ACC_XY_LPF_FACTOR)) + accRaw[i] * (1.0f /ACC_XY_LPF_FACTOR); 
//				
//				gyroFilted[i] = gyroFilted[i] * (1.0f - (1.0f / GYRO_XY_LPF_FACTOR)) + gyroRaw[i] * (1.0f/GYRO_XY_LPF_FACTOR);
//		}
//		accFilted[2]=LPF2pApply_3(accRaw[2]);			// need to set cutoff freq and sample rate before 
//		gyroFilted[2] = gyroFilted[2] * (1.0f - (1.0f / GYRO_Z_LPF_FACTOR)) + gyroRaw[2] * (1.0f/GYRO_Z_LPF_FACTOR);
		
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
	  
	for(i=0;i<CHECK_TIME;i++)
  {
		MPU6050AccRead(imu.accADC);
		accZSum += imu.accADC[2];
	} 
	imu.accRaw[2]= (float)(accZSum /(float)CHECK_TIME) *ACC_SCALE * CONSTANTS_ONE_G ;
	accZb=imu.accRaw[2]-imu.accOffset[2];	
	
	if((accZb > CONSTANTS_ONE_G-ACCZ_ERR_MAX ) && (accZb < CONSTANTS_ONE_G + ACCZ_ERR_MAX))
	//	return 1;
		imu.caliPass=1;
	else
		imu.caliPass=0;
		//return 0;
		
	
	return imu.caliPass;
		
}


#ifdef USE_SW_IMU
//
//int16_t accADC[3];
//int16_t gyroADC[3];
int16_t accADCOffset[3]={0,0,0},gyroADCOffset[3]={-10,17,0};
float accRaw[3];
float gyroRaw[3];		//deg/s
volatile float accFilted[3]={0,0,0};	//m/s2,in body frame, filtered
volatile float gyroFilted[3]={0};
float DCMgb[3][3]={0};
float accZoffsetTemp=0.6f ;	//m/s2
//static float q[4];

//imu
volatile float qa0=1, qa1=0, qa2=0, qa3=0;	//初始化值
float IMU_Roll=0,IMU_Pitch=0,IMU_Yaw=0;
volatile double halftime ;
volatile uint32_t lastUpdate, now; // 采样周期计数 单位 us
volatile float integralFBx=0,integralFBy=0,integralFBz=0;


//-----Get raw acc gyro data from mpu6050 -----//(in600hz)
//read acc gyro，filt and process, 600Hz

#define ACC_XY_LPF_FACTOR  4.0f	//tobe fixed to digital filter
#define GYRO_XY_LPF_FACTOR 20.0f	//bigger ,filter more
#define GYRO_Z_LPF_FACTOR  3.0f	
 

//---------------------imu calculating------------------------//
/**************************实现函数********************************************
功　　能:  快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
static float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//correct error
#define twoKpDef  (1.0f ) // 2 * proportional gain 500Hz 1.0
#define twoKiDef  (0.25f) // 2 * integral gain

void FreeIMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az) 
{
	float norm;
	//  float hx, hy, hz, bx, bz;
	float vx, vy, vz;
	float ex, ey, ez;
	float temp0,temp1,temp2,temp3;

	// 先把这些用得到的值算好
	float q0q0 = qa0*qa0;
	float q0q1 = qa0*qa1;
	float q0q2 = qa0*qa2;
	float q0q3 = qa0*qa3;
	float q1q1 = qa1*qa1;
	float q1q2 = qa1*qa2;
	float q1q3 = qa1*qa3;
	float q2q2 = qa2*qa2;   
	float q2q3 = qa2*qa3;
	float q3q3 = qa3*qa3;          

	//------------ 读取解算时间 ---------------
	now = micros();  	  //读取时间
	if(now < lastUpdate)  //定时器溢出过了。
	{ 
		halftime =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);	
	}
	else
	{
		halftime =  ((float)(now - lastUpdate) / 2000000.0f);
	}
	lastUpdate = now;	 //更新时间
	
	
	norm = invSqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;   

	// estimated direction of gravity and flux (v and w)
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3; 

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy) ;
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;

	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		integralFBx +=  ex * twoKiDef * halftime;
		integralFBy +=  ey * twoKiDef * halftime;	
		integralFBz +=  ez * twoKiDef * halftime;

		gx = gx + twoKpDef*ex + integralFBx;
		gy = gy + twoKpDef*ey + integralFBy;
		gz = gz + twoKpDef*ez + integralFBz;

	}
	// integrate quaternion rate and normalise
	temp0 = qa0 + (double)(-qa1*gx - qa2*gy - qa3*gz)*halftime;
	temp1 = qa1 + (double)(qa0*gx + qa2*gz - qa3*gy)*halftime;
	temp2 = qa2 + (double)(qa0*gy - qa1*gz + qa3*gx)*halftime;
	temp3 = qa3 + (double)(qa0*gz + qa1*gy - qa2*gx)*halftime;  

	// normalise quaternion
	norm = invSqrt(temp0*temp0 + temp1*temp1 + temp2*temp2 + temp3*temp3);
	qa0 = temp0 * norm;
	qa1 = temp1 * norm;
	qa2 = temp2 * norm;
	qa3 = temp3 * norm;
	
}
void IMU_getQ(float * q)
{
	ReadIMUSensorHandle();
	FreeIMU_AHRSupdate(gyroFilted[0]* M_PI_F / 180.0f,gyroFilted[1]* M_PI_F / 180.0f,gyroFilted[2]* M_PI_F / 180.0f,accFilted[0],accFilted[1],accFilted[2]);
	q[0]=qa0;
	q[1]=qa1;
	q[2]=qa2;
	q[3]=qa3;
	
}
//******************************************************************************
// 扩展的反正弦函数：检查输入值的范围（-1~1）、若输入非数字（isnan函数判断）则返回0
//******************************************************************************
float safe_asin(float v)
{
	if (isnan(v)) return 0.0f;

	if (v >= 1.0f) return M_PI_F/2;

	if (v <= -1.0f)return -M_PI_F/2;
	
	return asin(v);
}

void IMU_getYawPitchRoll(void) 
{
  float qn[4]; //四元数
	
	IMU_getQ(qn); //更新全局四元数

	//注意：在陀螺仪规定的坐标系中（x向前，y向左，z向上），IMU_Roll、IMU_Pitch、IMU_Yaw的反三角函数前均无负号
	//该坐标系与航空坐标系（x向前，y向右，z向下）是不同的！
	IMU_Roll= atan2(2.0f*(qn[0]*qn[1] + qn[2]*qn[3]),1 - 2.0f*(qn[1]*qn[1] + qn[2]*qn[2]))* 180/M_PI_F;
//	eularRad[0]=Q_ANGLE.Roll * M_PI_F /180.0f;
	
	// 使用扩展的反正弦函数避免90/-90度附近的奇异值
	IMU_Pitch= safe_asin(2.0f*(qn[0]*qn[2] - qn[3]*qn[1]))* 180/M_PI_F;
//	eularRad[1]=Q_ANGLE.Pitch * M_PI_F /180.0;

  IMU_Yaw= atan2(2 * qn[0] * qn[2] + 2 * qn[0] * qn[3], -2 * qn[2]*qn[2] - 2 * qn[3] * qn[3] + 1)* 180/M_PI_F; // yaw
	//  Q_ANGLE.Yaw =  DMP_DATA.dmp_yaw;  
}

#endif

 

//----Run software imu or use dmp, to get quat DCM euler-----------//
// get DCM from body to ground
void quat2DCM(float DCM[3][3],float q[4])
{
	float q0=q[0],q1=q[1],q2=q[2],q3=q[3];
	float q12=q1*q1, q22=q2*q2, q32=q3*q3;
	
	DCM[0][0]=1.0f -2.0f * (q22   + q32);
	DCM[1][0]=			2.0f * (q1*q2 + q0*q3);
	DCM[2][0]=			2.0f * (q1*q3 - q0*q2);
	
	DCM[0][1]= 			2.0f * (q1*q2 - q0*q3);
	DCM[1][1]=1.0f -2.0f * (q12   +  q32);
	DCM[2][1]=			2.0f * (q2*q3 + q0*q1);
	
	DCM[0][2]=			2.0f * (q1*q3 + q0*q2);
	DCM[1][2]=			2.0f * (q2*q3 - q0*q1);
	DCM[2][2]=1.0f -2.0f * (q12 + q22);
}
/*
in standard sequence , roll-pitch-yaw , x-y-z
angle in rad
get DCM for ground to body

*/

void eular2DCM(float DCM[3][3],float roll,float pitch,float yaw) 
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
// from body to ground
void RotFromEuler(float R[3][3],float roll, float pitch, float yaw) 
{
		float cp = cosf(pitch);
		float sp = sinf(pitch);
		float sr = sinf(roll);
		float cr = cosf(roll);
		float sy = sinf(yaw);
		float cy = cosf(yaw);

		R[0][0] = cp * cy;
		R[0][1] = (sr * sp * cy) - (cr * sy);
		R[0][2] = (cr * sp * cy) + (sr * sy);
		R[1][0] = cp * sy;
		R[1][1] = (sr * sp * sy) + (cr * cy);
		R[1][2] = (cr * sp * sy) - (sr * cy);
		R[2][0] = -sp;
		R[2][1] = sr * cp;
		R[2][2] = cr * cp;
}
//
void QuadFromDcm(quad q, float dcm[3][3]) 
{
		// avoiding singularities by not using division equations
		q[0] = 0.5f * sqrtf(1.0f + dcm[0][0] + dcm[1][1] + dcm[2][2]);
		q[1] = 0.5f * sqrtf(1.0f + dcm[0][0] - dcm[1][1] - dcm[2][2]);
		q[2] = 0.5f * sqrtf(1.0f - dcm[0][0] + dcm[1][1] - dcm[2][2]);
		q[3] = 0.5f * sqrtf(1.0f - dcm[0][0] - dcm[1][1] + dcm[2][2]);
}
//
void Mat3Transpose(float mDes[3][3],float mSrc[3][3])
{
	uint8_t i,j;
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			mDes[i][j]=mSrc[j][i];
}


void mt3m(float ret[3],float m[3][3],float d[3])
{
		uint8_t i,j;
		for(i=0;i<3;i++)
		{
			ret[i]=0;
			for(j=0;j<3;j++)
				ret[i]+=m[i][j]*d[j];
				
		}
}
//
mat3 mt3mt3(float m1[3][3],float m2[3][3])
{
	mat3 ret;
	uint8_t i,j,k;
	
	for(i=0;i<3;i++)		//行
		for(j=0;j<3;j++)	//列
		{
				ret.m[i][j]=0;
				for( k=0;k<3;k++)
					ret.m[i][j] += m1[i][k] * m2[k][j];
		}
		
	return ret;
}
//
float vt3v(float v1[3],float v2[3])
{
	uint8_t i;
	float ret=0;
	
	  for(i=0;i<3;i++)
			ret+=v1[i]*v2[i];
	
	return ret;
}
//
vec3 Vector3Mod(float dataA[3],float dataB[3])
{
	vec3 tempA;
	
	tempA.v[0]=dataA[1] * dataB[2] - dataA[2] * dataB[1];
	tempA.v[1]=dataA[2] * dataB[0] - dataA[0] * dataB[2];
	tempA.v[2]=dataA[0] * dataB[1] - dataA[1] * dataB[0];
	
	return  tempA;
		       
}
//-------------Get acc in earth frame---------------//



