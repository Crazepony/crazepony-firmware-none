#include "config.h"
#include "SysConfig.h"
#include "CommApp.h"
#include "imu.h"
#include "Altitude.h"
 

uint8_t flyLogF=0;
uint8_t flyLogApp=0;
uint32_t lastGetRCTime;
uint8_t appCmdFlag=0;
uint8_t armState=DISARMED;//,disarmRequest=0;

uint8_t btSrc=0;

 
#define MAX_LEN 32
volatile uint8_t UdataBuf[MAX_LEN];
uint8_t bufP=0;//static 
static uint8_t validDataLen=0;
const uint8_t HEADER[2]={0xAA,0x55};
uint16_t rcData[4]={1500,1500,1500,1500};

#define CONSTRAIN(x,min,max) {if(x<min) x=min; if(x>max) x=max;}
extern float dbScaleLinear(float x, float x_end, float deadband);

// input: rcData , raw data from remote control source
// output: RC_DATA, desired  thro, pitch, roll, yaw
void RCDataProcess(void)
{
	  CONSTRAIN(rcData[THROTTLE],1000,2000);
		CONSTRAIN(rcData[YAW],1000,2000);
		CONSTRAIN(rcData[PITCH],1000,2000);
		CONSTRAIN(rcData[ROLL],1000,2000);
	 
//						 CUT_DB(rcData[YAW],1500,APP_YAW_DB);
//						 CUT_DB(rcData[PITCH],1500,APP_PR_DB);
//						 CUT_DB(rcData[ROLL],1500,APP_PR_DB);
 
	 RC_DATA.THROTTLE=rcData[THROTTLE]-1000;
//						 RC_DATA.YAW=(rcData[YAW]-1500)/500.0 *Angle_Max;
//						 RC_DATA.PITCH=(rcData[PITCH]-1500)/500.0 *Angle_Max; // + Pitch_error_init;
//						 RC_DATA.ROOL=(rcData[ROLL]-1500)/500.0 *Angle_Max;  // + Rool_error_init;
	RC_DATA.YAW= YAW_RATE_MAX * dbScaleLinear((rcData[YAW] - 1500),500,APP_YAW_DB);
	RC_DATA.PITCH= Angle_Max * dbScaleLinear((rcData[PITCH] - 1500),500,APP_PR_DB);
	RC_DATA.ROOL= Angle_Max * dbScaleLinear((rcData[ROLL] - 1500),500,APP_PR_DB);
	
	switch(armState)
	{
		case REQ_ARM:
			
			 
			if(IMUCheck() && !Battery.alarm)
			{	
				armState=ARMED;
				FLY_ENABLE=0xA5;
			}
			else
			{
				FLY_ENABLE=0;
				armState=DISARMED;
			}
		break;
		case REQ_DISARM:
			FLY_ENABLE=0;
			altCtrlMode=MANUAL;		//上锁后加的处理
			 zIntReset=1;		//
			 thrustZSp=0;	
			 thrustZInt=HOVER_THRU;
			 offLandFlag=0;
			
			armState=DISARMED;
		break;
		default:
			break;
			
	}
	
}

//
void CommAppCmdProcess(void)
{
		//process
			 switch(UdataBuf[4])//MSP_SET_4CON
			 {
					 case MSP_SET_4CON:	//UdataBuf
						 rcData[THROTTLE]=UdataBuf[6]<<8 | UdataBuf[5];
						 rcData[YAW]=UdataBuf[8]<<8 | UdataBuf[7];
						 rcData[PITCH]=UdataBuf[10]<<8 | UdataBuf[9];
						 rcData[ROLL]=UdataBuf[12]<<8 | UdataBuf[11];
							
					//	 RCDataProcess();
						
					 break;
					 case MSP_ARM_IT://MSP_ARM_IT
							
					//		FLY_ENABLE=0xA5;
							armState=REQ_ARM;
						 break;
					 case MSP_DISARM_IT:
						  armState=REQ_DISARM;
						 
						 break;
					 case MSP_FLY_STATE:	
						 flyLogApp=1;
						 break;
					 case MSP_ACC_CALI:
						 imuCaliFlag=1;
						 break;
					 case MSP_HEAD_FREE:
						 SetHeadFree(1);
						 break;
					 case MSP_STOP_HEAD_FREE:
						 SetHeadFree(0);
						 break;
					 case MSP_LAND_DOWN:		//自动降落
						 altCtrlMode=LANDING;
						 break;
			 }
}
 

static uint8_t checksum=0; 

//
void CommApp(uint8_t ch)
{
		UdataBuf[bufP]= ch;
		if(bufP<3)
		{
				switch(bufP)
				{
					case 0:
						if(UdataBuf[bufP]=='$')
							bufP++;
						break;
					case 1:
						if(UdataBuf[bufP]=='M')
							bufP++;
						else
							bufP=0;
						break;
					case 2:
						if(UdataBuf[bufP]=='<')
							bufP++;
						else
							bufP=0;
						break;
//					case 3:	//len
//						validDataLen=UdataBuf[bufP];
//						bufP++;
//						break;	
					 
				}
		}
		else	//valid data
		{
			  if(bufP==3)		//len
				{
					checksum=0;
					validDataLen=UdataBuf[bufP];
				}

				bufP++;
			  if(bufP >= validDataLen + 6)	// rec over. process. tobe placed in 50Hz loop
				{
						//chksum
						if(UdataBuf[bufP-1]==checksum)
						{
					 			CommAppCmdProcess();		//could be place to main
							//	appCmdFlag=1;
								btSrc=SRC_APP; 
								lastGetRCTime=millis();		//ms
						}
						 // 
						bufP=0; 
				//  validDataLen=0;
				}
				else
						checksum^=UdataBuf[bufP-1];
		}  
}




static  void uart8chk(uint8_t _x) 
{
	UartBuf_WD(&UartTxbuf,_x); 
	checksum ^= _x; 
}
static  void uart32chk(uint32_t a)
{
    static uint8_t t;
    t = a;
    UartBuf_WD(&UartTxbuf,t);
    checksum ^= t;
    t = a >> 8;
    UartBuf_WD(&UartTxbuf,t);
    checksum ^= t;
    t = a >> 16;
    UartBuf_WD(&UartTxbuf,t);
    checksum ^= t;
    t = a >> 24;
    UartBuf_WD(&UartTxbuf,t);
    checksum ^= t;
}

static void uart16chk(int16_t a)
{
    static uint8_t t;
    t = a;
    UartBuf_WD(&UartTxbuf,t);
    checksum ^= t;
    t = a >> 8 & 0xff;
    UartBuf_WD(&UartTxbuf,t);
    checksum ^= t;
}
//

void AppUpload(uint8_t cmd, uint8_t dataLen, uint8_t *dat)
{

		uint8_t i;
    uart8chk('$');
    uart8chk('M');
 //   uart8chk(err ? '!' : '>');
		uart8chk('>');
  	checksum = 0;               // start calculating a new checksum
    uart8chk(dataLen);	  	//应答帧的包含数据字节数（不包含帧头），可能是要返回的ACC GYRO MAG的数据字节数，可能是GPS。。。。
    uart8chk(cmd);	 //	当前执行的命令
/*	
	for (i = 0; i < 2; i++)
      serialize16(angle[i]);
    serialize16(heading);
		serialize32(EstAlt);
		uart8chk(f.GPS_FIX);
    uart8chk(GPS_numSat);
	*/
	  for(i=0;i<dataLen;i++)
			uart8chk(*dat++);
	  uart8chk(checksum);
	  checksum=0;
}

void CommAppUpload(void)
{
	//	uint8_t i;
    uart8chk('$');
    uart8chk('M');
 //   uart8chk(err ? '!' : '>');
		uart8chk('>');
  	checksum = 0;   
		uart8chk(12+2); 	  		
		uart8chk(MSP_FLY_STATE);	
	//	for (i = 0; i < 2; i++)
	//			uart16chk(angle[i]);
		uart16chk((int16_t)(imu.roll * 10));
		uart16chk((int16_t)(imu.pitch * 10));
		uart16chk((int16_t)(imu.yaw * 10) ); 
		uart32chk((int32_t)(-nav.z * 100));	//alt
		uart16chk((int16_t)(Battery.BatteryVal * 100));//bat
		uart16chk((int16_t)(-nav.vz * 1000));
		
		uart8chk(checksum);
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}