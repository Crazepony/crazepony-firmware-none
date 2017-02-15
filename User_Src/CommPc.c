#include "config.h"
#include "SysConfig.h"
#include "CommPC.h"
#include "imu.h"
#include "Altitude.h"



#define CONV_ENDIAN

uint8_t pcCmdFlag=0;
//static uint8_t uploadBuf[32]={0xAA,0xAA};
#define PC_REQ_PID    0x02
#define PC_PID_PITCH	0x10
#define PC_PID_ROLL		0x11
#define PC_PID_YAW		0x12
#define PC_PID_ALT		0x14

typedef struct DataPackage_tt
{
    uint8_t header[2];
    uint8_t cmd;
    uint8_t len;
    uint8_t data[30];
    uint8_t sum;
} DataPackage_t;


//注意地址对齐问题！！
HawkerPacket_t up= {{0xAA,0xAA},0x01,18};	//upload packet
DataPackage_t up2= {{0xAA,0xAA},0x02,30,{0}};

static void EndianConvert(uint8_t arr[], uint8_t len);



//分包发送，分散cpu占用时间
static uint8_t sendPCBuf[64]= {0xAA,0xAA,0x01,0x14,0,100,0,200,0,130,0,0,0,100,0,0,0,200,0,0,0,30,0,10,0x6B};


//------------------New Send --------------------//
#define TEST_LEN 1+5
//uint8_t testData[0x0C+5]={0xAA,0xAF,0x10,0x0C,0,1,0,0,0,0,0,0,0,0,0,0,(uint8_t)(0xAA+0xAF+0x10+0x0C+0x01)};
uint8_t testData[1+5]= {0xAA,0xAF,0x02,0x01,0x01,(uint8_t)(0xAA+0xAF+0x02+1+1)};
static uint8_t sendCnt=0;
static uint8_t checksum;


extern uint8_t gParamsSaveEEPROMRequset;

void testCommPC(void)
{
    uint8_t i=0;
    for(i=0; i<TEST_LEN; i++)
        CommPC(testData[i]);
}



static void BufAdd8Chk(uint8_t a)
{
    //UartBuf_WD(&UartTxbuf,_x);
    sendPCBuf[sendCnt++]=a;
    checksum += a;
}
static void BufAddInt16(int16_t a)
{
    BufAdd8Chk((uint8_t)(a>>8));
    BufAdd8Chk((uint8_t)(a&0xff));
}
static void BufAddArr(uint8_t *dat,uint8_t len)
{
    uint8_t i;
    for(i=0; i<len; i++)
        BufAdd8Chk(dat[i]);
}
static void BufUpload(void)
{
    UartSendBuffer(sendPCBuf,sendCnt);
    sendCnt=0;
    checksum=0;
}

//根据不同命令字上传
void CommPCUpload(uint8_t cmd)
{
    //	UartSendBuffer(testData,6);

//		sendPCBuf[0]=0xAA;
//		sendPCBuf[1]=0xAA;
//		sendPCBuf[2]=cmd;
    checksum=0;
//		UartBufClear(&UartTxbuf);
    BufAdd8Chk(0xAA);
    BufAdd8Chk(0xAA);
    BufAdd8Chk(cmd);
    switch(cmd)
    {
    case PC_PID_PITCH:
        BufAdd8Chk(0x0C);	//len
        BufAddInt16((int16_t)((pitch_rate_PID.P * 100)));
        BufAddInt16((int16_t)((pitch_rate_PID.I * 100)));
        BufAddInt16((int16_t)((pitch_rate_PID.D * 100)));

        BufAddInt16((int16_t)((pitch_angle_PID.P * 100)));
        BufAddInt16((int16_t)((pitch_angle_PID.I * 100)));
        BufAddInt16((int16_t)((pitch_angle_PID.D * 100)));
        break;
    case PC_PID_ROLL:
        BufAdd8Chk(0x0C);	//len
        BufAddInt16((int16_t)((roll_rate_PID.P * 100)));
        BufAddInt16((int16_t)((roll_rate_PID.I * 100)));
        BufAddInt16((int16_t)((roll_rate_PID.D * 100)));

        BufAddInt16((int16_t)((roll_angle_PID.P * 100)));
        BufAddInt16((int16_t)((roll_angle_PID.I * 100)));
        BufAddInt16((int16_t)((roll_angle_PID.D * 100)));
        break;
    case PC_PID_YAW:
        BufAdd8Chk(0x0C);	//len
        BufAddInt16((int16_t)((yaw_rate_PID.P * 100)));
        BufAddInt16((int16_t)((yaw_rate_PID.I * 100)));
        BufAddInt16((int16_t)((yaw_rate_PID.D * 100)));

        BufAddInt16((int16_t)((yaw_angle_PID.P * 100)));
        BufAddInt16((int16_t)((yaw_angle_PID.I * 100)));
        BufAddInt16((int16_t)((yaw_angle_PID.D * 100)));
        break;
    case PC_PID_ALT:
        BufAdd8Chk(0x0C);	//len
        BufAddInt16((int16_t)((alt_vel_PID.P * 100)));
        BufAddInt16((int16_t)((alt_vel_PID.I * 100)));
        BufAddInt16((int16_t)((alt_vel_PID.D * 100)));

        BufAddInt16((int16_t)((alt_PID.P * 100)));
        BufAddInt16((int16_t)((alt_PID.I * 100)));
        BufAddInt16((int16_t)((alt_PID.D * 100)));
        break;
    }
    BufAdd8Chk(checksum);
    BufUpload();

}

//接收


static uint8_t cmd=0,len=0,chkSum=0;
#define DAT_MAX_LEN 32
static uint8_t datBuf[DAT_MAX_LEN]= {0};
static uint8_t  datCnt=0;

enum
{
    IDLE=0,
    HEADER1,
    HEADER2,
    CMD,
    LEN,
    DATA,
    CHK
};
static uint8_t ps=IDLE;
void CommPC(uint8_t c)
{


    switch(ps)
    {
    case IDLE:
        chkSum=0;
        if(c==0xAA)
            ps=HEADER1;
        break;
    case HEADER1:
        if(c==0xAF)
            ps=HEADER2;
        else
            ps=IDLE;
        break;
    case HEADER2:
        cmd=c;
        ps=CMD;
        break;
    case CMD:
        len=c;
        ps=DATA;
        chkSum=0xAA + 0xAF + len +cmd;
        break;
    case DATA:

        if(datCnt<len)
        {
            datBuf[datCnt++]=c;
            chkSum+=c;
        }
        if(datCnt==len)
            ps=CHK;
        break;
    case CHK:
        if(chkSum==c)
        {
            pcCmdFlag=1;		//specific cmd process executed in main, not in irq
            //		CommPCProcessCmd();//process cmd
            datCnt=0;
            btSrc=SRC_PC;
        }
        ps=IDLE;
        break;
    }

}

//处理PC发过来的命令
void CommPCProcessCmd(void)
{
    //UartBufClear(&UartTxbuf);	//以备发送
    switch(cmd)
    {
    case PC_REQ_PID:
        if(datBuf[0]==0x01)	//read PID
        {
            CommPCUpload(PC_PID_PITCH);
            CommPCUpload(PC_PID_ROLL);
            CommPCUpload(PC_PID_YAW);
            CommPCUpload(PC_PID_ALT);
        }
        break;
    case PC_PID_PITCH:	//pitch sub pid, main pid
        pitch_rate_PID.P=(int16_t)(datBuf[0]<<8 | datBuf[1]) * 0.01f;
        pitch_rate_PID.I=(int16_t)(datBuf[2]<<8 | datBuf[3]) * 0.01f;
        pitch_rate_PID.D=(int16_t)(datBuf[4]<<8 | datBuf[5]) * 0.01f;

        pitch_angle_PID.P=(int16_t)(datBuf[6]<<8 | datBuf[7]) * 0.01f;
        pitch_angle_PID.I=(int16_t)(datBuf[8]<<8 | datBuf[9]) * 0.01f;
        pitch_angle_PID.D=(int16_t)(datBuf[10]<<8 | datBuf[11]) * 0.01f;

        //		CommPCUpload(PC_PID_PITCH);
        //		UartSendBuffer(0xAA,1);
        ReturnPIDHead(PC_PID_PITCH);

        gParamsSaveEEPROMRequset=1;
        break;
    case PC_PID_ROLL:	//roll sub pid, main pid
        roll_rate_PID.P=(int16_t)(datBuf[0]<<8 | datBuf[1]) * 0.01f;
        roll_rate_PID.I=(int16_t)(datBuf[2]<<8 | datBuf[3]) * 0.01f;
        roll_rate_PID.D=(int16_t)(datBuf[4]<<8 | datBuf[5]) * 0.01f;

        roll_angle_PID.P=(int16_t)(datBuf[6]<<8 | datBuf[7]) * 0.01f;
        roll_angle_PID.I=(int16_t)(datBuf[8]<<8 | datBuf[9]) * 0.01f;
        roll_angle_PID.D=(int16_t)(datBuf[10]<<8 | datBuf[11]) * 0.01f;

        //	CommPCUpload(PC_PID_ROLL);
        ReturnPIDHead(PC_PID_ROLL);
        gParamsSaveEEPROMRequset=1;
        break;
    case PC_PID_YAW:	//yaw
        yaw_rate_PID.P=(int16_t)(datBuf[0]<<8 | datBuf[1]) * 0.01f;
        yaw_rate_PID.I=(int16_t)(datBuf[2]<<8 | datBuf[3]) * 0.01f;
        yaw_rate_PID.D=(int16_t)(datBuf[4]<<8 | datBuf[5]) * 0.01f;

        yaw_angle_PID.P=(int16_t)(datBuf[6]<<8 | datBuf[7]) * 0.01f;
        yaw_angle_PID.I=(int16_t)(datBuf[8]<<8 | datBuf[9]) * 0.01f;
        yaw_angle_PID.D=(int16_t)(datBuf[10]<<8 | datBuf[11]) * 0.01f;

        //CommPCUpload(PC_PID_YAW);
        ReturnPIDHead(PC_PID_YAW);
        gParamsSaveEEPROMRequset=1;
        break;
    case 0x13:	//

        break;
    case 0x14: //alt sub pid , main pid
        alt_vel_PID.P=(int16_t)(datBuf[0]<<8 | datBuf[1]) * 0.01f;
        alt_vel_PID.I=(int16_t)(datBuf[2]<<8 | datBuf[3]) * 0.01f;
        alt_vel_PID.D=(int16_t)(datBuf[4]<<8 | datBuf[5]) * 0.01f;

        alt_PID.P=(int16_t)(datBuf[6]<<8 | datBuf[7]) * 0.01f;
        alt_PID.I=(int16_t)(datBuf[8]<<8 | datBuf[9]) * 0.01f;
        alt_PID.D=(int16_t)(datBuf[10]<<8 | datBuf[11]) * 0.01f;

        //CommPCUpload(PC_PID_ALT);
        ReturnPIDHead(PC_PID_ALT);
        gParamsSaveEEPROMRequset=1;
        break;
    }


}

void ReturnPIDHead(uint8_t pidType)
{
    checksum=0;
    sendCnt=0;
    BufAdd8Chk(0xAA);
    BufAdd8Chk(0xAA);
    BufAdd8Chk(pidType);
    BufAdd8Chk(0x0C);

    BufAddArr(datBuf,12);
    BufAdd8Chk(checksum);
    BufUpload();
}

//--- a little zzz
//interface with hawker
void DebugUploadHandle(void)
{
    up.roll.val= imu.roll * 100;
    up.pitch.val= imu.pitch * 100;
    up.yaw.val= imu.yaw * 100;
    up.alti.val=nav.z * 100;		//combined
    up.temp.val=MS5611_Temperature * 100;
    up.pres.val=MS5611_Pressure;
    up.speed.val=nav.vz * 100;

#ifdef CONV_ENDIAN
    EndianConvert(up.roll.b,2);
    EndianConvert(up.pitch.b,2);
    EndianConvert(up.yaw.b,2);
    EndianConvert(up.alti.b,4);
    EndianConvert(up.temp.b,2);
    EndianConvert(up.pres.b,4);
    EndianConvert(up.speed.b,2);
#endif

}
//arm is high in front. convert to fit upper.
static void EndianConvert(uint8_t arr[], uint8_t len)
{
    uint8_t arrS[8],i;
    for(i=0; i<len; i++)
        arrS[i]=arr[i];
    for(i=0; i<len; i++)
        arr[len-1-i]=arrS[i];
}


static void DebugUploadHandle2()
{
    checksum=0;
//		UartBufClear(&UartTxbuf);
    BufAdd8Chk(0xAA);
    BufAdd8Chk(0xAA);
    BufAdd8Chk(0x02);	//cmd
    BufAdd8Chk(30);	//len
    //acc
    BufAddInt16(imu.accb[0] * 1000 );
    BufAddInt16(imu.accb[1] * 1000 );
    BufAddInt16(imu.accb[2] * 1000 );
    //gyro
    BufAddInt16(imu.gyro[0]*180.0f/M_PI_F * 100 );
    BufAddInt16(imu.gyro[1]*180.0f/M_PI_F * 100 );
    BufAddInt16(imu.gyro[2]*180.0f/M_PI_F * 100 );
    //mag
    BufAddInt16(0);
    BufAddInt16(0 );
    BufAddInt16(0 );
    //raw
    BufAddInt16(imu.accRaw[0] * 1000 );
    BufAddInt16(imu.accRaw[1] * 1000 );
    BufAddInt16(imu.accRaw[2] * 1000 );

    BufAddInt16(imu.gyroRaw[0]*180.0f/M_PI_F * 100 );
    BufAddInt16(imu.gyroRaw[1]*180.0f/M_PI_F * 100 );
    BufAddInt16(imu.gyroRaw[2]*180.0f/M_PI_F * 100 );

    BufAdd8Chk(checksum);
    BufUpload();

}


static void DebubUploadHandle3()
{
    uint8_t i;
    up2.cmd=0x08;
    up2.len=6*2;
    up2.data[0]=0;
    up2.data[1]=0;
    up2.data[2]=0;//((short)(MS5611_VerticalSpeed*1000))>>8;	//baro_speed
    up2.data[3]=0;//((short)(MS5611_VerticalSpeed*1000))&0xff;
    up2.data[4]=((short)(-RC_DATA.PITCH*100))>>8;	//acc speed
    up2.data[5]=((short)(-RC_DATA.PITCH*100))&0xff;	//pitch
    up2.data[6]=((short)(MS5611_Altitude*1000))>>8;
    up2.data[7]=((short)(MS5611_Altitude*1000))&0xff;
    up2.data[8]=((short)(imu.accg[2]*1000))>>8;		//accz
    up2.data[9]=((short)(imu.accg[2]*1000))&0xff;;
    up2.data[10]=0;		//inte alt of accz
    up2.data[11]=0;
    up2.sum=0;
    for(i=0; i<4; i++)
        up2.sum+=*((uint8_t *)(&up2)+i);
    for(i=0; i<up2.len; i++)
        up2.sum+=up2.data[i];
    UartSendBuffer((uint8_t *)(&up2),up2.len + 4);
    UartSendBuffer( &(up2.sum),1);
}


void CommPCUploadHandle()
{
    static uint8_t pkgDivCnt=0;

    uint8_t i=0;

    pkgDivCnt++;
    if(pkgDivCnt>2)
    {
        pkgDivCnt=0;
    }

    if(pkgDivCnt == 0)		//div time to send different datapacket to avoid use too much cpu at a time
    {
        DebugUploadHandle();
        for(i=0; i<10; i++)		//solove data in ram address align
            sendPCBuf[i]=*((uint8_t *)(&up) + i);
        for(i=10; i<16; i++)
            sendPCBuf[i]=*((uint8_t *)(&up) + 2+i);
        for(i=16; i<23; i++)
            sendPCBuf[i]=*((uint8_t *)(&up) + 4+i);
        sendPCBuf[22]=0;
        for(i=0; i<22; i++)
            sendPCBuf[22]+=sendPCBuf[i];
        UartSendBuffer(sendPCBuf,23);
    }
    else if(pkgDivCnt == 1)
    {
        DebugUploadHandle2();
    }
    else if(pkgDivCnt==2)
    {
        DebubUploadHandle3();
    }
}

