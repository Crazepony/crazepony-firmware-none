/**
* EEPROM Table by samit
**/

#include "ConfigTable.h"
#include "stmflash.h"
#include "Control.h"
#include "imu.h"
#include "SysConfig.h"
#include "BT.h"
#include "NRF24L01.h"
//
#define TABLE_ADDRESS (STM32_FLASH_BASE+STM32_FLASH_OFFEST+0)
//用来存放EEPROM列表上的存放的参数变量的信息
config_table_t table;				//tobe improved: config mean in const / eeprom.
//请求保存参数到EEPROM的信号量
uint8_t gParamsSaveEEPROMRequset=0;

#define EEPROM_DEFAULT_VERSION 1

static uint8_t  isEEPROMValid(void)
{
    STMFLASH_Read(TABLE_ADDRESS,(uint16_t *)(&table),2);
    if((int16_t)table.version==EEPROM_DEFAULT_VERSION)
        return 1;
    else
        return 0;
}

//table defalat . if
void TableResetDefault(void)
{
    STMFLASH_Write(TABLE_ADDRESS,(uint16_t *)(&(table.version)),2);
}

//load params for EEPROM
void TableReadEEPROM(void)
{
    uint8_t paramNums=sizeof(table)/sizeof(float);

    STMFLASH_Read(TABLE_ADDRESS,(uint16_t *)(&table),paramNums * 2);
}

void TableWriteEEPROM(void)
{
    uint8_t paramNums=sizeof(table)/sizeof(float);

    STMFLASH_Write(TABLE_ADDRESS,(uint16_t *)(&table),paramNums * 2);
}


extern u8 RX_ADDRESS[RX_ADR_WIDTH];


extern u8 NRFMatched;


void TableToParam(void)
{
    uint8_t i=0;
    for(i=0; i<3; i++)
    {
        ((float *)(&pitch_angle_PID))[i]=((float *)(&table.pidPitch))[i];
        ((float *)(&roll_angle_PID))[i]=((float *)(&table.pidRoll))[i];
        ((float *)(&yaw_angle_PID))[i]=((float *)(&table.pidYaw))[i];

        ((float *)(&pitch_rate_PID))[i]=((float *)(&table.pidPitchRate))[i];
        ((float *)(&roll_rate_PID))[i]=((float *)(&table.pidRollRate))[i];
        ((float *)(&yaw_rate_PID))[i]=((float *)(&table.pidYawRate))[i];

        ((float *)(&alt_PID))[i]=((float *)(&table.pidAlt))[i];
        ((float *)(&alt_vel_PID))[i]=((float *)(&table.pidAltVel))[i];

        imu.accOffset[i]=table.accOffset[i];
        imu.gyroOffset[i]=table.gyroOffset[i];



#ifdef NEW_ATTI_CTRL
        AttiCtrlParamsFromPIDTable();	//load to new ctrl param
#endif

    }

    for(i=0; i<5; i++) {
        ((u8 *)(&RX_ADDRESS))[i] = ((float *)(&table.NRFaddr))[i];

        printf("RX_ADDRESS[%d]:0x%x\r\n",i,RX_ADDRESS[i]);
    }

    BTstate = table.BTstate;
    NRFMatched = table.NRFmatchFlag;



}


void ParamToTable(void)
{
    uint8_t i=0;
    float temp;
    for(i=0; i<3; i++)
    {
        ((float *)(&table.pidPitch))[i]=((float *)(&pitch_angle_PID))[i];
        temp=((float *)(&roll_angle_PID))[i];
        *((float *)(&table.pidRoll) + i) =  ((float *)(&roll_angle_PID))[i];
        ((float *)(&table.pidRoll))[i]=((float *)(&roll_angle_PID))[i];
        ((float *)(&table.pidYaw))[i]=((float *)(&yaw_angle_PID))[i];

        ((float *)(&table.pidPitchRate))[i]=((float *)(&pitch_rate_PID))[i];
        ((float *)(&table.pidRollRate))[i]=((float *)(&roll_rate_PID))[i];
        ((float *)(&table.pidYawRate))[i]=((float *)(&yaw_rate_PID))[i];

        ((float *)(&table.pidAlt))[i]=((float *)(&alt_PID))[i];
        ((float *)(&table.pidAltVel))[i]=((float *)(&alt_vel_PID))[i];


        table.accOffset[i]=imu.accOffset[i];
        table.gyroOffset[i]=imu.gyroOffset[i];
    }

    for(i=0; i<5; i++)
        ((float *)(&table.NRFaddr))[i] = ((u8 *)(&RX_ADDRESS))[i];


    table.BTstate = BTstate;
    table.NRFmatchFlag = NRFMatched;


}

void LoadParamsFromEEPROM(void)
{
    if(isEEPROMValid())
    {
        TableReadEEPROM();
        TableToParam();

    }
    else
    {
        printf("load params from eeprom failed,set default value\r\n");

        ParamSetDefault();//版本检测不对，各项参数设为默认值
        ParamToTable();
        table.version=EEPROM_DEFAULT_VERSION;
        TableWriteEEPROM();
    }
}

void SaveParamsToEEPROM(void)
{
    ParamToTable();
    TableWriteEEPROM();
}

//all default value
void ParamSetDefault(void)
{

#ifndef NEW_ATTI_CTRL
    pitch_angle_PID.P = 3.5;
    pitch_angle_PID.I = 0;//1.0;		//0
    pitch_angle_PID.D = 0;

    pitch_angle_PID.iLimit = 300;	//or 1000

    pitch_rate_PID.P  = 0.7;
    pitch_rate_PID.I  = 0.5; 		//0.5
    pitch_rate_PID.D  = 0.03;

    pitch_rate_PID.iLimit = 300;
////////////////////////////////////////////
    roll_angle_PID.P = 3.5;
    roll_angle_PID.I = 0;//1.0;
    roll_angle_PID.D = 0;
    roll_angle_PID.iLimit = 300;	//or 1000

    roll_rate_PID.P  = 0.7;
    roll_rate_PID.I  = 0.5;; 	//0.5
    roll_rate_PID.D  = 0.03;
    roll_rate_PID.iLimit = 300;
///////////////////////////////////////////
    yaw_angle_PID.P = 1;
    yaw_angle_PID.I = 0.2;
    yaw_angle_PID.D = 0;

    yaw_rate_PID.P  = 20;
    yaw_rate_PID.I  = 0;
    yaw_rate_PID.D  = 0;
#else
    AttiCtrlParamsSetDefault();
#endif
//
    alt_PID.P=1.0;
    alt_PID.I=0;
    alt_PID.D=0;

    alt_vel_PID.P=0.1f;
    alt_vel_PID.I=0.02f;
    alt_vel_PID.D=0;

    //should chango to read eeprom cfg. should be 0.
    imu.accOffset[0]=-0.1620515;
    imu.accOffset[1]=0.07422026;
    imu.accOffset[2]=0.7743073;

    imu.gyroOffset[0]=-0.06097556;
    imu.gyroOffset[1]=-0.03780485;
    imu.gyroOffset[2]=0;


//		AttiCtrlParamsFromPIDTable();

}
