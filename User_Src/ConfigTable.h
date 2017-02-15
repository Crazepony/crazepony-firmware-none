#ifndef __CONFIG_TABLE_H
#define __CONFIG_TABLE_H
//
#include "stm32f10x.h"

typedef struct config_table_tt
{
    float version;
    float pidPitch[3];
    float pidPitchRate[3];
    float pidRoll[3];
    float pidRollRate[3];
    float pidYaw[3];
    float pidYawRate[3];
    float pidAlt[3];
    float pidAltVel[3];
    float accOffset[3];
    float gyroOffset[3];
    float magOffset[3];
    float NRFaddr[5];
    float BTstate;    //蓝牙是否需要重新写参数状态
    float NRFmatchFlag;

} config_table_t;

extern config_table_t table;
extern uint8_t gParamsSaveEEPROMRequset;

void LoadParamsFromEEPROM(void);
void ParamSetDefault(void) ;
void ParamToTable(void);
void TableToParam(void);
void TableWriteEEPROM(void);
void TableReadEEPROM(void);
void SaveParamsToEEPROM(void);

#endif
