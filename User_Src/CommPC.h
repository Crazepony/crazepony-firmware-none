#ifndef _COMM_PC_H
#define _COMM_PC_H

#include "UART1.h"

//only send data
typedef union int16un
{
    uint8_t b[2];
    int16_t val;
} int16_un;
typedef union int32un
{
    uint8_t b[4];
    int32_t val;
} int32_un;

typedef struct HawkerPacket_tt
{
    uint8_t header[2];
    uint8_t cmd;
    uint8_t len;

    int16_un roll;
    int16_un pitch;
    int16_un yaw;
    int32_un alti;
    int16_un temp;
    int32_un pres;
    int16_un speed;

    uint8_t sum;
} HawkerPacket_t;


//
extern HawkerPacket_t up;
extern uint8_t pcCmdFlag;
extern uint16_t rcData[4];

void CommPCUploadHandle(void);
void CommPCProcessCmd(void);
void CommPC(uint8_t c);
void testCommPC(void);
void ReturnPIDHead(uint8_t pidType);
//
#define UartSendInt16(_x)  UartSendBuffer((uint8_t *)(&_x ),2)
#define UartSendInt32(_x)  UartSendBuffer((uint8_t *)(&_x ),4)

#endif

