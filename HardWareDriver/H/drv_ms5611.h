#ifndef __MS5611_H
#define __MS5611_H

#define bool char

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int   uint32_t;
typedef short 		int16_t;
typedef int 			int32_t;



typedef void (* baroOpFuncPtr)(void);                       // baro start operation
typedef void (* baroCalculateFuncPtr)(int32_t *pressure, int32_t *temperature);             // baro calculation (filled params are pressure and temperature)


typedef struct baro_t
{
    uint16_t ut_delay;
    uint16_t up_delay;
    baroOpFuncPtr start_ut;
    baroOpFuncPtr get_ut;
    baroOpFuncPtr start_up;
    baroOpFuncPtr get_up;
    baroCalculateFuncPtr calculate;
} baro_t;

bool ms5611Detect(baro_t *baro);


#endif