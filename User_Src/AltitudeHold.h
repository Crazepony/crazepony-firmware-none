#ifndef _AltitudeHold_H_
#define _AltitudeHold_H_

#include "FT.h"

extern int32_t AltHold;
extern int32_t vario;
extern int16_t relThrottle;


typedef struct airplaneConfig_s {
    int8_t fixedwing_althold_dir;           // +1 or -1 for pitch/althold gain. later check if need more than just sign
} airplaneConfig_t;


void calculateEstimatedAltitude(uint32_t currentTime);

void applyAltHold(void);
void updateAltHoldState(void);

int32_t altitudeHoldGetEstimatedAltitude(void);

#endif
