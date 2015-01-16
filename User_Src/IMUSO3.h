#ifndef IMUSO3_H
#define IMUSO3_H


/* Function prototypes */
float invSqrt(float number);
void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz);
void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt);
void IMUSO3Thread(void);

void TestForIMURot(void);

#endif

