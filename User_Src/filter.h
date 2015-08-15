#ifndef _FILTER_H
#define _FILTER_H

#include <math.h>

void LPF2pSetCutoffFreq_1(float sample_freq, float cutoff_freq);
float LPF2pApply_1(float sample);
void LPF2pSetCutoffFreq_2(float sample_freq, float cutoff_freq);
float LPF2pApply_2(float sample);
void LPF2pSetCutoffFreq_3(float sample_freq, float cutoff_freq);
float LPF2pApply_3(float sample);
void LPF2pSetCutoffFreq_4(float sample_freq, float cutoff_freq);
float LPF2pApply_4(float sample);
void LPF2pSetCutoffFreq_5(float sample_freq, float cutoff_freq);
float LPF2pApply_5(float sample);
void LPF2pSetCutoffFreq_6(float sample_freq, float cutoff_freq);
float LPF2pApply_6(float sample);

#endif

