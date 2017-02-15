/*
*  TOBE FIXED: Use filter object instead of repeated code
*
*/

#include "SysConfig.h"
#include "imu.h"
#include "filter.h"



static float           _cutoff_freq1;
static float           _a11;
static float           _a21;
static float           _b01;
static float           _b11;
static float           _b21;
static float           _delay_element_11;        // buffered sample -1
static float           _delay_element_21;        // buffered sample -2
void LPF2pSetCutoffFreq_1(float sample_freq, float cutoff_freq)
{
    float fr =0;
    float ohm =0;
    float c =0;

    fr= sample_freq/cutoff_freq;
    ohm=tanf(M_PI_F/fr);
    c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;

    _cutoff_freq1 = cutoff_freq;
    if (_cutoff_freq1 > 0.0f)
    {
        _b01 = ohm*ohm/c;
        _b11 = 2.0f*_b01;
        _b21 = _b01;
        _a11 = 2.0f*(ohm*ohm-1.0f)/c;
        _a21 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
    }
}

float LPF2pApply_1(float sample)
{

    float delay_element_0 = 0, output=0;
    if (_cutoff_freq1 <= 0.0f) {
        // no filtering
        return sample;
    }
    else
    {
        delay_element_0 = sample - _delay_element_11 * _a11 - _delay_element_21 * _a21;
        // do the filtering
        if (isnan(delay_element_0) || isinf(delay_element_0)) {
            // don't allow bad values to propogate via the filter
            delay_element_0 = sample;
        }
        output = delay_element_0 * _b01 + _delay_element_11 * _b11 + _delay_element_21 * _b21;

        _delay_element_21 = _delay_element_11;
        _delay_element_11 = delay_element_0;

        // return the value.  Should be no need to check limits
        return output;
    }
}

static float           _cutoff_freq2;
static float           _a12;
static float           _a22;
static float           _b02;
static float           _b12;
static float           _b22;
static float           _delay_element_12;        // buffered sample -1
static float           _delay_element_22;        // buffered sample -2
void LPF2pSetCutoffFreq_2(float sample_freq, float cutoff_freq)
{
    float fr =0;
    float ohm =0;
    float c =0;

    fr= sample_freq/cutoff_freq;
    ohm=tanf(M_PI_F/fr);
    c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;

    _cutoff_freq2 = cutoff_freq;
    if (_cutoff_freq2 > 0.0f)
    {
        _b02 = ohm*ohm/c;
        _b12 = 2.0f*_b02;
        _b22 = _b02;
        _a12 = 2.0f*(ohm*ohm-1.0f)/c;
        _a22 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
    }
}

float LPF2pApply_2(float sample)
{

    float delay_element_0 = 0, output=0;
    if (_cutoff_freq2 <= 0.0f) {
        // no filtering
        return sample;
    }
    else
    {
        delay_element_0 = sample - _delay_element_12 * _a12 - _delay_element_22 * _a22;
        // do the filtering
        if (isnan(delay_element_0) || isinf(delay_element_0)) {
            // don't allow bad values to propogate via the filter
            delay_element_0 = sample;
        }
        output = delay_element_0 * _b02 + _delay_element_12 * _b12 + _delay_element_22 * _b22;

        _delay_element_22 = _delay_element_12;
        _delay_element_12 = delay_element_0;

        // return the value.  Should be no need to check limits
        return output;
    }
}

static float           _cutoff_freq3;
static float           _a13;
static float           _a23;
static float           _b03;
static float           _b13;
static float           _b23;
static float           _delay_element_13;        // buffered sample -1
static float           _delay_element_23;        // buffered sample -2
void LPF2pSetCutoffFreq_3(float sample_freq, float cutoff_freq)
{
    float fr =0;
    float ohm =0;
    float c =0;

    fr= sample_freq/cutoff_freq;
    ohm=tanf(M_PI_F/fr);
    c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;

    _cutoff_freq3 = cutoff_freq;
    if (_cutoff_freq3 > 0.0f)
    {
        _b03 = ohm*ohm/c;
        _b13 = 2.0f*_b03;
        _b23 = _b03;
        _a13 = 2.0f*(ohm*ohm-1.0f)/c;
        _a23 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
    }
}

float LPF2pApply_3(float sample)
{

    float delay_element_0 = 0, output=0;
    if (_cutoff_freq3 <= 0.0f) {
        // no filtering
        return sample;
    }
    else
    {
        delay_element_0 = sample - _delay_element_13 * _a13 - _delay_element_23 * _a23;
        // do the filtering
        if (isnan(delay_element_0) || isinf(delay_element_0)) {
            // don't allow bad values to propogate via the filter
            delay_element_0 = sample;
        }
        output = delay_element_0 * _b03 + _delay_element_13 * _b13 + _delay_element_23 * _b23;

        _delay_element_23 = _delay_element_13;
        _delay_element_13 = delay_element_0;

        // return the value.  Should be no need to check limits
        return output;
    }
}


static float           _cutoff_freq4;
static float           _a14;
static float           _a24;
static float           _b04;
static float           _b14;
static float           _b24;
static float           _delay_element_14;        // buffered sample -1
static float           _delay_element_24;        // buffered sample -2
void LPF2pSetCutoffFreq_4(float sample_freq, float cutoff_freq)
{
    float fr =0;
    float ohm =0;
    float c =0;

    fr= sample_freq/cutoff_freq;
    ohm=tanf(M_PI_F/fr);
    c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;

    _cutoff_freq4 = cutoff_freq;
    if (_cutoff_freq4 > 0.0f)
    {
        _b04 = ohm*ohm/c;
        _b14 = 2.0f*_b04;
        _b24 = _b04;
        _a14 = 2.0f*(ohm*ohm-1.0f)/c;
        _a24 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
    }
}

float LPF2pApply_4(float sample)
{

    float delay_element_0 = 0, output=0;
    if (_cutoff_freq4 <= 0.0f) {
        // no filtering
        return sample;
    }
    else
    {
        delay_element_0 = sample - _delay_element_14 * _a14 - _delay_element_24 * _a24;
        // do the filtering
        if (isnan(delay_element_0) || isinf(delay_element_0)) {
            // don't allow bad values to propogate via the filter
            delay_element_0 = sample;
        }
        output = delay_element_0 * _b04 + _delay_element_14 * _b14 + _delay_element_24 * _b24;

        _delay_element_24 = _delay_element_14;
        _delay_element_14 = delay_element_0;

        // return the value.  Should be no need to check limits
        return output;
    }
}



static float           _cutoff_freq5;
static float           _a15;
static float           _a25;
static float           _b05;
static float           _b15;
static float           _b25;
static float           _delay_element_15;        // buffered sample -1
static float           _delay_element_25;        // buffered sample -2
void LPF2pSetCutoffFreq_5(float sample_freq, float cutoff_freq)
{
    float fr =0;
    float ohm =0;
    float c =0;

    fr= sample_freq/cutoff_freq;
    ohm=tanf(M_PI_F/fr);
    c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;

    _cutoff_freq5 = cutoff_freq;
    if (_cutoff_freq5 > 0.0f)
    {
        _b05 = ohm*ohm/c;
        _b15 = 2.0f*_b05;
        _b25 = _b05;
        _a15 = 2.0f*(ohm*ohm-1.0f)/c;
        _a25 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
    }
}

float LPF2pApply_5(float sample)
{

    float delay_element_0 = 0, output=0;
    if (_cutoff_freq5 <= 0.0f) {
        // no filtering
        return sample;
    }
    else
    {
        delay_element_0 = sample - _delay_element_15 * _a15 - _delay_element_25 * _a25;
        // do the filtering
        if (isnan(delay_element_0) || isinf(delay_element_0)) {
            // don't allow bad values to propogate via the filter
            delay_element_0 = sample;
        }
        output = delay_element_0 * _b05 + _delay_element_15 * _b15 + _delay_element_25 * _b25;

        _delay_element_25 = _delay_element_15;
        _delay_element_15 = delay_element_0;

        // return the value.  Should be no need to check limits
        return output;
    }
}



static float           _cutoff_freq6;
static float           _a16;
static float           _a26;
static float           _b06;
static float           _b16;
static float           _b26;
static float           _delay_element_16;        // buffered sample -1
static float           _delay_element_26;        // buffered sample -2
void LPF2pSetCutoffFreq_6(float sample_freq, float cutoff_freq)
{
    float fr =0;
    float ohm =0;
    float c =0;

    fr= sample_freq/cutoff_freq;
    ohm=tanf(M_PI_F/fr);
    c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;

    _cutoff_freq6 = cutoff_freq;
    if (_cutoff_freq6 > 0.0f)
    {
        _b06 = ohm*ohm/c;
        _b16 = 2.0f*_b06;
        _b26 = _b06;
        _a16 = 2.0f*(ohm*ohm-1.0f)/c;
        _a26 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
    }
}

float LPF2pApply_6(float sample)
{

    float delay_element_0 = 0, output=0;
    if (_cutoff_freq6 <= 0.0f) {
        // no filtering
        return sample;
    }
    else
    {
        delay_element_0 = sample - _delay_element_16 * _a16 - _delay_element_26 * _a26;
        // do the filtering
        if (isnan(delay_element_0) || isinf(delay_element_0)) {
            // don't allow bad values to propogate via the filter
            delay_element_0 = sample;
        }
        output = delay_element_0 * _b06 + _delay_element_16 * _b16 + _delay_element_26 * _b26;

        _delay_element_26 = _delay_element_16;
        _delay_element_16 = delay_element_0;

        // return the value.  Should be no need to check limits
        return output;
    }
}
