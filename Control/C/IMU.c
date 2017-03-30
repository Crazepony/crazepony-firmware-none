#include "imu.h"

//
float accFilted[3];	//m/s2,in body frame, filtered
float DCMbg[3][3];


//-----Get raw acc gyro data from mpu6050 -----//(in1Khz)


//----cut offset and scale to m/s2 deg/s
//-----------Filter -----------//
static float           _cutoff_freq;
static float           _a1;
static float           _a2;
static float           _b0;
static float           _b1;
static float           _b2;
static float           _delay_element_1;        // buffered sample -1
static float           _delay_element_2;        // buffered sample -2
void LowPassFilter2p_set_cutoff_frequency(float sample_freq, float cutoff_freq)
{
    float fr = sample_freq/cutoff_freq;
    float ohm = tanf(M_PI_F/fr);
    float c = 1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;

    _cutoff_freq = cutoff_freq;
    if (_cutoff_freq > 0.0f)
    {
        _b0 = ohm*ohm/c;
        _b1 = 2.0f*_b0;
        _b2 = _b0;
        _a1 = 2.0f*(ohm*ohm-1.0f)/c;
        _a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
    }
}

float LowPassFilter2p_apply(float sample)
{

    float delay_element_0 = 0, output=0;
    if (_cutoff_freq <= 0.0f) {
        // no filtering
        return sample;
    }
    else
    {
        delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;
        // do the filtering
        if (isnan(delay_element_0) || isinf(delay_element_0)) {
            // don't allow bad values to propogate via the filter
            delay_element_0 = sample;
        }
        output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;

        _delay_element_2 = _delay_element_1;
        _delay_element_1 = delay_element_0;

        // return the value.  Should be no need to check limits
        return output;
    }
}
//----Run software imu or use dmp, to get quat DCM euler-----------//
// get DCM from body to ground
void quat2DCM(float DCM[3][3],float q[4])
{
    float q0=q[0],q1=q[1],q2=q[2],q3=q[3];
    float q12=q1*q1, q22=q2*q2, q32=q3*q3;

    DCM[0][0]=1.0f -2.0f * (q22   + q32);
    DCM[1][0]=			2.0f * (q1*q2 + q0*q3);
    DCM[2][0]=			2.0f * (q1*q3 - q0*q2);

    DCM[0][1]= 			2.0f * (q1*q2 - q0*q3);
    DCM[1][1]=1.0f -2.0f * (q12   +  q32);
    DCM[2][1]=			2.0f * (q2*q3 + q0*q1);

    DCM[0][2]=			2.0f * (q1*q3 + q0*q2);
    DCM[1][2]=			2.0f * (q2*q3 - q0*q1);
    DCM[2][2]=1.0f -2.0f * (q12 + q22);
}

//-------------Get acc in earth frame---------------//



