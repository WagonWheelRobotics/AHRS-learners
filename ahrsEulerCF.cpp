#include "ahrsEulerCF.h"
#include "imuCore.h"
#include <cmath>

ahrsEulerCF::ahrsEulerCF(float dt) : ahrsCore(dt)
{
    _init = 1;  // requires initialization

    float tc = 15.0f;   // time constant [sec]

    _k = 1.0f / (1.0f + (tc/dt));

    _output.emplace_back("roll");
    _output.emplace_back("pitch");
    _output.emplace_back("yaw");
    _output.emplace_back("roll_ref");
    _output.emplace_back("pitch_ref");
}

int ahrsEulerCF::update(const float *imu, std::vector<float> &output)
{
    if(_init)
    {
        //alignment
        euler[0] = std::atan2(imu[IMU_AY],imu[IMU_AZ]);									// roll[rad]
        euler[1] =-std::atan2(imu[IMU_AX],std::sqrt(SQ(imu[IMU_AY])+SQ(imu[IMU_AZ])));	// pitch[rad]
        euler[2] = 0.0f; //yaw[rad]

        _init=0;
    }

    //gyroscope measurement in [rad/s] x [s]
    float wx = imu[IMU_GX] * D2R * _dt;
    float wy = imu[IMU_GY] * D2R * _dt;
    float wz = imu[IMU_GZ] * D2R * _dt;

    float sin_r =std::sin(euler[0]);
    float cos_r =std::cos(euler[0]);
    float sin_p =std::sin(euler[1]);
    float cos_p =std::cos(euler[1]);
    float sec_p=1.0f/cos_p;
    float tan_p=sin_p*sec_p;

    //attitude increments from previous attitude and gyroscope measurement
    float r_dot=wx + sin_r*tan_p*wy + cos_r*tan_p*wz;
    float p_dot=     cos_r      *wy - sin_r      *wz;
    float y_dot=     sin_r*sec_p*wy + cos_r*sec_p*wz;

    //gravity reference for roll & pitch (includes error due to translational acceleration)
    float ref_r = std::atan2(imu[IMU_AY],imu[IMU_AZ]);									// roll[rad]
    float ref_p =-std::atan2(imu[IMU_AX],std::sqrt(SQ(imu[IMU_AY])+SQ(imu[IMU_AZ])));	// pitch[rad]

    //complementary filter (High-pass gyroscope and Low-pass gravity reference)
    euler[0] =(euler[0] +r_dot)*(1.0f-_k) + ref_r*_k;
    euler[1] =(euler[1] +p_dot)*(1.0f-_k) + ref_p*_k;

    // Yaw angle is purely integral since there is no reference
    euler[2] = euler[2] +y_dot;

    // limit yaw angle in +-180deg
    if (euler[2]> A180) euler[2] -= A360;
    else if (euler[2]<-A180) euler[2] += A360;

    // for plot
    output.resize(5);
    output[0] = euler[0];   //roll [rad]
    output[1] = euler[1];   //pitch [rad]
    output[2] = euler[2];   //yaw [rad]
    output[3] = ref_r;   //roll reference [rad]
    output[4] = ref_p;   //pitch reference [rad]

    // for 3D model
    _outputEuler[0] = euler[0];
    _outputEuler[1] = euler[1];
    _outputEuler[2] = euler[2];


    //_outputEuler[0] = ref_r;
    //_outputEuler[1] = ref_p;

    return 1;
}
