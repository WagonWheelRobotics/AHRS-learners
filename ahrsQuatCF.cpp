#include "ahrsQuatCF.h"
#include "imuCore.h"
#include <cmath>

ahrsQuatCF::ahrsQuatCF(float dt) : ahrsCore(dt)
{
    _init = 1;  // requires initialization

    float tc = 15.0f;   // time constant [sec]

    _k = 1.0f / (1.0f + (tc/dt));

    _output.emplace_back("q0");
    _output.emplace_back("q1");
    _output.emplace_back("q2");
    _output.emplace_back("q3");
    _output.emplace_back("an_x");
    _output.emplace_back("an_y");
    _output.emplace_back("an_z");
    _output.emplace_back("fall");
}

int ahrsQuatCF::update(const float *imu, std::vector<float> &output)
{
    if(_init)
    {
        //alignment
        float euler[3];
        euler[0] = std::atan2(imu[IMU_AY],imu[IMU_AZ]);									// roll[rad]
        euler[1] =-std::atan2(imu[IMU_AX],std::sqrt(SQ(imu[IMU_AY])+SQ(imu[IMU_AZ])));	// pitch[rad]
        euler[2] = 0.0f; //yaw[rad]
        euler2dcm(euler,dcm);
        dcm2quat(dcm,q);
        _init=0;
    }

    //gyroscope measurement in [rad/s] x [s]
    float wx = imu[IMU_GX] * D2R * _dt;
    float wy = imu[IMU_GY] * D2R * _dt;
    float wz = imu[IMU_GZ] * D2R * _dt;

    float d[4];     //delta quaternion
    d[0] = 0.5f *(         -wx*q[1] -wy*q[2] -wz*q[3]);
    d[1] = 0.5f *( wx*q[0]          +wz*q[2] -wy*q[3]);
    d[2] = 0.5f *( wy*q[0] -wz*q[1]          +wx*q[3]);
    d[3] = 0.5f *( wz*q[0] +wy*q[1] -wx*q[2]         );

    q[0] += d[0];
    q[1] += d[1];
    q[2] += d[2];
    q[3] += d[3];
    normalizeQuat(q);

    float a_n[3]={0.0f,0.0f,1.0f};

    float at_sq = SQ(imu[IMU_AX]) + SQ(imu[IMU_AY]) + SQ(imu[IMU_AZ]);
    float j = std::abs(std::round(at_sq * 10000.0f));
    if(j<1.0f)
    {
        //near free fall, skip filtering
        j=0.0f;
    }
    else
    {
        j=std::sqrt(j); // for free fall monitoring
        float iat = 1.0f/std::sqrt(at_sq);
        float a_b[3] = {imu[IMU_AX] * iat, imu[IMU_AY] * iat, imu[IMU_AZ] * iat};   // normalized accelerometer reading
        vec3RotQuat(q,a_b,a_n);
        const float g_n[3]={0.0f,0.0f,1.0f};
        float axg[3];
        crossV3(a_n,g_n,axg);
        float n=norm(axg,3);
        if(std::abs(n)>0.001f)
        {
            float na[3] = {axg[0]/n, axg[1]/n, axg[2]/n};
            float theta = std::acos(innerV3(a_n,g_n));  //[rad]
            float qa[4];
            float sa=std::sin(_k*theta*0.5f);
            qa[0]=std::cos(_k*theta*0.5f);
            qa[1]=na[0] * sa;
            qa[2]=na[1] * sa;
            qa[3]=na[2] * sa;
            mulQuat(qa,q,q);
            normalizeQuat(q);
        }
        else
        {
            // error is too small
        }
    }

    // for plot
    output.resize(8);
    output[0] = q[0];
    output[1] = q[1];
    output[2] = q[2];
    output[3] = q[3];
    output[4] = a_n[0];
    output[5] = a_n[1];
    output[6] = a_n[2];
    output[7] = j*0.03f;    //scaling 100->3

    // for 3D model
    _outputQuat[0] = q[0];
    _outputQuat[1] = q[1];
    _outputQuat[2] = q[2];
    _outputQuat[3] = q[3];

    return 1;

}

bool ahrsQuatCF::outputIsEuler() const
{
    return false;
}
