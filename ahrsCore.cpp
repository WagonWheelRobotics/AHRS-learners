#include "ahrsCore.h"
#include <cmath>

ahrsCore::ahrsCore(float dt)
{
    _dt = dt;
    _outputEuler[0] = 0.0f;
    _outputEuler[1] = 0.0f;
    _outputEuler[2] = 0.0f;
}

int ahrsCore::update(const float *imu, std::vector<float> &output)
{
    return 0;
}

const std::vector<std::string> &ahrsCore::output() const
{
    return _output;
}

void ahrsCore::getEuler(float *euler) const
{
    euler[0] = _outputEuler[0];
    euler[1] = _outputEuler[1];
    euler[2] = _outputEuler[2];
}

void ahrsCore::euler2dcm(const float *euler, float *dcm) const
{
    float cr = std::cos(euler[0]);
    float sr = std::sin(euler[0]);

    float cp = std::cos(euler[1]);
    float sp = std::sin(euler[1]);

    float cy = std::cos(euler[2]);
    float sy = std::sin(euler[2]);

    dcm[0] = cy * cp;
    dcm[1] = cy * sp * sr - sy * cr;
    dcm[2] = cy * sp * cr + sy * sr;

    dcm[3] = sy * cp;
    dcm[4] = sy * sp * sr + cy * cr;
    dcm[5] = -cy * sr + sy * sp * cr;

    dcm[6] = -sp;
    dcm[7] = cp * sr;
    dcm[8] = cp * cr;
}

void ahrsCore::dcm2quat(const float *dcm, float *quat) const
{
    float tr;
    tr = dcm[0] + dcm[4] + dcm[8];

    if (tr > 0.0f)
    {
        float p0 = std::sqrt(1.0f + tr);
        quat[0] = p0 * 0.5f;
        quat[1] = 0.5f * (dcm[7] - dcm[5]) / p0;		//C(2, 1) - C(1, 2)
        quat[2] = 0.5f * (dcm[2] - dcm[6]) / p0;		//C(0, 2) - C(2, 0)
        quat[3] = 0.5f * (dcm[3] - dcm[1]) / p0;		//C(1, 0) - C(0, 1)
    }
    else
    {
        if (dcm[0] > dcm[4] && dcm[0] > dcm[8])
        {
            float s = 2.0f * std::sqrt(1.0f + dcm[0] - dcm[4] - dcm[8]);
            quat[0] = (dcm[7] - dcm[5]) / s;		//C(2, 1) - C(1, 2)
            quat[1] = s * 0.25f;
            quat[2] = (dcm[3] + dcm[1]) / s;		//C(1, 0) + C(0, 1)
            quat[3] = (dcm[2] + dcm[6]) / s;		//C(0, 2) + C(2, 0)
        }
        else if (dcm[4] > dcm[8])
        {
            float s = 2.0f * std::sqrt(1.0f - dcm[0] + dcm[4] - dcm[8]);
            quat[0] = (dcm[2] - dcm[6]) / s;		//C(0, 2) - C(2, 0)
            quat[1] = (dcm[3] + dcm[1]) / s;		//C(1, 0) + C(0, 1)
            quat[2] = s * 0.25f;
            quat[3] = (dcm[7] + dcm[5]) / s;		//C(2, 1) + C(1, 2)
        }
        else
        {
            float s = 2.0f * std::sqrt(1.0f - dcm[0] - dcm[4] + dcm[8]);
            quat[0] = (dcm[3] - dcm[1]) / s;		//C(1, 0) - C(0, 1)
            quat[1] = (dcm[2] + dcm[6]) / s;		//C(0, 2) + C(2, 0)
            quat[2] = (dcm[7] + dcm[5]) / s;		//C(2, 1) + C(1, 2)
            quat[3] = s * 0.25f;
        }
    }
}

