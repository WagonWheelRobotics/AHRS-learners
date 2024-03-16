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

void ahrsCore::getQuat(float *quat) const
{
    quat[0] = _outputQuat[0];
    quat[1] = _outputQuat[1];
    quat[2] = _outputQuat[2];
    quat[3] = _outputQuat[3];
}

bool ahrsCore::outputIsEuler() const
{
    return true;
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

void ahrsCore::quat2dcm(const float *q, float *dcm) const
{
    dcm[0] = SQ(q[0])+SQ(q[1])-SQ(q[2])-SQ(q[3]);
    dcm[1] = 2.0f*(q[1]*q[2]-q[0]*q[3]);
    dcm[2] = 2.0f*(q[1]*q[3]+q[0]*q[2]);
    dcm[3] = 2.0f*(q[1]*q[2]+q[0]*q[3]);
    dcm[4] = SQ(q[0])-SQ(q[1])+SQ(q[2])-SQ(q[3]);
    dcm[5] = 2.0f*(q[2]*q[3]-q[0]*q[1]);
    dcm[6] = 2.0f*(q[1]*q[3]-q[0]*q[2]);
    dcm[7] = 2.0f*(q[2]*q[3]+q[0]*q[1]);
    dcm[8] = SQ(q[0])-SQ(q[1])-SQ(q[2])+SQ(q[3]);
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

void ahrsCore::normalizeQuat(float *q) const
{
    float it = 1.0f/std::sqrt(SQ(q[0])+SQ(q[1])+SQ(q[2])+SQ(q[3]));
    q[0]*=it;
    q[1]*=it;
    q[2]*=it;
    q[3]*=it;
}

void ahrsCore::mulQuat(const float *q, const float *p, float *qp) const
{
    float y[4];
    y[0] = q[0]*p[0] -q[1]*p[1] -q[2]*p[2] -q[3]*p[3];
    y[1] = q[1]*p[0] +q[0]*p[1] -q[3]*p[2] +q[2]*p[3];
    y[2] = q[2]*p[0] +q[3]*p[1] +q[0]*p[2] -q[1]*p[3];
    y[3] = q[3]*p[0] -q[2]*p[1] +q[1]*p[2] +q[0]*p[3];
    qp[0]=y[0];
    qp[1]=y[1];
    qp[2]=y[2];
    qp[3]=y[3];
}

void ahrsCore::invQuat(const float *q, float *iq) const
{
    iq[0]= q[0];
    iq[1]=-q[1];
    iq[2]=-q[2];
    iq[3]=-q[3];
}

void ahrsCore::vec3RotQuat(const float *q, const float *v3, float *o3) const
{
#if 1
    float p[4]={0.0f, v3[0],v3[1],v3[2]};
    float iq[4],qp[4];
    invQuat(q,iq);
    mulQuat(q,p,qp);
    mulQuat(qp,iq,p);
    o3[0] = p[1];
    o3[1] = p[2];
    o3[2] = p[3];
#else
    float C[9],p[3];
    quat2dcm(q,C);
    p[0]=C[0]*v3[0]+C[1]*v3[1]+C[2]*v3[2];
    p[1]=C[3]*v3[0]+C[4]*v3[1]+C[5]*v3[2];
    p[2]=C[6]*v3[0]+C[7]*v3[1]+C[8]*v3[2];
    o3[0]=p[0];
    o3[1]=p[1];
    o3[2]=p[2];
#endif
}

void ahrsCore::crossV3(const float *a, const float *b, float *axb) const
{
    float p[3];
    p[0] = a[1]*b[2]-a[2]*b[1];
    p[1] = a[2]*b[0]-a[0]*b[2];
    p[2] = a[0]*b[1]-a[1]*b[0];
    axb[0]=p[0];
    axb[1]=p[1];
    axb[2]=p[2];
}

float ahrsCore::innerV3(const float *a, const float *b) const
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

float ahrsCore::norm(const float *a, int len) const
{
    float t=0.0f;
    for(int i=0;i<len;i++)  t+=SQ(a[i]);
    return std::sqrt(t);
}

