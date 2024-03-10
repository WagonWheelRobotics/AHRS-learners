#ifndef AHRSEULERCF_H
#define AHRSEULERCF_H

#include "ahrsCore.h"

class ahrsEulerCF : public ahrsCore
{
public:
    ahrsEulerCF(float dt);

    virtual int update(const float *imu, std::vector<float> &output);

private:
    int _init;
    float euler[3];
    float dcm[9];
    float _k;
};

#endif // AHRSEULERCF_H
