#ifndef AHRSQUATCF_H
#define AHRSQUATCF_H

#include "ahrsCore.h"

class ahrsQuatCF : public ahrsCore
{
public:
    ahrsQuatCF(float dt);

    virtual int update(const float *imu, std::vector<float> &output);

    virtual bool outputIsEuler(void) const;

private:
    int _init;
    float q[4];
    float dcm[9];
    float _k;
};

#endif // AHRSQUATCF_H
