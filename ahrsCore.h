#ifndef AHRSCORE_H
#define AHRSCORE_H

#include <vector>
#include <string>

class ahrsCore
{
public:
    ahrsCore(float dt);

    virtual int update(const float *imu, std::vector<float> &output);

    const std::vector<std::string> &output() const;

    void getEuler(float *euler) const;

protected:
    void euler2dcm(const float *euler, float *dcm) const;
    void dcm2quat(const float *dcm, float *quat) const;

protected:
    float _dt;
    std::vector<std::string> _output;
    float _outputEuler[3];
};

#define D2R ((float)(M_PI/180.0))
#define R2D ((float)(180.0/M_PI))
#define A180 ((float)M_PI)
#define A360 ((float)M_PI*2.0f)
#define SQ(x) ((x)*(x))

#endif // AHRSCORE_H
