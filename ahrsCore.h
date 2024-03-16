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
    void getQuat(float *quat) const;

    virtual bool outputIsEuler(void) const; //for 3D model

protected:
    void euler2dcm(const float *euler, float *dcm) const;
    void quat2dcm(const float *q, float *dcm) const;
    void dcm2quat(const float *dcm, float *quat) const;
    void normalizeQuat(float *q) const;
    void mulQuat(const float *q, const float *p, float *qp) const;
    void invQuat(const float *q, float *iq) const;
    void vec3RotQuat(const float *q, const float *v3, float *o3) const;
    void crossV3(const float *a, const float *b, float *axb) const;
    float innerV3(const float *a, const float *b) const;
    float norm(const float *a, int len) const;

protected:
    float _dt;
    std::vector<std::string> _output;
    float _outputEuler[3];      //for 3D model
    float _outputQuat[4];       //for 3D model
};

#define D2R ((float)(M_PI/180.0))
#define R2D ((float)(180.0/M_PI))
#define A180 ((float)M_PI)
#define A360 ((float)M_PI*2.0f)
#define SQ(x) ((x)*(x))

#endif // AHRSCORE_H
