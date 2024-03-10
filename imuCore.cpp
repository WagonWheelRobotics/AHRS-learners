#include "imuCore.h"
#include <QDebug>

static void mat_mul(const float *m, const int16_t *x, float *c) // c=m x a (c=4x1, m=4x4, c=4x1, only 3x4 of m is used)
{
    c[0] = m[0] * (float)x[0] + m[1] * (float)x[1] + m[ 2] * (float)x[2] + m[ 3];
    c[1] = m[4] * (float)x[0] + m[5] * (float)x[1] + m[ 6] * (float)x[2] + m[ 7];
    c[2] = m[8] * (float)x[0] + m[9] * (float)x[1] + m[10] * (float)x[2] + m[11];
}

static void mat_from_scale_offset(const float *scale, const float *offset, float *m)
{
    m[0] = scale[0]; m[1] = 0.0f;  m[ 2] = 0.0f;  m[ 3] = -scale[0] * offset[0];
    m[4] = 0.0f;  m[5] = scale[1]; m[ 6] = 0.0f;  m[ 7] = -scale[1] * offset[1];
    m[8] = 0.0f;  m[9] = 0.0f;  m[10] = scale[2]; m[11] = -scale[2] * offset[2];
}

static void scale_bias_from_1g(const float plus1g, const float minus1g, float *scale, float *bias)
{
    *scale = 2.0f / (plus1g-minus1g);
    *bias = plus1g - 1.0f / (*scale);
}

imuCore::imuCore()
{
    // manually measured
    float ax_calib[2] = {972.0f,-1076.0f}; // +1g and -1g
    float ay_calib[2] = {1072.0f,-972.0f}; // +1g and -1g
    float az_calib[2] = {765.0f,-1282.0f}; // +1g and -1g

    float acel_bias[3];
    float acel_scale[3];
    scale_bias_from_1g(ax_calib[0], ax_calib[1], &acel_scale[0], &acel_bias[0]);
    scale_bias_from_1g(ay_calib[0], ay_calib[1], &acel_scale[1], &acel_bias[1]);
    scale_bias_from_1g(az_calib[0], az_calib[1], &acel_scale[2], &acel_bias[2]);

    mat_from_scale_offset(acel_scale, acel_bias, acel_mat);

    qDebug()<< acel_scale[0] << acel_bias[0];
    qDebug()<< acel_scale[1] << acel_bias[1];
    qDebug()<< acel_scale[2] << acel_bias[2];


    float gyro_bias[3] = {13.5f,-17.75f,10.5};                  // manually measured
    float gyro_scale[3] = {1.0f/16.4f,1.0f/16.4f,1.0f/16.4f};   // from datasheet

    mat_from_scale_offset(gyro_scale, gyro_bias, gyro_mat);
}


int imuCore::get(const int16_t *rawImu, float *corImu)
{
    const int16_t *raw_gyro = &rawImu[IMU_GX];
    const int16_t *raw_acel = &rawImu[IMU_AX];

    float *cor_gyro = &corImu[IMU_GX];
    float *cor_acel = &corImu[IMU_AX];

    mat_mul(gyro_mat, raw_gyro, cor_gyro);
    mat_mul(acel_mat, raw_acel, cor_acel);

    return 1;
}
