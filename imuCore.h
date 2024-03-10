#ifndef IMUCORE_H
#define IMUCORE_H

#include <cstdint>

#define IMU_GX 0    // [deg/s]
#define IMU_GY 1    // [deg/s]
#define IMU_GZ 2    // [deg/s]
#define IMU_AX 3    // [g]
#define IMU_AY 4    // [g]
#define IMU_AZ 5    // [g]

class imuCore
{
public:
    imuCore();
    int get(const int16_t *rawImu, float *corImu);

private:
    float gyro_mat[12];
    float acel_mat[12];
};

#endif // IMUCORE_H
