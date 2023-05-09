//
// Created by wayne on 2023/5/9.
//

#ifndef AHRS_LIB_BASEAHRS_H
#define AHRS_LIB_BASEAHRS_H


class BaseAHRS {
public:
    explicit BaseAHRS(float frequency);
    void get_orientation(float *quaternion) const;
    virtual void update_marg(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    virtual void update_imu(float gx, float gy, float gz, float ax, float ay, float az);
    float qw, qx, qy, qz;
    float frequency;
};


#endif //AHRS_LIB_BASEAHRS_H
