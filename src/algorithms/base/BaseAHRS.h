//
// Created by wayne on 2023/5/9.
//

#ifndef AHRS_LIB_BASEAHRS_H
#define AHRS_LIB_BASEAHRS_H


class BaseAHRS {
public:
    explicit BaseAHRS(float frequency);
    void get_orientation(float *quaternion) const;
    float qw, qx, qy, qz;
    float frequency;
};


#endif //AHRS_LIB_BASEAHRS_H
