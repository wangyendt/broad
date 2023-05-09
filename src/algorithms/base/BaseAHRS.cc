//
// Created by wayne on 2023/5/9.
//

#include "BaseAHRS.h"

void BaseAHRS::get_orientation_6x(float *quaternion) {
    quaternion[0] = qw;
    quaternion[1] = qx;
    quaternion[2] = qy;
    quaternion[3] = qz;
}

void BaseAHRS::get_orientation_9x(float *quaternion) {
    quaternion[0] = qw;
    quaternion[1] = qx;
    quaternion[2] = qy;
    quaternion[3] = qz;
}

BaseAHRS::BaseAHRS(float frequency) : qw(1.0f), qx(0.0f), qy(0.0f), qz(0.0f) {
    this->frequency = frequency;
}

void BaseAHRS::update_marg(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {

}

void BaseAHRS::update_imu(float gx, float gy, float gz, float ax, float ay, float az) {

}
