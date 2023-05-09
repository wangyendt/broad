//
// Created by wayne on 2023/5/9.
//

#include "VqfAHRS.h"

VqfAHRS::VqfAHRS(float tau_acc, float tau_mag, float sampleFreq) :
        BaseAHRS(sampleFreq),
        tau_acc(tau_acc), tau_mag(tau_mag) {
    vqf = std::make_shared<VQF>(1.0 / sampleFreq, 1.0 / sampleFreq);
    vqf->resetState();
    vqf->setTauAcc(tau_acc);
    vqf->setTauMag(tau_mag);
}

void VqfAHRS::update_marg(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float acc[3] = {ax, ay, az};
    float gyro[3] = {gx, gy, gz};
    float mag[3] = {mx, my, mz};
    vqf->update(gyro, acc, mag);
}

void VqfAHRS::update_imu(float gx, float gy, float gz, float ax, float ay, float az) {
    float acc[3] = {ax, ay, az};
    float gyro[3] = {gx, gy, gz};
    vqf->update(gyro, acc);
}

void VqfAHRS::get_orientation_6x(float *quaternion) {
    vqf->getQuat6D(quaternion);
}

void VqfAHRS::get_orientation_9x(float *quaternion) {
    vqf->getQuat9D(quaternion);
}
