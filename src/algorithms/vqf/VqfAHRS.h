//
// Created by wayne on 2023/5/9.
//

#ifndef AHRS_LIB_VQFAHRS_H
#define AHRS_LIB_VQFAHRS_H

#include "BaseAHRS.h"
#include "vqf.h"
#include <memory>

class VqfAHRS : public BaseAHRS {
public:
    explicit VqfAHRS(float tau_acc, float tau_mag, float sampleFreq);

    void update_marg(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) override;

    void update_imu(float gx, float gy, float gz, float ax, float ay, float az) override;

    void get_orientation_6x(float *quaternion) override;

    void get_orientation_9x(float *quaternion) override;

    float tau_acc, tau_mag;
private:
    std::shared_ptr<VQF> vqf;
};


#endif //AHRS_LIB_VQFAHRS_H
