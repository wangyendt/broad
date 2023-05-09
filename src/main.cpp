#include <iostream>

#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"

void print_quaternion(float *quat) {
    std::cout << quat[0] << ","
              << quat[1] << ","
              << quat[2] << ","
              << quat[3]
              << std::endl;
}

int main(int argc, char **argv) {

    std::cout.precision(10);

    const float acc[3] = {9.81f, 0.0f, 0.0f};
    const float gyro[3] = {0.1f, 0.1f, 0.1f};
    const float mag[3] = {0.1f, 0.1f, 0.1f};
    float quat[4];

    MadgwickAHRS madgwickAhrs(0.033, 100);
    MahonyAHRS mahonyAhrs(0.1, 0.1, 100);

    madgwickAhrs.update_imu(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);
    madgwickAhrs.update_marg(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2]);
    madgwickAhrs.get_orientation(quat);
    print_quaternion(quat);

    mahonyAhrs.update_imu(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);
    mahonyAhrs.update_marg(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2]);
    mahonyAhrs.get_orientation(quat);
    print_quaternion(quat);

    return 0;
}
