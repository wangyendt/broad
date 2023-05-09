cmake_minimum_required(VERSION 3.10)
project(ahrs_lib)

find_package(Eigen3 REQUIRED)

include_directories(./algorithms/base)
include_directories(./algorithms/madgwick)
include_directories(./algorithms/mahony)

add_executable(ahrs
        main.cpp
        algorithms/base/BaseAHRS.cc
        algorithms/madgwick/MadgwickAHRS.cc
        algorithms/mahony/MahonyAHRS.cc
        )

target_link_libraries(ahrs
        Eigen3::Eigen
        )
