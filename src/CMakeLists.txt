cmake_minimum_required(VERSION 3.10)
project(ahrs_lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

file(GLOB_RECURSE ALGORITHM_FILES
        algorithms/base/*.cc
        algorithms/base/*.h
        algorithms/madgwick/*.cc
        algorithms/madgwick/*.h
        algorithms/mahony/*.cc
        algorithms/mahony/*.h
        )

add_executable(test_algorithms test/test_algorithms.cpp ${ALGORITHM_FILES})
add_executable(main main.cpp data_processing/DataProcessing.cpp ${ALGORITHM_FILES})

target_include_directories(test_algorithms PUBLIC
        algorithms/base
        algorithms/madgwick
        algorithms/mahony
        data_processing
        )

target_include_directories(main PUBLIC
        algorithms/base
        algorithms/madgwick
        algorithms/mahony
        data_processing
        utils
        )

target_link_libraries(test_algorithms Eigen3::Eigen)
target_link_libraries(main Eigen3::Eigen)
