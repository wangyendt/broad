//
// Created by wayne on 2023/5/9.
//

#include "BaseAHRS.h"
#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"
#include "DataProcessing.h"
#include "SaveFileUtils.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <regex>
#include <string>
#include <unordered_map>

DataProcessing::DataProcessing(std::string &file_path, std::string &method) :
        file_path(file_path), method(method) {
}

std::vector<std::string> split(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream token_stream(s);
    while (std::getline(token_stream, token, delimiter)) {
        tokens.emplace_back(token);
    }
    return tokens;
}

void DataProcessing::load_data() {
    std::ifstream file(file_path);
    std::string line;

    if (file.is_open()) {
        std::getline(file, line); // Read header
        header = split(line, ',');
        header.assign(header.begin(), header.begin() + 10);

        while (std::getline(file, line)) {
            std::unordered_map<std::string, float> row_data;
            std::vector<std::string> row_values = split(line, ',');

            for (size_t i = 0; i < 10; ++i) {
                row_data[header[i]] = std::stof(row_values[i]);
            }

            data.emplace_back(row_data);
        }

        file.close();
    }
}

void DataProcessing::process_data() {
    std::shared_ptr<BaseAHRS> ahrs;
    if (method == "madgwick") {
        ahrs = std::make_shared<MadgwickAHRS>(0.033f, 2000.0f / 7);
    } else if (method == "mahony") {
        ahrs = std::make_shared<MahonyAHRS>(0.1, 0.1, 2000.0f / 7);
    }
    float quat_out[4];
    quat.clear();
    for (auto &d: data) {
//        ahrs->update_imu(
//                d["gyro_x"], d["gyro_y"], d["gyro_z"],
//                d["acc_x"], d["acc_y"], d["acc_z"]
//        );
        ahrs->update_marg(
                d["gyro_x"], d["gyro_y"], d["gyro_z"],
                d["acc_x"], d["acc_y"], d["acc_z"],
                d["mag_x"], d["mag_y"], d["mag_z"]
        );
        ahrs->get_orientation(quat_out);
        std::vector<float> quat_vec(quat_out, quat_out + sizeof(quat_out) / sizeof(quat_out[0]));
        quat.emplace_back(quat_vec);
    }
}

void DataProcessing::save_result() {
    filesystem::path result_path = std::regex_replace(file_path, std::regex("_txt"), "_result/" + method);
    if (!filesystem::exists(result_path.parent_path())) {
        filesystem::create_directories(result_path.parent_path());
    }
    SaveFileUtils saveFileUtils(result_path, true);
    for (auto &it: header) {
        saveFileUtils.writeItem(it, ",");
    }
    saveFileUtils.writeItem(std::string("qw"), ",");
    saveFileUtils.writeItem(std::string("qx"), ",");
    saveFileUtils.writeItem(std::string("qy"), ",");
    saveFileUtils.writeItem(std::string("qz"), "\n");
    for (auto i = 0; i < data.size(); ++i) {
        for (auto &h: header) {
            saveFileUtils.writeItem(data[i][h], ",");
        }
        saveFileUtils.writeItem(quat[i][0], ",");
        saveFileUtils.writeItem(quat[i][1], ",");
        saveFileUtils.writeItem(quat[i][2], ",");
        saveFileUtils.writeItem(quat[i][3], "\n");
    }
    saveFileUtils.save();
    saveFileUtils.close();
}
