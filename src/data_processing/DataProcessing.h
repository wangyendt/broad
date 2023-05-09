//
// Created by wayne on 2023/5/9.
//

#ifndef AHRS_LIB_DATAPROCESSING_H
#define AHRS_LIB_DATAPROCESSING_H

#include "string"
#include "vector"
#include "unordered_map"

class DataProcessing {
public:
    explicit DataProcessing(std::string &file_path, std::string &method, std::string &n_axis);

    void load_data();

    void process_data();

    void save_result();

private:
    std::string file_path;
    std::string method;
    std::string n_axis;
    std::vector<std::string> header;
    std::vector<std::unordered_map<std::string, float>> data;
    std::vector<std::vector<float>> quat;
};


#endif //AHRS_LIB_DATAPROCESSING_H
