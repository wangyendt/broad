//
// Created by wayne on 2023/5/9.
//

#include "DataProcessing.h"
#include "iostream"

int main(int argc, char **argv) {
    std::string path, algorithm, n_axis;
    if (argc == 4) {
        path = argv[1]; // "/Users/wayne/Documents/work/code/python/broad/data_txt/01_undisturbed_slow_rotation_A.txt";
        algorithm = argv[2]; // "madgwick";
        n_axis = argv[3];
    } else {
        path = "/Users/wayne/Documents/work/code/python/broad/data_txt/01_undisturbed_slow_rotation_A.txt";
        algorithm = "vqf";
        n_axis = "6";
    }
    DataProcessing dataProcessing(path, algorithm, n_axis);
    dataProcessing.load_data();
    dataProcessing.process_data();
    dataProcessing.save_result();
    std::cout << "complete:\t[" << algorithm << "]\t" << path << std::endl;
    return 0;
}
