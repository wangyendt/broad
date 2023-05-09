//
// Created by wayne on 2022/11/9.
//

#ifndef SLAMDEMO_SAVEFILEUTILS_H
#define SLAMDEMO_SAVEFILEUTILS_H

#include <string>
#include <fstream>
#include <libgen.h>
#include <ctime>
#include <chrono>

namespace filesystem = std::__fs::filesystem;

class SaveFileUtils {
public:
    SaveFileUtils(std::string path, bool new_file) {
        if (filesystem::exists(path)) {
            if (filesystem::is_regular_file(path)) {
                if (new_file) {
                    filesystem::remove(path);
                }
                is_available = true;
            } else {
                is_available = false;
            }
        } else {
            auto cur_path = filesystem::path(path);
            auto parent_path = cur_path.parent_path();
            if (!filesystem::exists(parent_path)) {
                try {
                    filesystem::create_directories(parent_path);
                    is_available = true;
                    if (!filesystem::is_regular_file(path)) {
                        is_available = false;
                    }
                } catch (std::exception) {
                    is_available = false;
                }
            } else {
                is_available = true;
            }
        }
        if (is_available) {
            ofstream = std::ofstream(path, std::ios::app);
        }
    }

    template<typename T>
    void writeItem(T item, const std::string& delimiter = ",") {
        if (ofstream.is_open()) {
            ofstream << std::to_string(item) << delimiter;
        }
    }

    void writeItem(const std::string& item, const std::string& delimiter = ",") {
        if (ofstream.is_open()) {
            ofstream << item << delimiter;
        }
    }

    void writeLines(const std::string& line) {
        if (ofstream.is_open()) {
            ofstream << line << "\n";
        }
    }

    void save() {
        if (ofstream.is_open()) {
            ofstream.flush();
        }
    }

    void close() {
        if (ofstream.is_open()) {
            ofstream.close();
        }
    }

    static void makeDir(std::string path) {
        if (!filesystem::exists(path)) {
            filesystem::create_directories(path);
        }
    }

    static void clearDir(std::string dirName) {
        if (filesystem::exists(dirName) && filesystem::is_directory(dirName)) {
            for (auto file: filesystem::recursive_directory_iterator(dirName)) {
                if (filesystem::is_regular_file(file)) {
                    filesystem::remove(file);
                }
            }
        }
    }

    static void writeLinesStatic(std::string rootDirName, std::string item) {
        static bool init = false;
        static std::ofstream ofs;
        if (!init) {
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
            std::time_t timestamp = tp.time_since_epoch().count();
            filesystem::path fsSavePath = "/sdcard/Documents";
            fsSavePath /= rootDirName;
            fsSavePath /= "running_time";
            fsSavePath /= std::to_string(timestamp) + ".txt";
            filesystem::create_directories(fsSavePath.parent_path());
            ofs = std::ofstream(fsSavePath, std::ios::app);
            if (ofs.is_open()) {
                ofs << "timestamp,function,elapsed_time(ms)" << std::endl;
            }
            init = true;
        }
        if (ofs.is_open()) {
            ofs << item << std::endl;
            ofs.flush();
        }
    }

private:
    std::ofstream ofstream;
    bool is_available;
};

#endif //SLAMDEMO_SAVEFILEUTILS_H
