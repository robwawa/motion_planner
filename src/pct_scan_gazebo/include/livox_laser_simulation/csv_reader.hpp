//
// Created by lfc on 2021/3/1.
//

#ifndef SRC_GAZEBO_CSV_READER_HPP
#define SRC_GAZEBO_CSV_READER_HPP

#include <fstream>
#include <motion_planner_log/logging.h>
#include <sstream>
#include <string>
#include <vector>

class CsvReader {
 public:
    static bool ReadCsvFile(std::string file_name, std::vector<std::vector<double>>& datas) {
        std::fstream file_stream;
        file_stream.open(file_name, std::ios::in);
        if (file_stream.is_open()) {
            std::string header;
            std::getline(file_stream, header, '\n');
            while (!file_stream.eof()) {
                std::string line_str;
                std::getline(file_stream, line_str, '\n');
                std::stringstream line_stream;
                line_stream << line_str;
                std::vector<double> data;
                try {
                    while (!line_stream.eof()) {
                        std::string value;
                        std::getline(line_stream, value, ',');
                        data.push_back(std::stod(value));
                    }
                } catch (...) {
                    MOTION_PLANNER_LOG_ERROR_STREAM("cannot convert str:" << line_str);
                    continue;
                }
                datas.push_back(data);
            }
            MOTION_PLANNER_LOG_DEBUG_STREAM("Livox CSV rows loaded: count=" << datas.size());
            return true;
        } else {
            MOTION_PLANNER_LOG_ERROR_STREAM("Cannot read CSV file: path=" << file_name);
        }
        return false;
    }
};

#endif  // SRC_GAZEBO_CSV_READER_HPP
