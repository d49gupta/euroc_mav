#ifndef CSV_READER_HPP
#define CSV_READER_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "math_utils.hpp"

using namespace std;

struct IMUSample
{
    double timestamp;
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
};

class csvReader 
{
    public:
        csvReader(string filePath) : filePath(filePath) {}
        vector<string> split(const string &str, char delimiter);
        virtual void readCSV() = 0;
        virtual void printRow() = 0;
        void printHeaders();
        vector<string> headers;

    protected:
        string filePath;
};

class IMUReader : public csvReader
{
    public:
        using csvReader::csvReader;
        void readCSV() override;
        void printRow() override;

        vector<IMUSample> data;
};

class GTReader : public csvReader
{
    public:
        using csvReader::csvReader;
        void readCSV() override;
        void printRow() override;

        vector<Eigen::Quaterniond> gt_orient_data;
        vector<Eigen::Vector3d> gt_euler_data;
};
#endif