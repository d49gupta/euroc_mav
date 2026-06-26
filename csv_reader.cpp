#include "csv_reader.hpp"

// Function to split a string by a delimiter
vector<string> csvReader::split(const string &str, char delimiter) 
{
    vector<string> result;
    stringstream ss(str);
    string token;
    while (getline(ss, token, delimiter)) {
        result.push_back(token);
    }
    return result;
}

// Function to read IMU CSV and store the data
void IMUReader::readCSV() {
    ifstream file(this->filePath);

    if (!file.is_open()) {
        cerr << "Failed to open file: " << this->filePath << endl;
        return;
    }

    string line;
    bool first_row = true;
    // Read the file line by line
    while (getline(file, line)) {
        vector<string> row = split(line, ',');

        if (first_row) {
            this->headers = row;
            first_row = false;
        } 
        else {
            IMUSample sample;
            sample.timestamp = stod(row[0]);
            sample.gyro = Eigen::Vector3d(stod(row[1]), stod(row[2]), stod(row[3]));
            sample.accel = Eigen::Vector3d(stod(row[4]), stod(row[5]), stod(row[6]));
            data.push_back(sample);
        }
    }

    file.close();
}

// Function to print a row of IMU data
void IMUReader::printRow() 
{
    if (this->data.empty()) {
        cout << "No data to print." << endl;
        return;
    }

    std::cout<< "Timestamp: " << this->data[0].timestamp << ", "
             << "Gyro: [" << this->data[0].gyro.x() << ", " << this->data[0].gyro.y() << ", " << this->data[0].gyro.z() << "], "
             << "Accel: [" << this->data[0].accel.x() << ", " << this->data[0].accel.y() << ", " << this->data[0].accel.z() << "]"
             << std::endl;
}

// Function to print headers
void csvReader::printHeaders() 
{
    for (const auto &header : headers) {
        cout << header << " | ";
    }
    cout << endl;
}

void GTReader::readCSV() {
    ifstream file(this->filePath);

    if (!file.is_open()) {
        cerr << "Failed to open file: " << this->filePath << endl;
        return;
    }

    string line;
    bool first_row = true;
    // Read the file line by line
    while (getline(file, line)) {
        vector<string> row = split(line, ',');

        if (first_row) {
            this->headers = row;
            first_row = false;
        } 
        else {
            // sample.timestamp = stod(row[0]);
            // sample.pos = Eigen::Vector3d(stod(row[1]), stod(row[2]), stod(row[3]));
            Eigen::Quaterniond curr_orient = Eigen::Quaterniond(stod(row[4]), stod(row[5]), stod(row[6]), stod(row[7]));
            Eigen::Vector3d curr_euler = quat_to_eulers(curr_orient);
            // sample.vel = Eigen::Vector3d(stod(row[8]), stod(row[9]), stod(row[10]));
            Eigen::Vector3d curr_gyro_bias = Eigen::Vector3d(stod(row[11]), stod(row[12]), stod(row[13]));
            // sample.accel_bias = Eigen::Vector3d(stod(row[14]), stod(row[15]), stod(row[16]));
            gt_orient_data.push_back(curr_orient);
            gt_euler_data.push_back(curr_euler);
            gt_gyro_bias.push_back(curr_gyro_bias);
        }
    }

    file.close();
}

void GTReader::printRow() 
{
    if (this->gt_orient_data.empty()) {
        cout << "No data to print." << endl;
        return;
    }
}
