#include "dataset_config.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <limits>
#include <stdexcept>

namespace
{
    vector<string> split(const string& str, char delimiter)
    {
        vector<string> result;
        stringstream ss(str);
        string token;
        while (getline(ss, token, delimiter))
            result.push_back(token);
        return result;
    }

    string readFile(const string& path)
    {
        ifstream file(path);
        if (!file.is_open())
            throw std::runtime_error("Failed to open file: " + path);
        stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }

    // Pulls the flat list of numbers out of a "key: [n1, n2, ...]" line, or a
    // "key:\n  ...\n  data: [...]" block (T_BS) - both just need the first "[...]" after key
    vector<double> extractNumberList(const string& file_contents, const string& key)
    {
        size_t key_pos = file_contents.find(key);
        if (key_pos == string::npos)
            throw std::runtime_error("Missing key '" + key + "' in sensor.yaml");

        size_t bracket_start = file_contents.find('[', key_pos);
        size_t bracket_end = file_contents.find(']', bracket_start);

        vector<double> values;
        for (const auto& token : split(file_contents.substr(bracket_start + 1, bracket_end - bracket_start - 1), ','))
            values.push_back(stod(token));
        return values;
    }

    // Reads the "T_BS" (sensor -> body) 4x4 extrinsic out of a sensor.yaml
    void loadExtrinsic(const string& contents, cv::Mat& R_B_S, cv::Mat& t_B_S)
    {
        vector<double> T_BS = extractNumberList(contents, "data:"); // row-major 4x4
        R_B_S = (cv::Mat_<double>(3, 3) << T_BS[0], T_BS[1], T_BS[2],
                                            T_BS[4], T_BS[5], T_BS[6],
                                            T_BS[8], T_BS[9], T_BS[10]);
        t_B_S = (cv::Mat_<double>(3, 1) << T_BS[3], T_BS[7], T_BS[11]);
    }

    void loadCameraYaml(const string& path, cv::Mat& K, cv::Mat& D, cv::Size& resolution, cv::Mat& R_B_S, cv::Mat& t_B_S)
    {
        string contents = readFile(path);

        vector<double> intrinsics = extractNumberList(contents, "intrinsics:"); // fu, fv, cu, cv
        K = (cv::Mat_<double>(3, 3) << intrinsics[0], 0, intrinsics[2],
                                        0, intrinsics[1], intrinsics[3],
                                        0, 0, 1);

        vector<double> dist = extractNumberList(contents, "distortion_coefficients:");
        D = cv::Mat(1, static_cast<int>(dist.size()), CV_64F);
        for (size_t i = 0; i < dist.size(); i++)
            D.at<double>(0, static_cast<int>(i)) = dist[i];

        vector<double> res = extractNumberList(contents, "resolution:");
        resolution = cv::Size(static_cast<int>(res[0]), static_cast<int>(res[1]));

        loadExtrinsic(contents, R_B_S, t_B_S);
    }

    // Finds the data.csv row whose timestamp (column 0) is closest to target_timestamp.
    // Rows are time-ordered, so the distance to target is unimodal - stop as soon as it grows.
    vector<string> closestRow(const string& csv_path, uint64_t target_timestamp)
    {
        ifstream file(csv_path);
        if (!file.is_open())
            throw std::runtime_error("Failed to open file: " + csv_path);

        string line;
        getline(file, line); // skip header

        vector<string> best_row;
        uint64_t best_diff = std::numeric_limits<uint64_t>::max();
        while (getline(file, line))
        {
            vector<string> row = split(line, ',');
            uint64_t timestamp = stoull(row[0]);
            uint64_t diff = timestamp > target_timestamp ? timestamp - target_timestamp : target_timestamp - timestamp;
            if (diff > best_diff)
                break;

            best_diff = diff;
            best_row = row;
        }
        return best_row;
    }
}

DatasetConfig loadDatasetConfig(const string& dataset_root, uint64_t first_cam_timestamp)
{
    DatasetConfig config;
    string mav0 = dataset_root + "/mav0";

    loadCameraYaml(mav0 + "/cam0/sensor.yaml", config.K_left, config.D_left, config.image_size, config.R_B_C0, config.t_B_C0);

    cv::Size cam1_resolution; // cam1 shares cam0's resolution, only needed to satisfy the shared loader
    loadCameraYaml(mav0 + "/cam1/sensor.yaml", config.K_right, config.D_right, cam1_resolution, config.R_B_C1, config.t_B_C1);

    loadExtrinsic(readFile(mav0 + "/vicon0/sensor.yaml"), config.R_B_V, config.t_B_V);

    vector<string> vicon_row = closestRow(mav0 + "/vicon0/data.csv", first_cam_timestamp);
    config.t_vicon0 = Eigen::Vector3d(stod(vicon_row[1]), stod(vicon_row[2]), stod(vicon_row[3]));
    config.q_vicon0 = Eigen::Quaterniond(stod(vicon_row[4]), stod(vicon_row[5]), stod(vicon_row[6]), stod(vicon_row[7]));

    vector<string> gt_row = closestRow(mav0 + "/state_groundtruth_estimate0/data.csv", first_cam_timestamp);
    config.t_gt_0 = Eigen::Vector3d(stod(gt_row[1]), stod(gt_row[2]), stod(gt_row[3]));
    config.q_gt0 = Eigen::Quaterniond(stod(gt_row[4]), stod(gt_row[5]), stod(gt_row[6]), stod(gt_row[7]));
    config.init_accel_bias = Eigen::Vector3d(stod(gt_row[14]), stod(gt_row[15]), stod(gt_row[16]));

    return config;
}
