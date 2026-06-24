#ifndef BASE_AHRS_HPP
#define BASE_AHRS_HPP

#include "csv_reader.hpp"
#include "math_utils.hpp"
#include <fstream>
#include <iomanip>
#include <string>
#include "filters.hpp"

class BASE_AHRS
{
    protected:
        std::vector<IMUSample> imu_data;
        Eigen::Vector3d accel_bias;
        Eigen::Vector3d gyro_bias;
        std::vector<Eigen::Quaterniond> orient_estimate;
        Eigen::Quaterniond q_align;
        Eigen::Quaterniond q_gt_0{0.060514, -0.828459, -0.058956, -0.553641}; // use {} for init in header
    
    public:
        std::vector<Eigen::Vector3d> euler_estimate;


        BASE_AHRS(const std::vector<IMUSample>&data) : imu_data(data), 
                    accel_bias(Eigen::Vector3d::Zero()), gyro_bias(Eigen::Vector3d::Zero()) 
        {
            this->orient_estimate.clear();
            this->orient_estimate.reserve(this->imu_data.size());
        }

        Eigen::Quaterniond init_orient(size_t num_samples = 200)
        {
            if (this->imu_data.size() < num_samples)
            {
                std::cerr << "Warning: Not enough IMU data for calibration window!" << std::endl;
                return Eigen::Quaterniond::Identity();
            }

            Eigen::Vector3d sum_gyro = Eigen::Vector3d::Zero();
            Eigen::Vector3d sum_accel = Eigen::Vector3d::Zero();
            for (size_t i = 0; i < num_samples; i++)
            {
                sum_gyro += this->imu_data[i].gyro;
                sum_accel += this->imu_data[i].accel;
            }
            
            this->gyro_bias = sum_gyro / static_cast<double>(num_samples);
            Eigen::Vector3d mean_accel = sum_accel / static_cast<double>(num_samples);

            double roll = atan2(mean_accel.y(), mean_accel.z());
            double pitch = atan2(-mean_accel.x(), sqrt(mean_accel.y()*mean_accel.y() + mean_accel.z()*mean_accel.z()));
            double yaw = 0.0;

            Eigen::Vector3d init_euler(roll, pitch, yaw);
            Eigen::Quaterniond init_quat;
            eulers_to_quat(init_euler, init_quat);
            // std::cout<<init_quat.w()<<" "<<init_quat.x()<<" "<<init_quat.y()<<" "<<init_quat.z()<<std::endl;

            this->q_align = this->q_gt_0 * init_quat.conjugate();
            Eigen::Quaterniond q_init_align = this->q_align * init_quat;
            q_init_align.normalize();

            // std::cout<<q_init_align.w()<<" "<<q_init_align.x()<<" "<<q_init_align.y()<<" "<<q_init_align.z()<<std::endl;
            return q_init_align;
        }

        virtual void ahrs_estimate() = 0; // set virtual functions to 0, destructor/constructor to defualt
        // ahrs_estimate will calculate current orientation estimate, append to ahrs_estimate

        void calculate_error(const std::string& filename, const std::vector<Eigen::Quaterniond>& gt_trajectory) 
        {
            std::ofstream csv_file(filename);

            if (!csv_file.is_open()) 
            {
                std::cerr << "Error: Could not open file " << filename << " for writing!" << std::endl;
                return;
            }

            csv_file << "est_qx,est_qy,est_qz,est_qw,angular_error_deg\n";
            csv_file << std::fixed << std::setprecision(6);

            size_t loop_size = std::min(this->orient_estimate.size(), gt_trajectory.size());

            for (size_t i = 0; i < loop_size; ++i) {
                const auto& q_est = this->orient_estimate[i];
                const auto& q_gt = gt_trajectory[i];

                double dot_product = q_est.w() * q_gt.w() + 
                                    q_est.x() * q_gt.x() + 
                                    q_est.y() * q_gt.y() + 
                                    q_est.z() * q_gt.z();

                dot_product = std::max(-1.0, std::min(1.0, dot_product));
                double angle_error_rad = 2.0 * std::acos(std::abs(dot_product));
                double angle_error_deg = angle_error_rad * (180.0 / M_PI);

                csv_file << q_est.x() << ","
                        << q_est.y() << ","
                        << q_est.z() << ","
                        << q_est.w() << ","
                        << angle_error_deg << "\n";
            }
        }

        double get_shortest_angle_error(double estimate, double ground_truth) 
        {
            double diff = estimate - ground_truth;
            double tmp = std::atan2(std::sin(diff), std::cos(diff)); // wrap angle between +- pi by finding shortest path
            // between two headings in circle
            return tmp * (180.0 / M_PI);
        }

        void euler_error(const std::string& filename, const std::vector<Eigen::Vector3d>& euler1,
                                    const std::vector<Eigen::Vector3d>& euler2) 
        {
            std::ofstream csv_file(filename);

            if (!csv_file.is_open()) 
            {
                std::cerr << "Error: Could not open file " << filename << " for writing!" << std::endl;
                return;
            }

            csv_file << "roll,pitch,yaw\n";
            csv_file << std::fixed << std::setprecision(6);

            size_t loop_size = std::min(euler1.size(), euler2.size());

            for (size_t i = 0; i < loop_size; ++i) 
            {
                Eigen::Vector3d orient1 = euler1[i];
                Eigen::Vector3d orient2 = euler2[i];

                double roll_diff = get_shortest_angle_error(orient1.x(), orient2.x());
                double pitch_diff = get_shortest_angle_error(orient1.y(), orient2.y());
                double yaw_diff = get_shortest_angle_error(orient1.z(), orient2.z());

                csv_file << roll_diff << ","
                        << pitch_diff << ","
                        << yaw_diff << "\n";
            }

            csv_file.close();
        }

        virtual ~BASE_AHRS() = default; // abstract class has one or more virtual functions
};

class DEAD_RECKONING_AHRS : public BASE_AHRS
{
    public:
        using BASE_AHRS::BASE_AHRS;
        void ahrs_estimate() override;

    private:
};

class MAHONY_AHRS : public BASE_AHRS
{
    public:
        using BASE_AHRS::BASE_AHRS;
        void ahrs_estimate() override;
        void CHEBY_LP_FILT(IMUSample& data);
        void COMPLEMENTARY_FILT(IMUSample& data, Eigen::Quaterniond& q);
        void GYRO_CORRECT(IMUSample& data, double dt);

    private:
        Chebyshev2Filter filter_gyro_x;
        Chebyshev2Filter filter_gyro_y;
        Chebyshev2Filter filter_gyro_z;

        Chebyshev2Filter filter_accel_x;
        Chebyshev2Filter filter_accel_y;
        Chebyshev2Filter filter_accel_z;
};


#endif