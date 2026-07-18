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
        Eigen::Quaterniond q_gt_0{0.060514, -0.828459, -0.058956, -0.553641}; // use {} for init in header
        Chebyshev2Filter filter_gyro_x;
        Chebyshev2Filter filter_gyro_y;
        Chebyshev2Filter filter_gyro_z;

        Chebyshev2Filter filter_accel_x;
        Chebyshev2Filter filter_accel_y;
        Chebyshev2Filter filter_accel_z;

        // Transformation body -> vicon (body relative to vicon)
        Eigen::Matrix3d R_V_B_eigen = (Eigen::Matrix3d() <<
            0.33638, -0.02078,  0.94150,
            -0.01749, -0.99972, -0.01582,
            0.94156, -0.01114, -0.33665).finished();

        Eigen::Vector3d t_V_B_eigen{0.092907, -0.028556, -0.107015};

        // Forward T_BS (vicon marker -> body), recovered by inverting R_V_B/t_V_B
        Eigen::Matrix3d R_B_V = R_V_B_eigen.transpose();
        Eigen::Vector3d t_B_V = -R_B_V * t_V_B_eigen;
        Eigen::Quaterniond q_V_B{R_V_B_eigen};
        Eigen::Quaterniond q_B_V{R_B_V};

        // Ground truth vicon marker pose at t=0 (vicon0/data.csv, timestamp 1403715273265220000)
        // Maps vicon marker -> room (ground truth)
        Eigen::Quaterniond q_gt0{0.993271232, -0.008876672, 0.022388759, 0.113279336};
        Eigen::Vector3d t_gt0{0.786659422, 2.176692611, 1.061934815};
    
    public:
        Eigen::Vector3d accel_bias;
        Eigen::Vector3d gyro_bias;
        std::vector<Eigen::Quaterniond> orient_estimate;
        std::vector<Eigen::Vector3d> euler_estimate;
        std::vector<Eigen::Vector3d> euler_correct;
        std::vector<Eigen::Vector3d> pos_estimate;
        std::vector<Eigen::Vector3d> vel_estimate;
        std::vector<uint64_t> timestamp;

        virtual void ahrs_estimate() = 0; // set virtual functions to 0, destructor/constructor to defualt
        // ahrs_estimate will calculate current orientation estimate, append to ahrs_estimate
        virtual ~BASE_AHRS() = default; // abstract class has one or more virtual functions

        void CHEBY_LP_FILT(IMUSample& data);
        void vec_error(const std::string& filename, const std::vector<Eigen::Vector3d>& euler1,
                            const std::vector<Eigen::Vector3d>& euler2);
        void outputPose(const std::string& filename, const std::vector<uint64_t>& timestamp,
                            const std::vector<Eigen::Vector3d>& pos_estimate, const std::vector<Eigen::Vector3d>& orient_estimate);
        double get_shortest_angle_error(double estimate, double ground_truth);
        void quat_error(const std::string& filename, const std::vector<Eigen::Quaterniond>& gt_trajectory);
        Eigen::Quaterniond init_orient(size_t num_samples = 200);
        void estimate_position();

        BASE_AHRS(const std::vector<IMUSample>&data) : imu_data(data), 
                    accel_bias(Eigen::Vector3d::Zero()), gyro_bias(Eigen::Vector3d::Zero()) 
        {
            this->orient_estimate.clear();
            this->orient_estimate.reserve(this->imu_data.size());

            this->euler_estimate.clear();
            this->euler_estimate.reserve(this->imu_data.size());
        }
        void align_estimate();
        // actual gravity vector at t=0 since IMU measures normal reaction
        Eigen::Vector3d g_ref; // negative of average accel data, when drone is upright stationary (|g_ref| = 9.81), gravity vector relative to true vertical
        Eigen::Quaterniond R_align; // rotation between true vertical and intial gravity vector
};

class DEAD_RECKONING_AHRS : public BASE_AHRS
{
    public:
        // constructor is not auto inherited from base so you have to define it yourself
        DEAD_RECKONING_AHRS(const std::vector<IMUSample>& imu_data) : BASE_AHRS(imu_data) {}
        void ahrs_estimate() override;
};

class COMPLEMENTARY_AHRS : public BASE_AHRS
{
    public:
        COMPLEMENTARY_AHRS(const std::vector<IMUSample>& imu_data) : BASE_AHRS(imu_data) {}
        void ahrs_estimate() override;
        void COMPLEMENTARY_FILT(IMUSample& data, Eigen::Quaterniond& q);
};

class MAHONY_AHRS : public BASE_AHRS
{
    public:
        MAHONY_AHRS(const std::vector<IMUSample>& imu_data) : BASE_AHRS(imu_data) {}
        void ahrs_estimate() override;
        void MAHONY_FILT(IMUSample& data, double dt);

    private:
        double kp = 0.05;
        double ki = 0.0025;
        Eigen::Vector3d error = Eigen::Vector3d::Zero();
};

class MADGWICK_AHRS : public BASE_AHRS
{
    public:
        MADGWICK_AHRS(const std::vector<IMUSample>& imu_data) : BASE_AHRS(imu_data) {}
        void ahrs_estimate() override;
        void MADGWICK_FILT(IMUSample& data, Eigen::Quaterniond& q, double step_size);

    private:
        double alpha = 15.0;
};

class EKF_AHRS : public BASE_AHRS
{
    public:
        EKF_AHRS(const std::vector<IMUSample>& imu_data) : BASE_AHRS(imu_data) {}
        void ahrs_estimate() override;
        void init_params();
        void predict(IMUSample& data, double dt);
        void correct(IMUSample& data);
        std::vector<Eigen::Vector3d> gyro_bias_estimate;
        
    private:
        Eigen::Vector<double, 7> x; // state vector
        Eigen::Matrix<double, 7, 7> P; // covariance matrix
        Eigen::Matrix<double, 7, 7> Q; // process noise covariance matrix
        Eigen::Matrix<double, 7, 7> A; // state transition matrix (Jacobian)
        Eigen::Matrix<double, 3, 7> H; // observation matrix (jacobian)
        Eigen::Matrix<double, 3, 3> R; // measurement noise covariance

        const double sigma_gyro_sq = 2.8791e-8; // (1.6968e-4)^2
        const double sigma_bias_sq = 3.7609e-10; // (1.9393e-5)^2
        const double sigma_accel_sq = 4.0e-6; // (2.0e-3)^2
        const double dt = 1 / 200.0;
};

#endif