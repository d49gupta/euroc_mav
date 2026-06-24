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
        Eigen::Quaterniond q_align;
        Eigen::Quaterniond q_gt_0{0.060514, -0.828459, -0.058956, -0.553641}; // use {} for init in header
        Chebyshev2Filter filter_gyro_x;
        Chebyshev2Filter filter_gyro_y;
        Chebyshev2Filter filter_gyro_z;

        Chebyshev2Filter filter_accel_x;
        Chebyshev2Filter filter_accel_y;
        Chebyshev2Filter filter_accel_z;
    
    public:
        Eigen::Vector3d accel_bias;
        Eigen::Vector3d gyro_bias;
        std::vector<Eigen::Quaterniond> orient_estimate;
        std::vector<Eigen::Vector3d> euler_estimate;

        virtual void ahrs_estimate() = 0; // set virtual functions to 0, destructor/constructor to defualt
        // ahrs_estimate will calculate current orientation estimate, append to ahrs_estimate
        virtual ~BASE_AHRS() = default; // abstract class has one or more virtual functions

        void CHEBY_LP_FILT(IMUSample& data);
        void euler_error(const std::string& filename, const std::vector<Eigen::Vector3d>& euler1,
                            const std::vector<Eigen::Vector3d>& euler2);
        double get_shortest_angle_error(double estimate, double ground_truth);
        void quat_error(const std::string& filename, const std::vector<Eigen::Quaterniond>& gt_trajectory);
        Eigen::Quaterniond init_orient(size_t num_samples = 200);

        BASE_AHRS(const std::vector<IMUSample>&data) : imu_data(data), 
                    accel_bias(Eigen::Vector3d::Zero()), gyro_bias(Eigen::Vector3d::Zero()) 
        {
            this->orient_estimate.clear();
            this->orient_estimate.reserve(this->imu_data.size());

            this->euler_estimate.clear();
            this->euler_estimate.reserve(this->imu_data.size());
        }
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
        void MADGWICK_FILT(IMUSample& data, Eigen::Quaterniond& q_madgwick, double step_size);

    private:
        double alpha = 10.0;
};

#endif