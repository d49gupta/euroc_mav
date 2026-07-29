#ifndef VISUAL_INERTIAL_HPP
#define VISUAL_INERTIAL_HPP

#include "vo.hpp"
#include "ahrs.hpp"

using namespace std;
using namespace cv;

struct stateEstimate
{
    uint64_t timestamp = 0.0;
    Eigen::Vector3d accel_bias{0.0, 0.0, 0.0};
    Eigen::Quaterniond orient_estimate = Eigen::Quaterniond::Identity();
    Eigen::Vector3d gyro_bias{0.0, 0.0, 0.0};
    Eigen::Vector3d pos_estimate{0.0, 0.0, 0.0};
    Eigen::Vector3d vel_estimate{0.0, 0.0, 0.0};
};

class VisualInertialOdometry : public StereoTracker
{
    public:
        VisualInertialOdometry(const string nodeName) : StereoTracker(nodeName) {}
        void getPose(uint64_t timestep) override; // replace with VIO EKF
        void preintegrate(IMUSample& data, uint64_t dt); // IMU preintegration
        vector<stateEstimate> state_estimate;

        Eigen::Vector3d g_ref; // gravity vector at initial orientation
};

#endif