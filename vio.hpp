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
        void predict(IMUSample& data, double dt); // IMU preintegration
        void update(uint64_t timestep); // replace with VIO EKF
        void outputState(string filename); // properly transform all states

        void init_params();
        void update_covariance(IMUSample& data, double dt);
        ~VisualInertialOdometry()
        {
            optimization_log.close();
        }
        
        vector<stateEstimate> state_estimate;
        stateEstimate last_estimate;
        Eigen::Vector3d g_ref; // gravity vector at initial orientation
    private:
        Eigen::Vector<double, 15> xr; // error state vector (nominal state vector held in this->state_estimate)
        Eigen::Matrix<double, 15, 15> P; // covariance matrix
        Eigen::Matrix<double, 15, 15> A; // state transition matrix (Jacobian)
        Eigen::Matrix<double, 15, 15> Q; // process noise covariance matrix
        Eigen::Matrix<double, 2, 2> R; // measurement noise covariance matrix
        Eigen::Matrix<double, 2, 15> H; // observation matrix (Jacobian) 
};

#endif