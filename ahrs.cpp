#include "ahrs.hpp"

void DEAD_RECKONING_AHRS::ahrs_estimate()
{
    Eigen::Quaterniond init_quat = init_orient();
    this->orient_estimate.push_back(init_quat);

    for (size_t i = 1; i < this->imu_data.size(); i++)
    {
        Eigen::Quaterniond q_curr;
        Eigen::Quaterniond q_last = this->orient_estimate.back();

        double dt = this->imu_data[i].timestamp - this->imu_data[i-1].timestamp;

        double wx = this->imu_data[i].gyro.x();
        double wy = this->imu_data[i].gyro.y();
        double wz = this->imu_data[i].gyro.z();

        q_curr.w() = q_last.w() + dt * 0.5 * (-q_last.x() * wx - q_last.y() * wy - q_last.z() * wz);
        q_curr.x() = q_last.x() + dt * 0.5 * ( q_last.w() * wx + q_last.y() * wz - q_last.z() * wy);
        q_curr.y() = q_last.y() + dt * 0.5 * ( q_last.w() * wy - q_last.x() * wz + q_last.z() * wx);
        q_curr.z() = q_last.z() + dt * 0.5 * ( q_last.w() * wz + q_last.x() * wy - q_last.y() * wx);

        q_curr.normalize();
        this->orient_estimate.push_back(q_curr);
    }
    
    return;
}