#include "vio.hpp"

void VisualInertialOdometry::preintegrate(IMUSample& data, uint64_t dt)
{
    stateEstimate last_estimate = this->state_estimate.back();
    stateEstimate curr_estimate;
    Eigen::Quaterniond q_last = last_estimate.orient_estimate;

    double wx = data.gyro.x() - last_estimate.gyro_bias.x();
    double wy = data.gyro.y() - last_estimate.gyro_bias.y();
    double wz = data.gyro.z() - last_estimate.gyro_bias.z();

    Eigen::Quaterniond q_curr;
    q_curr.w() = q_last.w() + dt * 0.5 * (-q_last.x() * wx - q_last.y() * wy - q_last.z() * wz);
    q_curr.x() = q_last.x() + dt * 0.5 * ( q_last.w() * wx + q_last.y() * wz - q_last.z() * wy);
    q_curr.y() = q_last.y() + dt * 0.5 * ( q_last.w() * wy - q_last.x() * wz + q_last.z() * wx);
    q_curr.z() = q_last.z() + dt * 0.5 * ( q_last.w() * wz + q_last.x() * wy - q_last.y() * wx);
    q_curr.normalize();
    curr_estimate.orient_estimate = q_curr;

    Eigen::Vector3d unbiased_accel = data.accel - last_estimate.accel_bias;
    Eigen::Vector3d world_accel = q_last * unbiased_accel;

    Eigen::Vector3d last_pos = last_estimate.pos_estimate;
    Eigen::Vector3d last_vel = last_estimate.vel_estimate;

    curr_estimate.pos_estimate = last_pos + dt * last_vel + 0.5 * dt * dt * (this->g_ref + world_accel);
    curr_estimate.vel_estimate = last_vel + dt * (this->g_ref + world_accel);

    // assume gyro and accel bias remain constant through prediction
    curr_estimate.gyro_bias = last_estimate.gyro_bias;
    curr_estimate.accel_bias = last_estimate.accel_bias;
    this->state_estimate.push_back(curr_estimate);
}

void VisualInertialOdometry::getPose(uint64_t timestep)
{
    // TODO
}