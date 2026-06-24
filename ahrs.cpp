#include "ahrs.hpp"
#include "math_utils.hpp"

void DEAD_RECKONING_AHRS::ahrs_estimate()
{
    Eigen::Quaterniond init_quat = init_orient();
    this->orient_estimate.push_back(init_quat);

    for (size_t i = 1; i < this->imu_data.size(); i++)
    {
        Eigen::Quaterniond q_curr;
        Eigen::Quaterniond q_last = this->orient_estimate.back();

        uint64_t t_now = this->imu_data[i].timestamp;
        uint64_t t_last = this->imu_data[i-1].timestamp;
        double dt = static_cast<double>(t_now - t_last) / 1e9;

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

void MAHONY_AHRS::ahrs_estimate()
{
    Eigen::Quaterniond init_quat = init_orient();
    this->orient_estimate.push_back(init_quat);

    for (size_t i = 1; i < this->imu_data.size(); i++)
    {
        Eigen::Quaterniond q_curr;
        Eigen::Quaterniond q_last = this->orient_estimate.back();

        uint64_t t_now = this->imu_data[i].timestamp;
        uint64_t t_last = this->imu_data[i-1].timestamp;
        double dt = static_cast<double>(t_now - t_last) / 1e9;

        this->CHEBY_LP_FILT(imu_data[i]);
        // this->GYRO_CORRECT(imu_data[i], dt);
        double wx = this->imu_data[i].gyro.x() - this->gyro_bias.x();
        double wy = this->imu_data[i].gyro.y() - this->gyro_bias.y();
        double wz = this->imu_data[i].gyro.z() - this->gyro_bias.z();

        q_curr.w() = q_last.w() + dt * 0.5 * (-q_last.x() * wx - q_last.y() * wy - q_last.z() * wz);
        q_curr.x() = q_last.x() + dt * 0.5 * ( q_last.w() * wx + q_last.y() * wz - q_last.z() * wy);
        q_curr.y() = q_last.y() + dt * 0.5 * ( q_last.w() * wy - q_last.x() * wz + q_last.z() * wx);
        q_curr.z() = q_last.z() + dt * 0.5 * ( q_last.w() * wz + q_last.x() * wy - q_last.y() * wx);
        q_curr.normalize();
        Eigen::Vector3d euler_curr = quat_to_eulers(q_curr);

        this->COMPLEMENTARY_FILT(imu_data[i], q_curr);
        this->orient_estimate.push_back(q_curr);
        this->euler_estimate.push_back(euler_curr);
    }

    return;
}

void MAHONY_AHRS::COMPLEMENTARY_FILT(IMUSample& data, Eigen::Quaterniond& q)
{
    double a_mag = data.accel.norm();
    if (abs(a_mag - GRAV) > 0.30)
        return;

    Eigen::Quaterniond q_accel = Eigen::Quaterniond::Identity();
    Eigen::Vector3d a_norm = data.accel.normalized();
    double roll_accel  = atan2(data.accel.y(), data.accel.z());
    double pitch_accel = atan2(-data.accel.x(), sqrt(data.accel.y()*data.accel.y() + data.accel.z()*data.accel.z()));
    double yaw_gyro = yaw_from_quat(q);
    Eigen::Vector3d euler_target(roll_accel, pitch_accel, yaw_gyro);

    Eigen::Quaterniond q_target = Eigen::Quaterniond::Identity();
    eulers_to_quat(euler_target, q_target);
    Eigen::Quaterniond q_target_align = this->q_align * q_target;
    q_target_align.normalize();

    q = q.slerp(0.01, q_target_align); 
    q.normalize();
    return;
}

void MAHONY_AHRS::CHEBY_LP_FILT(IMUSample& data)
{
    float clean_wx = filter_gyro_x.filter(data.gyro.x());
    float clean_wy = filter_gyro_y.filter(data.gyro.y());
    float clean_wz = filter_gyro_z.filter(data.gyro.z());

    float clean_ax = filter_accel_x.filter(data.accel.x());
    float clean_ay = filter_accel_y.filter(data.accel.y());
    float clean_az = filter_accel_z.filter(data.accel.z());

    data.gyro  = Eigen::Vector3d(clean_wx, clean_wy, clean_wz);
    data.accel = Eigen::Vector3d(clean_ax, clean_ay, clean_az);
}

void MAHONY_AHRS::MAHONY_FILT(IMUSample& data, Eigen::Quaterniond& q)
{
    double accel_mag = data.accel.norm();
    double confidence = clamp((1.0 - fabs(accel_mag - GRAV) / GRAV), 0.0, 1.0) * 1.0; 

    Eigen::Vector3d a_meas = data.accel.normalized();
    double roll = atan2(a_meas.y(), a_meas.z());
    double pitch = atan2(-a_meas.x(), sqrt(a_meas.y()*a_meas.y() + a_meas.z()*a_meas.z()));
    double yaw = yaw_from_quat(q);

    Eigen::Vector3d euler_accel(roll, pitch, yaw);
    Eigen::Quaterniond q_accel = Eigen::Quaterniond::Identity();
    eulers_to_quat(euler_accel, q_accel);
    SLERP_v4(q, q_accel, confidence);
    q.normalize();
    return;
}

void MAHONY_AHRS::GYRO_CORRECT(IMUSample& data, double dt)
{
    Eigen::Quaterniond q_last = this->orient_estimate.back();
    double vx = 2.0 * (q_last.x() * q_last.z() - q_last.w() * q_last.y());
    double vy = 2.0 * (q_last.y() * q_last.z() + q_last.w() * q_last.x());
    double vz = -2.0 * (q_last.x() * q_last.x() + q_last.y() * q_last.y()) + 1;
    Eigen::Vector3d v_est(vx, vy, vz); // Estimated gravity vector

    Eigen::Vector3d a_meas = data.accel.normalized();
    Eigen::Vector3d error = a_meas.cross(v_est);

    double Kp = 0.25;
    double Ki = 0.05;

    double accel_mag = data.accel.norm();
    double confidence = clamp((1.0 - fabs(accel_mag - GRAV) / GRAV), 0.0, 1.0);

    error *= confidence;
    this->gyro_bias += Ki * error * dt; // this growing unbounded
    Eigen::Vector3d w_raw(data.gyro.x(), data.gyro.y(), data.gyro.z());
    data.gyro = w_raw + (Kp * error);
    return;
}