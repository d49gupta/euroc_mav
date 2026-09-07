#include "ahrs.hpp"
#include "math_utils.hpp"

Eigen::Quaterniond BASE_AHRS::init_orient(size_t num_samples)
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
    this->g_ref = -mean_accel;

    // orientation of drone at t=0 relative to true vertical
    double roll = atan2(mean_accel.y(), mean_accel.z());
    double pitch = atan2(-mean_accel.x(), sqrt(mean_accel.y()*mean_accel.y() + mean_accel.z()*mean_accel.z()));
    double yaw = 0.0;

    Eigen::Vector3d init_euler(roll, pitch, yaw);
    Eigen::Quaterniond init_quat;
    eulers_to_quat(init_euler, init_quat);
    this->orient_estimate.push_back(init_quat);

    return init_quat;
}

void BASE_AHRS::estimate_position()
{
    Eigen::Vector3d init_pos{0.0, 0.0, 0.0};
    this->pos_estimate.push_back(init_pos);
    this->vel_estimate.push_back(init_pos);
    this->timestamp.push_back(this->imu_data[0].timestamp);

    for (size_t i = 1; i < this->imu_data.size(); i++)
    {
        uint64_t t_now = this->imu_data[i].timestamp;
        uint64_t t_last = this->imu_data[i-1].timestamp;
        double dt = static_cast<double>(t_now - t_last) / 1e9;
        this->timestamp.push_back(t_now);
        Eigen::Vector3d accel_ref = this->orient_estimate[i] * this->imu_data[i].accel + g_ref; // account for gravity

        Eigen::Vector3d curr_vel = vel_estimate.back() + accel_ref * dt;
        Eigen::Vector3d curr_pos = pos_estimate.back() + vel_estimate.back() * dt + 0.5 * accel_ref * dt * dt; // Taylor series for position and velocity calculations
        this->vel_estimate.push_back(curr_vel);
        this->pos_estimate.push_back(curr_pos);
    }
}

void BASE_AHRS::quat_output(const std::string& filename, const std::vector<uint64_t>& timestamp, const std::vector<Eigen::Quaterniond>& gt_trajectory) 
{
    std::ofstream csv_file(filename);

    if (!csv_file.is_open()) 
    {
        std::cerr << "Error: Could not open file " << filename << " for writing!" << std::endl;
        return;
    }

    csv_file << "timestamp,est_qw,est_qx,est_qy,est_qz\n";
    csv_file << std::fixed << std::setprecision(6);

    size_t loop_size = std::min(timestamp.size(), gt_trajectory.size());

    for (size_t i = 0; i < loop_size; ++i) 
    {
        const uint64_t time = timestamp[i];
        const auto& q_est = gt_trajectory[i];

        csv_file << time << ","
                << q_est.w() << ","
                << q_est.x() << ","
                << q_est.y() << ","
                << q_est.z() << "\n";
    }
}

void BASE_AHRS::quat_error(const std::string& filename, const std::vector<Eigen::Quaterniond>& gt_trajectory) 
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

double BASE_AHRS::get_shortest_angle_error(double estimate, double ground_truth) 
{
    double diff = estimate - ground_truth;
    double tmp = std::atan2(std::sin(diff), std::cos(diff)); // wrap angle between +- pi by finding shortest path
    // between two headings in circle
    return tmp * (180.0 / M_PI);
}

void BASE_AHRS::vec_error(const std::string& filename, const std::vector<Eigen::Vector3d>& euler1,
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

void BASE_AHRS::outputPose(const std::string& filename, const std::vector<uint64_t>& timestamp,
                            const std::vector<Eigen::Vector3d>& pos_estimate, const std::vector<Eigen::Vector3d>& orient_estimate) 
{
    std::ofstream csv_file(filename);

    if (!csv_file.is_open()) 
    {
        std::cerr << "Error: Could not open file " << filename << " for writing!" << std::endl;
        return;
    }

    csv_file << "timestamp,pos_x,pos_y,pos_z,roll,pitch,yaw\n";
    csv_file << std::fixed << std::setprecision(6);

    size_t loop_size = std::min({timestamp.size(), pos_estimate.size(), orient_estimate.size()});

    for (size_t i = 0; i < loop_size; ++i) 
    {
        Eigen::Vector3d pos = pos_estimate[i];
        Eigen::Vector3d orient = orient_estimate[i];
        uint64_t time = timestamp[i];

        csv_file << time << ","
                << pos.x() << ","
                << pos.y() << ","
                << pos.z() << ","
                << orient.x() << ","
                << orient.y() << ","
                << orient.z() << "\n";
    }

    csv_file.close();
}

void BASE_AHRS::CHEBY_LP_FILT(IMUSample& data)
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

void DEAD_RECKONING_AHRS::ahrs_estimate()
{
    Eigen::Quaterniond init_quat = init_orient();

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
        // Eigen::Vector3d euler_curr = quat_to_eulers(q_curr);
        // this->euler_estimate.push_back(euler_curr);
        this->orient_estimate.push_back(q_curr);
    }
    

    return;
}

void COMPLEMENTARY_AHRS::ahrs_estimate()
{
    Eigen::Quaterniond init_quat = init_orient();

    for (size_t i = 1; i < this->imu_data.size(); i++)
    {
        Eigen::Quaterniond q_curr;
        Eigen::Quaterniond q_last = this->orient_estimate.back();

        uint64_t t_now = this->imu_data[i].timestamp;
        uint64_t t_last = this->imu_data[i-1].timestamp;
        double dt = static_cast<double>(t_now - t_last) / 1e9;

        this->CHEBY_LP_FILT(imu_data[i]);
        double wx = this->imu_data[i].gyro.x() - this->gyro_bias.x();
        double wy = this->imu_data[i].gyro.y() - this->gyro_bias.y();
        double wz = this->imu_data[i].gyro.z() - this->gyro_bias.z();

        q_curr.w() = q_last.w() + dt * 0.5 * (-q_last.x() * wx - q_last.y() * wy - q_last.z() * wz);
        q_curr.x() = q_last.x() + dt * 0.5 * ( q_last.w() * wx + q_last.y() * wz - q_last.z() * wy);
        q_curr.y() = q_last.y() + dt * 0.5 * ( q_last.w() * wy - q_last.x() * wz + q_last.z() * wx);
        q_curr.z() = q_last.z() + dt * 0.5 * ( q_last.w() * wz + q_last.x() * wy - q_last.y() * wx);
        q_curr.normalize();

        this->COMPLEMENTARY_FILT(imu_data[i], q_curr);
        this->orient_estimate.push_back(q_curr);
        // Eigen::Vector3d euler_curr = quat_to_eulers(q_curr);
        // this->euler_estimate.push_back(euler_curr);
    }
    
    return;
}

void COMPLEMENTARY_AHRS::COMPLEMENTARY_FILT(IMUSample& data, Eigen::Quaterniond& q)
{
    double a_mag = data.accel.norm();
    if (abs(a_mag - GRAV) > 0.30)
        return;

    Eigen::Quaterniond q_accel = Eigen::Quaterniond::Identity();
    Eigen::Vector3d a_norm = data.accel.normalized();
    
    double roll_accel  = atan2(data.accel.y(), data.accel.z());
    double pitch_accel = atan2(-data.accel.x(), sqrt(data.accel.y()*data.accel.y() + data.accel.z()*data.accel.z()));
    double yaw_gyro = yaw_from_quat(q); // take yaw from gyro's since accelerometers can't measure
    Eigen::Vector3d euler_target(roll_accel, pitch_accel, yaw_gyro);

    Eigen::Quaterniond q_target = Eigen::Quaterniond::Identity();
    eulers_to_quat(euler_target, q_target);

    q = q.slerp(0.05, q_target); 
    q.normalize();
    return;
}

void MAHONY_AHRS::ahrs_estimate()
{
    Eigen::Quaterniond init_quat = init_orient();

    for (size_t i = 1; i < this->imu_data.size(); i++)
    {
        Eigen::Quaterniond q_curr;
        Eigen::Quaterniond q_last = this->orient_estimate.back();

        uint64_t t_now = this->imu_data[i].timestamp;
        uint64_t t_last = this->imu_data[i-1].timestamp;
        double dt = static_cast<double>(t_now - t_last) / 1e9;

        this->CHEBY_LP_FILT(imu_data[i]);
        this->MAHONY_FILT(imu_data[i], dt);

        double wx = this->imu_data[i].gyro.x();
        double wy = this->imu_data[i].gyro.y();
        double wz = this->imu_data[i].gyro.z();

        q_curr.w() = q_last.w() + dt * 0.5 * (-q_last.x() * wx - q_last.y() * wy - q_last.z() * wz);
        q_curr.x() = q_last.x() + dt * 0.5 * ( q_last.w() * wx + q_last.y() * wz - q_last.z() * wy);
        q_curr.y() = q_last.y() + dt * 0.5 * ( q_last.w() * wy - q_last.x() * wz + q_last.z() * wx);
        q_curr.z() = q_last.z() + dt * 0.5 * ( q_last.w() * wz + q_last.x() * wy - q_last.y() * wx);
        q_curr.normalize();

        // Eigen::Vector3d euler_curr = quat_to_eulers(q_curr);
        // this->euler_estimate.push_back(euler_curr);
        this->orient_estimate.push_back(q_curr);
    }
    
}

void MAHONY_AHRS::MAHONY_FILT(IMUSample& data, double dt)
{
    double a_mag = data.accel.norm();
    double accel_error = std::abs(a_mag - GRAV);
    if (accel_error > 0.40)
    {
        data.gyro = (data.gyro - this->gyro_bias);
        return;
    }
    Eigen::Quaterniond q_prev = this->orient_estimate.back();
    Eigen::Vector3d v_world(0.0, 0.0, 1.0);
    Eigen::Vector3d v_pred = q_prev.conjugate() * v_world; // q * v would rotate body->world, q_conj * v would rotate world->body

    Eigen::Vector3d accel_norm = data.accel.normalized();
    Eigen::Vector3d error_axis = accel_norm.cross(v_pred);
    this->error += error_axis * dt;

    data.gyro = (data.gyro - this->gyro_bias) + (kp * error_axis) + (ki * this->error);
}

void MADGWICK_AHRS::ahrs_estimate()
{
    Eigen::Quaterniond init_quat = init_orient();

    for (size_t i = 1; i < this->imu_data.size(); i++)
    {
        Eigen::Quaterniond q_gyro;
        Eigen::Quaterniond q_last = this->orient_estimate.back();

        uint64_t t_now = this->imu_data[i].timestamp;
        uint64_t t_last = this->imu_data[i-1].timestamp;
        double dt = static_cast<double>(t_now - t_last) / 1e9;

        this->CHEBY_LP_FILT(imu_data[i]);

        double wx = this->imu_data[i].gyro.x() - this->gyro_bias.x();
        double wy = this->imu_data[i].gyro.y() - this->gyro_bias.y();
        double wz = this->imu_data[i].gyro.z() - this->gyro_bias.z();

        q_gyro.w() = q_last.w() + dt * 0.5 * (-q_last.x() * wx - q_last.y() * wy - q_last.z() * wz);
        q_gyro.x() = q_last.x() + dt * 0.5 * ( q_last.w() * wx + q_last.y() * wz - q_last.z() * wy);
        q_gyro.y() = q_last.y() + dt * 0.5 * ( q_last.w() * wy - q_last.x() * wz + q_last.z() * wx);
        q_gyro.z() = q_last.z() + dt * 0.5 * ( q_last.w() * wz + q_last.x() * wy - q_last.y() * wx);
        q_gyro.normalize();

        double gyro_mag = sqrt(wx*wx + wy*wy + wz*wz);
        double step_size = this->alpha * gyro_mag * dt;
        this->MADGWICK_FILT(imu_data[i], q_gyro, step_size);
 
        // Eigen::Vector3d euler_curr = quat_to_eulers(q_gyro);
        // this->euler_estimate.push_back(euler_curr);
        this->orient_estimate.push_back(q_gyro);
    }
    
}

void MADGWICK_AHRS::MADGWICK_FILT(IMUSample& data, Eigen::Quaterniond& q_gyro, double step_size)
{

    double a_mag = data.accel.norm();
    double accel_error = std::abs(a_mag - GRAV);

    const double LOWER_BOUND = 0.15;
    const double UPPER_BOUND = 0.55;

    double trust_factor = 1.0;
    if (accel_error <= LOWER_BOUND) {
        trust_factor = 1.0;
    } else if (accel_error >= UPPER_BOUND) {
        trust_factor = 0.0;
    } else {
        trust_factor = 1.0 - ((accel_error - LOWER_BOUND) / (UPPER_BOUND - LOWER_BOUND)); // linear interpolation
    }

    double active_slerp_gain = 0.75 * trust_factor;
    double active_step_size  = step_size  * trust_factor;

    Eigen::Quaterniond q_prev = this->orient_estimate.back();
    Eigen::Vector3d v_world(0.0, 0.0, 1.0);
    Eigen::Vector3d v_pred = q_prev.conjugate() * v_world; 
    // taking residual of v_world in body frame matches teh same frame as the accel vector which is in body frame
    Eigen::Vector3d accel_norm = data.accel.normalized();
    Eigen::Vector3d residual = v_pred - accel_norm;
    // Since for small angles sin theta = theta, if you pass in cross product error like in mahony instead of residual
    // you would still be proportionally minimizing the error

    double w = q_prev.w();
    double x = q_prev.x();
    double y = q_prev.y();
    double z = q_prev.z();

    Eigen::Matrix<double, 3, 4> J;
    J(0, 0) = -2.0 * y;
    J(0, 1) =  2.0 * z;
    J(0, 2) = -2.0 * w;
    J(0, 3) =  2.0 * x;

    J(1, 0) =  2.0 * x;
    J(1, 1) =  2.0 * w;
    J(1, 2) =  2.0 * z;
    J(1, 3) =  2.0 * y;

    J(2, 0) =  0.0;
    J(2, 1) = -4.0 * x;
    J(2, 2) = -4.0 * y;
    J(2, 3) =  0.0;

    Eigen::Vector4d gradient = J.transpose() * residual;
    gradient.normalize();
    Eigen::Quaterniond q_madgwick = Eigen::Quaterniond::Identity();
    q_madgwick.w() = w - active_step_size * gradient(0);
    q_madgwick.x() = x - active_step_size * gradient(1);
    q_madgwick.y() = y - active_step_size * gradient(2);
    q_madgwick.z() = z - active_step_size * gradient(3);
    q_madgwick.normalize();
    Eigen::Vector3d euler_madgwick = quat_to_eulers(q_madgwick);
    this->euler_correct.push_back(euler_madgwick);

    q_gyro = q_gyro.slerp(active_slerp_gain, q_madgwick);
}

void EKF_AHRS::ahrs_estimate()
{
    Eigen::Quaterniond init_quat = init_orient();
    this->gyro_bias_estimate.push_back(gyro_bias);
    this->init_params();

    for (size_t i = 1; i < this->imu_data.size(); i++)
    {
        this->CHEBY_LP_FILT(imu_data[i]);
        uint64_t t_now = this->imu_data[i].timestamp;
        uint64_t t_last = this->imu_data[i-1].timestamp;
        double dt = static_cast<double>(t_now - t_last) / 1e9;

        this->predict(imu_data[i], dt);
        this->correct(imu_data[i]);
    }
}

void EKF_AHRS::predict(IMUSample& data, double dt)
{
    Eigen::Quaterniond q_last(this->x(0), this->x(1), this->x(2), this->x(3));
    this->A.setIdentity();

    double wx = data.gyro.x() - this->x(4);
    double wy = data.gyro.y() - this->x(5);
    double wz = data.gyro.z() - this->x(6);

    Eigen::Quaterniond q_gyro;
    q_gyro.w() = q_last.w() + dt * 0.5 * (-q_last.x() * wx - q_last.y() * wy - q_last.z() * wz);
    q_gyro.x() = q_last.x() + dt * 0.5 * ( q_last.w() * wx + q_last.y() * wz - q_last.z() * wy);
    q_gyro.y() = q_last.y() + dt * 0.5 * ( q_last.w() * wy - q_last.x() * wz + q_last.z() * wx);
    q_gyro.z() = q_last.z() + dt * 0.5 * ( q_last.w() * wz + q_last.x() * wy - q_last.y() * wx);
    q_gyro.normalize();
    this->x.head<4>() = Eigen::Vector4d(q_gyro.w(), q_gyro.x(), q_gyro.y(), q_gyro.z());

    double half_dt = dt * 0.5;
    // --- Top-Left Block (4x4): Quaternion w.r.t Quaternion ---
    this->A(0, 1) = -half_dt * wx;
    this->A(0, 2) = -half_dt * wy;
    this->A(0, 3) = -half_dt * wz;

    this->A(1, 0) =  half_dt * wx;
    this->A(1, 2) =  half_dt * wz;
    this->A(1, 3) = -half_dt * wy;

    this->A(2, 0) =  half_dt * wy;
    this->A(2, 1) = -half_dt * wz;
    this->A(2, 3) =  half_dt * wx;

    this->A(3, 0) =  half_dt * wz;
    this->A(3, 1) =  half_dt * wy;
    this->A(3, 2) = -half_dt * wx;

    // --- Top-Right Block (4x3): Quaternion w.r.t Bias ---
    this->A(0, 4) =  half_dt * q_last.x();
    this->A(0, 5) =  half_dt * q_last.y();
    this->A(0, 6) =  half_dt * q_last.z();

    this->A(1, 4) = -half_dt * q_last.w();
    this->A(1, 5) =  half_dt * q_last.z();
    this->A(1, 6) = -half_dt * q_last.y();

    this->A(2, 4) = -half_dt * q_last.z();
    this->A(2, 5) = -half_dt * q_last.w();
    this->A(2, 6) =  half_dt * q_last.x();

    this->A(3, 4) =  half_dt * q_last.y();
    this->A(3, 5) = -half_dt * q_last.x();
    this->A(3, 6) = -half_dt * q_last.w();
    this->P = this->A * this->P * this->A.transpose() + this->Q;
    return;
}

void EKF_AHRS::correct(IMUSample& data)
{
    double a_mag = data.accel.norm();
    double accel_error = std::abs(a_mag - GRAV);
    if (accel_error > 0.40)
    {
        Eigen::Quaterniond q_pred(this->x(0), this->x(1), this->x(2), this->x(3));
        this->orient_estimate.push_back(q_pred);
        return;
    }

    this->H = Eigen::Matrix<double, 3, 7>::Zero();
    Eigen::Vector3d accel_norm = data.accel.normalized();
    Eigen::Quaterniond q_pred(this->x(0), this->x(1), this->x(2), this->x(3));
    Eigen::Vector3d v_pred(0.0, 0.0, 1.0);
    Eigen::Vector3d h_x = q_pred.conjugate() * v_pred;
    Eigen::Vector3d residual = accel_norm - h_x;
    
    double qw = this->x(0);
    double qx = this->x(1);
    double qy = this->x(2);
    double qz = this->x(3);

    // --- Row 0: Accelerometer X-axis derivatives ---
    this->H(0, 0) = -2.0 * qy;
    this->H(0, 1) =  2.0 * qz;
    this->H(0, 2) = -2.0 * qw;
    this->H(0, 3) =  2.0 * qx;

    // --- Row 1: Accelerometer Y-axis derivatives ---
    this->H(1, 0) =  2.0 * qx;
    this->H(1, 1) =  2.0 * qw;
    this->H(1, 2) =  2.0 * qz;
    this->H(1, 3) =  2.0 * qy;

    // --- Row 2: Accelerometer Z-axis derivatives ---
    this->H(2, 0) =  2.0 * qw;
    this->H(2, 1) = -2.0 * qx;
    this->H(2, 2) = -2.0 * qy;
    this->H(2, 3) =  2.0 * qz;

    Eigen::Matrix<double, 3, 3> S = this->H * this->P * this->H.transpose() + this->R; // innovation covariance
    Eigen::Matrix<double, 7, 3> K = this->P * this->H.transpose() * S.inverse(); // Kalman gain
    this->x += K * residual;
    Eigen::Quaterniond q_updated(this->x(0), this->x(1), this->x(2), this->x(3));
    q_updated.normalize();
    Eigen::Vector3d curr_gyro_bias(this->x(4), this->x(5), this->x(6));
    this->gyro_bias_estimate.push_back(curr_gyro_bias);
    this->orient_estimate.push_back(q_updated);

    // Eigen::Vector3d euler_curr = quat_to_eulers(q_updated);
    // this->euler_estimate.push_back(euler_curr);
    this->x.head<4>() = Eigen::Vector4d(q_updated.w(), q_updated.x(), q_updated.y(), q_updated.z());
    Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
    this->P = (I - K * this->H) * this->P;

    return;
}

void EKF_AHRS::init_params()
{
    Eigen::Quaterniond init_quat = this->orient_estimate.back();
    this->x.head<4>() = Eigen::Vector4d(init_quat.w(), init_quat.x(), init_quat.y(), init_quat.z());
    this->x(4) = this->gyro_bias.x();
    this->x(5) = this->gyro_bias.y();
    this->x(6) = this->gyro_bias.z();
    this->P = Eigen::MatrixXd::Identity(7, 7) * 0.01;

    this->Q = Eigen::Matrix<double, 7, 7>::Zero();
    this->Q.block<4,4>(0,0) = Eigen::Matrix4d::Identity() * (sigma_gyro_sq * dt);
    this->Q.block<3,3>(4,4) = Eigen::Matrix3d::Identity() * (sigma_bias_sq * dt);

    this->A = Eigen::MatrixXd::Identity(7, 7);
    this->R = Eigen::MatrixXd::Identity(3, 3) * sigma_accel_sq;
    this->H = Eigen::Matrix<double, 3, 7>::Zero();
}

void BASE_AHRS::align_estimate()
{
    Eigen::Quaterniond q_init = this->q_gt0 * this->q_V_B; // body -> vicon * vicon -> room
    Eigen::Vector3d    t_init = this->q_gt0 * this->t_V_B_eigen + this->t_gt0;

    for (size_t i = 0; i < this->orient_estimate.size(); i++)
    {
        this->orient_estimate[i] = q_init * this->orient_estimate[i];
        Eigen::Vector3d euler_curr = quat_to_eulers(this->orient_estimate[i]);
        this->euler_estimate.push_back(euler_curr);

        this->pos_estimate[i] = q_init * this->pos_estimate[i] + t_init;
    }
}