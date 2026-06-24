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

void BASE_AHRS::euler_error(const std::string& filename, const std::vector<Eigen::Vector3d>& euler1,
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
        Eigen::Vector3d euler_curr = quat_to_eulers(q_curr);
        this->orient_estimate.push_back(q_curr);
        this->euler_estimate.push_back(euler_curr);
    }

    return;
}

void COMPLEMENTARY_AHRS::ahrs_estimate()
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
    Eigen::Quaterniond q_target_align = this->q_align * q_target; // align body frame orietnation with vicon world frame
    q_target_align.normalize();

    q = q.slerp(0.05, q_target_align); 
    q.normalize();
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
        this->MAHONY_FILT(imu_data[i], dt);

        double wx = this->imu_data[i].gyro.x();
        double wy = this->imu_data[i].gyro.y();
        double wz = this->imu_data[i].gyro.z();

        q_curr.w() = q_last.w() + dt * 0.5 * (-q_last.x() * wx - q_last.y() * wy - q_last.z() * wz);
        q_curr.x() = q_last.x() + dt * 0.5 * ( q_last.w() * wx + q_last.y() * wz - q_last.z() * wy);
        q_curr.y() = q_last.y() + dt * 0.5 * ( q_last.w() * wy - q_last.x() * wz + q_last.z() * wx);
        q_curr.z() = q_last.z() + dt * 0.5 * ( q_last.w() * wz + q_last.x() * wy - q_last.y() * wx);
        q_curr.normalize();
        Eigen::Vector3d euler_curr = quat_to_eulers(q_curr);

        this->orient_estimate.push_back(q_curr);
        this->euler_estimate.push_back(euler_curr);
    }
}

void MAHONY_AHRS::MAHONY_FILT(IMUSample& data, double dt)
{
    Eigen::Quaterniond q_prev = this->orient_estimate.back();
    Eigen::Vector3d v_world(0.0, 0.0, 1.0);
    Eigen::Vector3d v_pred = q_prev.conjugate() * v_world; // q * v would rotate body->world, q_conj * v would rotate world->body

    Eigen::Vector3d accel_norm = data.accel.normalized();
    Eigen::Vector3d error_axis = accel_norm.cross(v_pred);
    this->error += error_axis * dt;

    data.gyro = (data.gyro - this->gyro_bias) + (kp * error_axis) + (ki * this->error);
}