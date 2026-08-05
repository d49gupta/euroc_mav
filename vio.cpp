#include "vio.hpp"

void VisualInertialOdometry::init_params()
{
    this->P.setZero();

    // Initial uncertainty of initial state
    P.block<3,3>(0,0) = 1e-4 * Eigen::Matrix3d::Identity(); // position
    P.block<3,3>(3,3) = 1e-2 * Eigen::Matrix3d::Identity(); // velocity
    P.block<3,3>(6,6) = 1e-3 * Eigen::Matrix3d::Identity(); // orientation
    P.block<3,3>(9,9) = 1e-3 * Eigen::Matrix3d::Identity(); // acceleration bias
    P.block<3,3>(12,12) = 1e-6 * Eigen::Matrix3d::Identity(); // gyroscope bias

    this->A = Eigen::Matrix<double,15,15>::Identity();
    this->Q.setZero();

    // TODO: CHECK R VALUES (update dynamically by claculating std at ecah frame)
    // Measurement noise: ~1 px std dev, converted to normalized image coordinates
    // (undistortPoints() in getPose() outputs normalized coords, so divide by focal length)
    const double sigma_pixel = 5.0;
    const double sigma_norm = sigma_pixel / this->K_left.at<double>(0, 0);
    this->R = (sigma_norm * sigma_norm) * Eigen::Matrix2d::Identity();
}

void VisualInertialOdometry::update_covariance(IMUSample& data, double dt)
{
    Eigen::Matrix3d R = last_estimate.orient_estimate.toRotationMatrix();
    Eigen::Vector3d unbiased_accel = data.accel - last_estimate.accel_bias;
    Eigen::Vector3d rvec = -dt * (data.gyro - last_estimate.gyro_bias);

    Eigen::Matrix3d accel_skew = VectoSkew(unbiased_accel);
    Eigen::Matrix3d mat = vectoR(rvec);

    this->A = Eigen::Matrix<double,15,15>::Identity();
    // Position covariance
    A.block<3,3>(0,3) = dt * Eigen::Matrix3d::Identity();
    A.block<3,3>(0,6) = -0.5 * dt * dt * R * accel_skew;
    A.block<3,3>(0,9) = -0.5 * dt * dt * R;

    // Velocity covariance
    A.block<3,3>(3,6) = -dt * R * accel_skew;
    A.block<3,3>(3,9) = -dt * R;

    // Orientation covariance
    A.block<3,3>(6,6) = mat;
    A.block<3,3>(6,12) = -dt * Eigen::Matrix3d::Identity();

    // TODO: CHECK Q VALUES
    const double sigma_a  = 2.0000e-3;   // accel noise density   [m/s^2/sqrt(Hz)]
    const double sigma_g  = 1.6968e-4;   // gyro noise density    [rad/s/sqrt(Hz)]
    const double sigma_ba = 3.0000e-3;   // accel bias random walk[m/s^3/sqrt(Hz)]
    const double sigma_bg = 1.9393e-5;   // gyro bias random walk [rad/s^2/sqrt(Hz)]

    // Q.block<3,3>(0,0) — δp: left as zero, no direct noise injection
    Q.block<3,3>(3,3)   = (sigma_a  * sigma_a  * dt) * Eigen::Matrix3d::Identity();  // δv
    Q.block<3,3>(6,6)   = (sigma_g  * sigma_g  * dt) * Eigen::Matrix3d::Identity();  // δφ
    Q.block<3,3>(9,9)   = (sigma_ba * sigma_ba * dt) * Eigen::Matrix3d::Identity();  // δb_a
    Q.block<3,3>(12,12) = (sigma_bg * sigma_bg * dt) * Eigen::Matrix3d::Identity();  // δb_g
    this->P = this->A * this->P * this->A.transpose() + this->Q;
}

void VisualInertialOdometry::predict(IMUSample& data, double dt)
{
    this->xr.setZero();
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
    this->last_estimate = curr_estimate;
}

void VisualInertialOdometry::update(uint64_t timestep)
{
    // use inliers from pnpsolveransac for measurements
    if (this->last_inlier_ids.empty() || this->last_inlier_pts.empty())
        return;
    
    vector<Point2f> undistorted_left_pts;
    // TODO: Maybe instead of relying on ransac to consider inliers, use your own reprojection method to get inliers from raw points
    undistortPoints(this->last_inlier_pts, undistorted_left_pts, this->K_left, this->D_left, Mat(), Mat());

    for (size_t a = 0; a < undistorted_left_pts.size(); a++)
    {
        if (!this->global_map.count(this->last_inlier_ids[a]))
            continue;

        this->H = Eigen::Matrix<double, 2, 15>::Zero();
        Eigen::Matrix3d R_W_B  = this->last_estimate.orient_estimate.toRotationMatrix();
        Eigen::Vector3d p_W_B  = this->last_estimate.pos_estimate;
        this->last_estimate.timestamp = timestep;

        Eigen::Matrix3d R_W_C = R_W_B * R_B_C0_eigen;
        Eigen::Vector3d p_W_C = p_W_B + R_W_B * t_B_C0_eigen;
        Eigen::Matrix3d R_C_W = R_W_C.transpose();

        Point2f left_pt = undistorted_left_pts[a];
        Point3d P_W_VO = this->global_map[this->last_inlier_ids[a]].pos;
        Eigen::Vector3d P_W_VO_eigen(P_W_VO.x, P_W_VO.y, P_W_VO.z); // landmark relative to starting camera frame (world vo)
        Eigen::Vector3d P_W_eigen = R_B_C0_eigen * P_W_VO_eigen + t_B_C0_eigen; // landmark relative to starting IMU frame
        Eigen::Vector3d P_C = R_C_W * (P_W_eigen - p_W_C); // landmark relative to current camera frame
        Eigen::Vector3d P_B = R_W_B.transpose() * (P_W_eigen - p_W_B); // landmark relative to current IMU frame
        Eigen::Matrix3d pb_skew = VectoSkew(P_B);

        double u_proj = P_C.x() / P_C.z();
        double v_proj = P_C.y() / P_C.z();
        Eigen::Vector2d residual(left_pt.x - u_proj, left_pt.y - v_proj);

        Eigen::Matrix<double,2,3> J_proj;
        J_proj << 1.0/P_C.z(), 0, -P_C.x()/(P_C.z()*P_C.z()),
                0, 1.0/P_C.z(), -P_C.y()/(P_C.z()*P_C.z());

        H.block<2,3>(0,0) = J_proj * (-R_C_W);                           
        H.block<2,3>(0,6) = J_proj * R_B_C0_eigen.transpose() * pb_skew;

        Eigen::Matrix<double, 2, 2>  S = this->H * this->P * this->H.transpose() + this->R;
        Eigen::Matrix<double, 15, 2> K = this->P * this->H.transpose() * S.inverse();
        Eigen::Matrix<double, 15, 1> correction = K * residual;

        stateEstimate& est = this->last_estimate;
        Eigen::Vector3d dp   = correction.segment<3>(0);
        Eigen::Vector3d dv   = correction.segment<3>(3);
        Eigen::Vector3d dphi = correction.segment<3>(6);
        Eigen::Vector3d dba  = correction.segment<3>(9);
        Eigen::Vector3d dbg  = correction.segment<3>(12);

        est.pos_estimate += dp;
        est.vel_estimate += dv;
        est.accel_bias   += dba;
        est.gyro_bias    += dbg;

        // convert rotation correction to quaternion
        Eigen::Quaterniond dq(1.0, 0.5*dphi.x(), 0.5*dphi.y(), 0.5*dphi.z());
        dq.normalize();
        est.orient_estimate = (est.orient_estimate * dq).normalized();

        Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
        this->P = (I - K * H) * this->P;
    }
}

void VisualInertialOdometry::outputState(string filename)
{
    std::ofstream csv_file(filename);
    csv_file << "timestamp,p_x,p_y,p_z,v_x,v_y,v_z,q_w,q_x,q_y,q_z,ba_x,ba_y,ba_z,bg_x,bg_y,bg_z\n";
    csv_file << std::fixed << std::setprecision(6);

    cout << "Estimate size: " << this->state_estimate.size() << endl;

    Eigen::Quaterniond prev_q_room_V = Eigen::Quaterniond::Identity();
    bool have_prev_q_room_V = false;

    for (const auto &state : this->state_estimate)
    {
        Eigen::Vector3d p_W_B = state.pos_estimate;
        Eigen::Matrix3d R_W_B = state.orient_estimate.toRotationMatrix();

        // Align into Vicon room frame via the t=0 ground-truth anchor
        Eigen::Matrix3d R_align = q_gt0.toRotationMatrix();
        Eigen::Matrix3d R_room_V = R_align * R_W_B;
        Eigen::Vector3d p_room_V = R_align * p_W_B + t_gt_0;
        Eigen::Quaterniond q_room_V(R_room_V);

        if (p_room_V.norm() > 1e3)
            continue;

        if (have_prev_q_room_V && q_room_V.dot(prev_q_room_V) < 0.0)
            q_room_V.coeffs() *= -1.0;
        prev_q_room_V = q_room_V;
        have_prev_q_room_V = true;

        // velocity of marker and body are the same so don't add translation, just rotation is needed for the transformation
        // biases dont need transform, they dont have direction so its meaningless
        Eigen::Vector3d v_room = R_init * state.vel_estimate;

        csv_file << state.timestamp << ","
                  << p_room_V.x() << "," << p_room_V.y() << "," << p_room_V.z() << ","
                  << v_room.x()   << "," << v_room.y()   << "," << v_room.z()   << ","
                  << q_room_V.w() << "," << q_room_V.x() << "," << q_room_V.y() << "," << q_room_V.z() << ","
                  << state.accel_bias.x() << "," << state.accel_bias.y() << "," << state.accel_bias.z() << ","
                  << state.gyro_bias.x()  << "," << state.gyro_bias.y()  << "," << state.gyro_bias.z()  << "\n";
    }
    csv_file.close();
}