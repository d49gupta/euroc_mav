#include "vio.hpp"

namespace
{
    // relative-pose edge for the pose graph: pose_i/pose_j are [rx,ry,rz,tx,ty,tz] (angle-axis +
    // translation, body-in-world). Residual is the discrepancy between the measured relative
    // transform (R_meas, t_meas) and what pose_i/pose_j currently predict, whitened by sqrt_information
    struct RelativePoseFactor
    {
        RelativePoseFactor(const Eigen::Matrix3d& R_meas, const Eigen::Vector3d& t_meas,
                            const Eigen::Matrix<double, 6, 6>& sqrt_information)
            : R_meas(R_meas), t_meas(t_meas), sqrt_information(sqrt_information) {}

        template <typename T>
        bool operator()(const T* const pose_i, const T* const pose_j, T* residuals_ptr) const
        {
            Eigen::Matrix<T, 3, 3> R_i, R_j;
            ceres::AngleAxisToRotationMatrix(pose_i, R_i.data());
            ceres::AngleAxisToRotationMatrix(pose_j, R_j.data());

            Eigen::Matrix<T, 3, 1> t_i(pose_i[3], pose_i[4], pose_i[5]);
            Eigen::Matrix<T, 3, 1> t_j(pose_j[3], pose_j[4], pose_j[5]);

            Eigen::Matrix<T, 3, 3> R_i_inv = R_i.transpose();
            Eigen::Matrix<T, 3, 3> R_rel_predicted = R_i_inv * R_j;
            Eigen::Matrix<T, 3, 1> t_rel_predicted = R_i_inv * (t_j - t_i);

            Eigen::Matrix<T, 3, 3> R_error = R_meas.cast<T>().transpose() * R_rel_predicted;
            T angle_axis_error[3];
            ceres::RotationMatrixToAngleAxis(R_error.data(), angle_axis_error);

            Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
            residuals.template segment<3>(0) = t_rel_predicted - t_meas.cast<T>();
            residuals(3) = angle_axis_error[0];
            residuals(4) = angle_axis_error[1];
            residuals(5) = angle_axis_error[2];

            residuals.applyOnTheLeft(sqrt_information.cast<T>());
            return true;
        }

        static ceres::CostFunction* Create(const Eigen::Matrix3d& R_meas, const Eigen::Vector3d& t_meas,
                                            const Eigen::Matrix<double, 6, 6>& sqrt_information)
        {
            // create new RelativePoseFactor with relative pose movement and information matrix weighting
            // Turn that into a cost function represented as edge in ceres graph (#residuals, size of each node connecting to edge)
            return new ceres::AutoDiffCostFunction<RelativePoseFactor, 6, 6, 6>(
                new RelativePoseFactor(R_meas, t_meas, sqrt_information));
        }

        Eigen::Matrix3d R_meas;
        Eigen::Vector3d t_meas;
        Eigen::Matrix<double, 6, 6> sqrt_information;
    };
}

void VisualInertialOdometry::init_params()
{
    this->resetCovariance(this->P);
    this->resetCovariance(this->P_keyframe);

    this->A = Eigen::Matrix<double,15,15>::Identity();
    this->Q.setZero();

    // TODO: vocab.load("path/to/vocabulary.yml.gz") once a vocabulary has been trained
    this->db.setVocabulary(this->vocab, false, 0);

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
    // P_keyframe is just meant to capture uncertainty in IMU preintegration for edges in factor graphs
    this->P_keyframe = this->A * this->P_keyframe * this->A.transpose() + this->Q;
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

        // convert rotation correction to quaternion and multiply correction (not add)
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

void VisualInertialOdometry::outputTrajectory(string filename)
{
    std::ofstream csv_file(filename);
    csv_file << "frame_id,p_x,p_y,p_z,q_w,q_x,q_y,q_z\n";
    csv_file << std::fixed << std::setprecision(6);

    Eigen::Quaterniond prev_q_room_V = Eigen::Quaterniond::Identity();
    bool have_prev_q_room_V = false;

    for (const auto& kf : keyframe_store)
    {
        cv::Mat R_cv;
        cv::Rodrigues(kf.rvec, R_cv);
        Eigen::Matrix3d R_W_B;
        R_W_B << R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
                 R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
                 R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2);
        Eigen::Vector3d p_W_B(kf.tvec.at<double>(0), kf.tvec.at<double>(1), kf.tvec.at<double>(2));

        // Align into Vicon room frame via the t=0 ground-truth anchor, same as outputState()
        // so the two trajectories are directly comparable
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

        csv_file << kf.frame_id << ","
                  << p_room_V.x() << "," << p_room_V.y() << "," << p_room_V.z() << ","
                  << q_room_V.w() << "," << q_room_V.x() << "," << q_room_V.y() << "," << q_room_V.z() << "\n";
    }
    csv_file.close();
}

KeyframeNode VisualInertialOdometry::createNode()
{
    // build keypoints/landmarks together from the tracked+triangulated points so indices match
    vector<cv::KeyPoint> keypoints;
    vector<Point3d> curr_landmarks;
    for (size_t i = 0; i < this->prev_feature_ids.size(); i++)
    {
        int id = this->prev_feature_ids[i];
        if (!global_map.count(id))
            continue;

        keypoints.emplace_back(this->prev_left_feature_pts[i], 31.0f); // 31.0 as keypoint diameter size (same as ORB)

        // global_map stores landmarks in VO-world convert to IMU world (ESKF)
        Point3d P_W_VO = global_map[id].pos;
        Eigen::Vector3d P_W_VO_eigen(P_W_VO.x, P_W_VO.y, P_W_VO.z);
        Eigen::Vector3d P_W_eigen = R_B_C0_eigen * P_W_VO_eigen + t_B_C0_eigen;
        curr_landmarks.push_back(Point3d(P_W_eigen.x(), P_W_eigen.y(), P_W_eigen.z()));
    }

    // orb->compute() is allowed to drop keypoints it can't describ so re-align landmarks against 
    // whatever keypoints actually survive
    cv::Mat descriptors;
    vector<cv::KeyPoint> computed_keypoints = keypoints;
    this->orb->compute(this->prev_left_img, computed_keypoints, descriptors);

    vector<Point3d> matched_landmarks;
    size_t src = 0;
    // All keypoints tracked temporally are a subset of computed_keypoints of the entire image (same order) 
    for (const auto& kp : computed_keypoints)
    {
        while (src < keypoints.size() && keypoints[src].pt != kp.pt)
            src++;
        if (src < keypoints.size())
            matched_landmarks.push_back(curr_landmarks[src]);
    }

    KeyframeNode keyframe_node;
    keyframe_node.frame_id = this->frame_id;
    keyframe_node.keypoints = computed_keypoints;
    keyframe_node.descriptors = descriptors;
    keyframe_node.landmarks = matched_landmarks;

    stateEstimate& est = this->last_estimate;
    Eigen::Matrix3d R = est.orient_estimate.toRotationMatrix();
    cv::Mat R_cv = (cv::Mat_<double>(3,3) << R(0,0), R(0,1), R(0,2),
                                            R(1,0), R(1,1), R(1,2),
                                            R(2,0), R(2,1), R(2,2));
    cv::Mat rvec;
    cv::Rodrigues(R_cv, rvec);
    cv::Mat tvec = (cv::Mat_<double>(3,1) << est.pos_estimate(0), est.pos_estimate(1), est.pos_estimate(2));
    // rvec and tvec are IMU body frame to IMU world frame, which is what you want to optimize
    keyframe_node.rvec = rvec;
    keyframe_node.tvec = tvec;
    keyframe_node.P = this->P_keyframe;
    return keyframe_node;
}

void VisualInertialOdometry::enqueueKeyframe(KeyframeNode node)
{
    {
        // lock_guard unlocks after it goes out of scope {}
        std::lock_guard<std::mutex> lock(queue_mutex);
        keyframe_queue.push_back(std::move(node));
    }
    queue_cv.notify_one();
}

void VisualInertialOdometry::slamLoop()
{
    // thread calls method to run until completion, while loop ensures constant polling over keyframe lifetime
    // without polling loop, once function is done, thread dies out
    while (running)
    {
        // lock guard stays locked until out of scope
        // with unique_lock you can manually unlock and lock (for queue_cv.wait)
        std::unique_lock<std::mutex> lock(queue_mutex);

        // checks predicate (second param), if true keep mutex locked and run code
        // if false, does not return, stays asleep until notified by notify_one, notify_all then rechecks condition
        // while sleeping, queue_cv unlocks the mutex and locks it back up when notified
        // !running is there to make sure that when thread is shutdown (!running) cv stops waiting for notify and gives up control so
        // it can join back up with main thread
        queue_cv.wait(lock, [this]{ return !keyframe_queue.empty() || !running; });
        if (!running && keyframe_queue.empty())
            break;

        KeyframeNode node = std::move(keyframe_queue.front());
        keyframe_queue.pop_front();
        lock.unlock();

        // convert MAT rvec to Eigen rotation matrix
        auto toEigenRot = [](const cv::Mat& rvec) 
        {
            cv::Mat R_cv;
            cv::Rodrigues(rvec, R_cv);
            Eigen::Matrix3d R;
            R << R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
                 R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
                 R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2);
            return R;
        };
        // convert MAT tvec to Eigen translation vector
        auto toEigenVec = [](const cv::Mat& v) {
            return Eigen::Vector3d(v.at<double>(0), v.at<double>(1), v.at<double>(2));
        };

        // register this keyframe's own pose as a new Ceres parameter block (body-in-world,
        // matches node.rvec/node.tvec's convention directly - no conversion needed)
        std::array<double, 6> params;
        params[0] = node.rvec.at<double>(0);
        params[1] = node.rvec.at<double>(1);
        params[2] = node.rvec.at<double>(2);
        params[3] = node.tvec.at<double>(0);
        params[4] = node.tvec.at<double>(1);
        params[5] = node.tvec.at<double>(2);
        pose_params.push_back(params);
        size_t curr_idx = pose_params.size() - 1;

        if (curr_idx == 0)
        {
            // anchor the very first keyframe so the graph has a fixed global reference (without one fixed node, whole graph can drift)
            pose_graph.AddParameterBlock(pose_params[curr_idx].data(), 6); // add variable node
            pose_graph.SetParameterBlockConstant(pose_params[curr_idx].data()); // define variable node as const
        }
        else
        {
            // sequential (odometry) edge between the previous keyframe and this one, weighted by
            // node.P (the P_keyframe snapshot: uncertainty accumulated since the previous keyframe)
            const KeyframeNode& prev_node = keyframe_store.back();
            Eigen::Matrix3d R_prev = toEigenRot(prev_node.rvec);
            Eigen::Vector3d t_prev = toEigenVec(prev_node.tvec);
            Eigen::Matrix3d R_curr = toEigenRot(node.rvec);
            Eigen::Vector3d t_curr = toEigenVec(node.tvec);

            // relative pose between previous and current node
            Eigen::Matrix3d R_meas = R_prev.transpose() * R_curr;
            Eigen::Vector3d t_meas = R_prev.transpose() * (t_curr - t_prev);

            Eigen::Matrix<double, 6, 6> pose_cov;
            const int idx[6] = {0, 1, 2, 6, 7, 8}; // position + orientation block of the 15-dim covariance
            for (int r = 0; r < 6; r++)
                for (int c = 0; c < 6; c++)
                    pose_cov(r, c) = node.P(idx[r], idx[c]);
            
            // TODO: CHECK THIS
            Eigen::Matrix<double, 6, 6> sqrt_information = pose_cov.inverse().llt().matrixL().transpose();

            // RelativePoseFactor::Create() is the edge's rule for computing a residual (using relative transform and weight of information matrix)
            // Edge is defined as touching previous node (curr_idx - 1) and current node (curr_idx)
            // nullptr is loss function that can down-weight a residual if its suspicously large
            pose_graph.AddResidualBlock(
                RelativePoseFactor::Create(R_meas, t_meas, sqrt_information),
                nullptr,
                pose_params[curr_idx - 1].data(), pose_params[curr_idx].data());
        }

        DBoW3::BowVector bow_vec;
        vocab.transform(node.descriptors, bow_vec);
        DBoW3::QueryResults results;
        db.query(node.descriptors, results, /*max_results=*/5); // each result contains Id (index) and Score (similarity)

        bool loop_closed = false;
        for (const auto&r : results)
        {
            // if (r.Score < SCORE_THRESHOLD)
            //     continue;

            KeyframeNode candidate_node = keyframe_store[r.Id];
            int candidate_frame_id = candidate_node.frame_id;
            if (node.frame_id - candidate_frame_id < TEMPORAL_GAP) // loop closure not detected (too soon), just continuity
                continue;

            // match descriptors between the two keyframes to get real 2D<->3D correspondences
            cv::BFMatcher matcher(cv::NORM_HAMMING);
            std::vector<cv::DMatch> matches;
            matcher.match(node.descriptors, candidate_node.descriptors, matches);

            std::vector<cv::Point2f> image_points;
            std::vector<cv::Point3f> object_points;
            for (const auto& m : matches)
            {
                image_points.push_back(node.keypoints[m.queryIdx].pt);
                object_points.push_back(candidate_node.landmarks[m.trainIdx]);
            }

            // check geometry verification
            cv::Mat rvec_loop, tvec_loop;
            std::vector<int> inliers;

            // solves IMU world to camera body
            bool success = cv::solvePnPRansac(object_points, image_points, K_left, D_left, rvec_loop, tvec_loop, true, 100, 3.0F, 0.99, inliers);
            if (!success || inliers.size() < MIN_PNP_INLIERS)
                continue;

            // rvec_loop/tvec_loop are T_C_W (world->camera, standard PnP convention) for node.
            // candidate_node.rvec/tvec are T_W_B (body-in-world, ESKF convention)
            Eigen::Matrix3d R_C_W_verified = toEigenRot(rvec_loop);
            Eigen::Vector3d t_C_W_verified = toEigenVec(tvec_loop);

            Eigen::Matrix3d R_W_C_verified = R_C_W_verified.transpose();
            Eigen::Vector3d p_W_C_verified = -R_W_C_verified * t_C_W_verified;

            // IMU body to IMU world (matches ESKF frame you want to optimize)
            Eigen::Matrix3d R_W_B_verified = R_W_C_verified * R_B_C0_eigen.transpose();
            Eigen::Vector3d p_W_B_verified = p_W_C_verified - R_W_B_verified * t_B_C0_eigen;

            // saved in createNode as IMU body to IMU world
            Eigen::Matrix3d R_candidate = toEigenRot(candidate_node.rvec);
            Eigen::Vector3d p_candidate = toEigenVec(candidate_node.tvec);

            // relative transform from candidate node to current node - the loop-closure edge measurement
            Eigen::Matrix3d R_relative = R_candidate.transpose() * R_W_B_verified;
            Eigen::Vector3d p_relative = R_candidate.transpose() * (p_W_B_verified - p_candidate);

            // placeholder weighting until a real PnP covariance propagation replaces it: more
            // inliers -> more information (tighter edge)
            Eigen::Matrix<double, 6, 6> loop_information = Eigen::Matrix<double, 6, 6>::Identity() * static_cast<double>(inliers.size());
            Eigen::Matrix<double, 6, 6> loop_sqrt_information = loop_information.llt().matrixL().transpose();

            pose_graph.AddResidualBlock(
                RelativePoseFactor::Create(R_relative, p_relative, loop_sqrt_information),
                new ceres::HuberLoss(1.0), // down-weight a bad/false-positive match instead of letting it dominate the solve
                pose_params[r.Id].data(), pose_params[curr_idx].data());
            loop_closed = true;
        }

        db.add(node.descriptors); // Id is own counter based on .add() does not line up with frame_id
        keyframe_store.push_back(node);

        if (loop_closed)
        {
            // only worth re-optimizing when a new loop edge actually changed the graph's constraints
            ceres::Solver::Options options;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &pose_graph, &summary);

            // update keyframe_store after pose_graph optimizes nodes to maintain pose integrity when checking loop closure
            // and when measuring relative pose transform between sequential nodes
            for (size_t i = 0; i < pose_params.size(); i++)
            {
                keyframe_store[i].rvec = (cv::Mat_<double>(3,1) << pose_params[i][0], pose_params[i][1], pose_params[i][2]);
                keyframe_store[i].tvec = (cv::Mat_<double>(3,1) << pose_params[i][3], pose_params[i][4], pose_params[i][5]);
            }
        }
    }
}

void VisualInertialOdometry::resetCovariance(Eigen::Matrix<double, 15, 15>& P)
{
    P.setZero();

    // Initial uncertainty of initial state
    P.block<3,3>(0,0) = 1e-4 * Eigen::Matrix3d::Identity(); // position
    P.block<3,3>(3,3) = 1e-2 * Eigen::Matrix3d::Identity(); // velocity
    P.block<3,3>(6,6) = 1e-3 * Eigen::Matrix3d::Identity(); // orientation
    P.block<3,3>(9,9) = 1e-3 * Eigen::Matrix3d::Identity(); // acceleration bias
    P.block<3,3>(12,12) = 1e-6 * Eigen::Matrix3d::Identity(); // gyroscope bias
}