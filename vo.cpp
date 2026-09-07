#include "vo.hpp"
#include "math_utils.hpp"

StereoTracker::StereoTracker(string nodeName, const DatasetConfig& config) : Node(nodeName)
{
    this->point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vo/point_cloud", 10);
    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("/vo/path", 10);
    path.header.frame_id = "map";

    this->rvec = cv::Mat::zeros(3, 1, CV_64F);
    this->tvec = cv::Mat::zeros(3, 1, CV_64F);
    this->R_W_C = cv::Mat::eye(3, 3, CV_64F);
    this->t_W_C = cv::Mat::zeros(3, 1, CV_64F);

    Pose inv_cam_pose;
    inv_cam_pose.translation = this->tvec.clone();
    inv_cam_pose.rotation = R_W_C.clone(); // same as R_C_W since identity matrix at t=0
    camera_pose[this->frame_id] = inv_cam_pose;

    optimization_log.open("src/build/landmark_optimization.csv");
    optimization_log
        << "keyframe,landmark_id,observations,iteration,"
        << "mean_residual,max_residual,dx_norm,"
        << "x,y,z\n";

    // Camera intrinsics/distortion + image size (cam0/cam1 sensor.yaml)
    this->K_left = config.K_left;
    this->D_left = config.D_left;
    this->K_right = config.K_right;
    this->D_right = config.D_right;
    this->image_size = config.image_size;

    // Extrinsic matrix from left camera to IMU body frame
    this->R_B_C0 = config.R_B_C0;
    this->t_B_C0 = config.t_B_C0;
    this->R_B_C0_eigen = toEigen3x3(R_B_C0);
    this->t_B_C0_eigen = toEigen3x1(t_B_C0);

    // Extrinsic matrix from right camera to IMU body frame
    this->R_B_C1 = config.R_B_C1;
    this->t_B_C1 = config.t_B_C1;

    // Inverted extrinsics left camera -> IMU body frame
    this->R_C_B0 = R_B_C0.t();
    this->t_C_B0 = -R_B_C0.t() * t_B_C0;
    this->R_C_B0_eigen = toEigen3x3(R_C_B0);
    this->t_C_B0_eigen = toEigen3x1(t_C_B0);

    // Transformation from left->right camera, needed as input to stereoRectify below
    this->R_left_right = R_B_C1.t() * R_B_C0;
    this->t_left_right = R_B_C1.t() * (t_B_C0 - t_B_C1);

    cv::stereoRectify(K_left, D_left, K_right, D_right, image_size,
                   R_left_right, t_left_right,
                   R1, R2, P1, P2, Q,
                   cv::CALIB_ZERO_DISPARITY, -1, image_size);

    // Extrinsic matrix from vicon marker to IMU body frame
    this->R_B_V = toEigen3x3(config.R_B_V);
    this->t_B_V = toEigen3x1(config.t_B_V);

    // Ground truth vicon marker pose at t=0, and ground truth state estimate at t=0
    this->q_vicon0 = config.q_vicon0;
    this->t_vicon0 = config.t_vicon0;
    this->q_gt0 = config.q_gt0;
    this->t_gt_0 = config.t_gt_0;

    // Vicon marker pose in the VO's world frame at t=0
    // composed with the camera->body and body->marker static extrinsics since camera to world is I at t=0
    Eigen::Matrix3d R_worldVO_marker0 = R_C_B0_eigen * R_B_V;
    Eigen::Vector3d t_worldVO_marker0 = R_C_B0_eigen * t_B_V + t_C_B0_eigen;

    // Maps world -> room by chaining (world->marker and marker->room)
    Eigen::Matrix3d R_gt0 = q_vicon0.toRotationMatrix();
    this->R_init = R_gt0 * R_worldVO_marker0.transpose();
    this->t_init = t_vicon0 - R_init * t_worldVO_marker0;
}

void StereoTracker::processFrame(string left_img_path, string right_img_path, uint64_t timestep)
{
    current_matches.clear();
    cv::Mat left_img = cv::imread(left_img_path, IMREAD_GRAYSCALE);
    cv::Mat right_img = cv::imread(right_img_path, IMREAD_GRAYSCALE);
    vector<cv::Point2f> kept_left_pts;
    vector<int> kept_ids;
    this->keyframe = false;

    if (left_img.empty() || right_img.empty())
    {
        cerr << "Could not open or find the images!" << endl;
        return;
    }

    if (prev_left_img.empty())
    {
        triangulate(left_img, right_img, prev_left_feature_pts, max_features, Mat(), kept_left_pts, kept_ids); // detect initial features
        this->prev_left_img = left_img;
        this->keyframe = true;
        return;
    }

    this->frame_id++;
    vector<Point2f> curr_left_pts;
    vector<uchar> status_temporal;
    vector<float> err_temporal;

    calcOpticalFlowPyrLK(prev_left_img, left_img, prev_left_feature_pts, curr_left_pts, 
                            status_temporal, err_temporal, window_size, max_level, term_crit); // track features across time

    vector<cv::Point2f> curr_right_pts;
    vector<uchar> status_stereo;
    vector<float> err_stereo;

    calcOpticalFlowPyrLK(left_img, right_img, curr_left_pts, curr_right_pts, 
                            status_stereo, err_stereo, window_size, max_level, term_crit); // track features across space

    for (size_t i = 0; i < prev_left_feature_pts.size(); i++) // temporal/spatial matches are ordered in index from prev_left_feature_pts
    {
        if (status_temporal[i] && status_stereo[i]) // check that feature is tracker across time and space (prev and right images)
        {
            Feature match;
            match.id = prev_feature_ids[i];
            match.left_pt = curr_left_pts[i];
            match.right_pt = curr_right_pts[i];
            current_matches.push_back(match);

            kept_left_pts.push_back(curr_left_pts[i]);
            kept_ids.push_back(prev_feature_ids[i]);
        }
    }

    getPose(timestep);
    bool features_low = kept_left_pts.size() < (size_t)(max_features * FEATURE_THRESH); // not enough tracked features
    if (features_low)
    {
        Mat mask = Mat::ones(left_img.size(), CV_8UC1); // one channel canvas of 1's size of image
        for (const auto& pt : kept_left_pts)
            circle(mask, pt, 10, 0, -1); // fill a circle of radius 10 with 0, this blocks out 10 px radius around existing features

        vector<Point2f> new_pts;
        int features_to_find = max_features - kept_left_pts.size();
        triangulate(left_img, right_img, new_pts, features_to_find, mask, kept_left_pts, kept_ids); // find new features with the mask
        this->keyframe = true;
    }

    if (this->keyframe)
    {
        this->numb_keyframes++;
        this->avg_mae += reprojectionError(kept_left_pts, kept_ids);
    }

    vector<cv::Point2f> filtered_left_pts;
    vector<int> filtered_ids;

    for (size_t i = 0; i < kept_ids.size(); i++)
    {
        if (!global_map.count(kept_ids[i]))
            continue;
        
        filtered_left_pts.push_back(kept_left_pts[i]);
        filtered_ids.push_back(kept_ids[i]);
    }

    prev_left_img = left_img;
    prev_left_feature_pts = filtered_left_pts;
    prev_feature_ids = filtered_ids;
}

void StereoTracker::visualizeMatch(string left_img_path, string right_img_path)
{
    cv::Mat img_left = cv::imread(left_img_path, IMREAD_GRAYSCALE);
    cv::Mat img_right = cv::imread(right_img_path, IMREAD_GRAYSCALE);

    if (img_left.empty() || img_right.empty())
    {
        cerr << "Could not open or find the images!" << endl;
        return;
    }
    if (img_left.rows != img_right.rows) 
    {
        double scale = static_cast<double>(img_left.rows) / img_right.rows;
        cv::Size new_size(static_cast<int>(img_right.cols * scale), img_left.rows);
        cv::resize(img_right, img_right, new_size);
    }

    for (auto& match : current_matches)
    {
        cv::circle(img_left, match.left_pt, 4, cv::Scalar(0), -1); // Black outer layer, white core
        cv::circle(img_left, match.left_pt, 2, cv::Scalar(255), -1);
        cv::circle(img_right, match.right_pt, 4, cv::Scalar(0), -1);
        cv::circle(img_right, match.right_pt, 2, cv::Scalar(255), -1);
    }
    cout<<"Number of matches: "<<current_matches.size()<<endl;

    cv::Mat combined_image;
    cv::hconcat(img_left, img_right, combined_image);
    cv::imshow("Stereo Cameras", combined_image);
    cv::waitKey(25);
}

void StereoTracker::triangulate(cv::Mat& left_img, cv::Mat& right_img, vector<Point2f>& left_pts, const int numb_features, const cv::Mat& mask,
                                vector<cv::Point2f>& added_left_pts, vector<int>& added_ids)
{
    if (mask.empty())
        cv::goodFeaturesToTrack(left_img, left_pts, numb_features, 0.01, 10); // detect initial features (initial frame)
    else
        cv::goodFeaturesToTrack(left_img, left_pts, numb_features, 0.01, 10, mask); // detect more features with mask (key frame)

    vector<cv::Point2f> right_pts;
    vector<uchar> status_stereo;
    vector<float> err_stereo;

    calcOpticalFlowPyrLK(left_img, right_img, left_pts, right_pts, 
                            status_stereo, err_stereo, window_size, max_level, term_crit); // track features across space

    vector<Point2f> kept_left_pts;
    vector<Point2f> kept_right_pts;

    for (size_t i = 0; i < left_pts.size(); i++)
    {
        if (status_stereo[i]) // filter out new features that exist in both left and right cameras
        {
            kept_left_pts.push_back(left_pts[i]);
            kept_right_pts.push_back(right_pts[i]);
        }
    }

    left_pts = kept_left_pts;
    if (this->prev_left_img.empty()) // ID initial features at t = 0
    {
        this->prev_feature_ids.resize(left_pts.size());
        for (size_t i = 0; i < left_pts.size(); i++)
        {
            this->prev_feature_ids[i] = this->next_id;
            this->next_id++;
        }
        this->prev_left_feature_pts = left_pts;
    }

    vector<Point2f> undistorted_left_pts;
    vector<Point2f> undistorted_right_pts;
    // stereoRectify would virtually transform both cameras into a common rectified plane to directly take the disparity (horizontal epipolar line)
    // this means camreas are coplanar, same orientation, baseline aligned along x-axis
    // undistortPoints transforms 2D point into normalized 3D point through K^-1 (3D point divided by z depth)
    undistortPoints(left_pts, undistorted_left_pts, K_left, D_left, R1, P1); // R is rotation matrix between each camera frame and rectified frame
    undistortPoints(kept_right_pts, undistorted_right_pts, K_right, D_right, R2, P2); // P is the projection matrix (intrinsics + position) between each camera frame and rectified frame
        
    double fx_rect = P1.at<double>(0, 0);
    double fy_rect = P1.at<double>(1, 1);
    double cx_rect = P1.at<double>(0, 2);
    double cy_rect = P1.at<double>(1, 2);
    double baseline = -P2.at<double>(0, 3) / P2.at<double>(0, 0);
    int numb_matches = min(kept_left_pts.size(), kept_right_pts.size());
    
    for (int i = 0; i < numb_matches; i++)
    {
        // can do this since left and right pixel coordinates are seperated by constant offset (x=200 in left image represents 200+disparity in right image)
        // only works if both cameras share same dimensions (length x width)
        double disparity_px = undistorted_left_pts[i].x - undistorted_right_pts[i].x;
        
        if (!std::isfinite(disparity_px) || disparity_px <= 1e-3) 
            continue; 

        // use z scaling factor to calculate 3D point
        double z = fx_rect * baseline / disparity_px;
        if (z < 0 || z > 15)
            continue;

        double x = (undistorted_left_pts[i].x - cx_rect) * z / fx_rect;
        double y = (undistorted_left_pts[i].y - cy_rect) * z / fy_rect;
        cv::Mat P_C_rect = (cv::Mat_<double>(3,1) << x, y, z); // 3D point in rectified frame
        cv::Mat P_C = R1.t() * P_C_rect; // 3D point in left camera frame
        cv::Mat P_W_mat = R_W_C * P_C + t_W_C; // 3D point in world frame
        cv::Point3d P_W(P_W_mat.at<double>(0), P_W_mat.at<double>(1), P_W_mat.at<double>(2));

        Landmark new_landmark;
        new_landmark.pos = P_W;
        if (this->prev_left_img.empty())
                global_map[prev_feature_ids[i]] = new_landmark; // For initial frame
        else 
            {
                global_map[this->next_id] = new_landmark; // For key frame
                added_left_pts.push_back(kept_left_pts[i]);
                added_ids.push_back(this->next_id);
                this->next_id++;
            }
    }
    publishMap();
}

void StereoTracker::publishMap()
{
    auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pc2_msg->header.stamp = rclcpp::Clock().now();
    pc2_msg->header.frame_id = "map";

    sensor_msgs::PointCloud2Modifier modifier(*pc2_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(global_map.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2_msg, "z");

    for (const auto& [id, pt] : global_map) // change to iterate from a to next_id then get the 3D pos
    {
        // match to rviz frame for pointcloud visualization
        *iter_x = static_cast<float>(pt.pos.x);
        *iter_y = static_cast<float>(pt.pos.y);
        *iter_z = static_cast<float>(pt.pos.z);

        ++iter_x; ++iter_y; ++iter_z;
    }

    this->point_cloud_pub->publish(*pc2_msg);
}

void StereoTracker::getPose(uint64_t timestep)
{
    std::vector<cv::Point3f> object_points; // 3D points in global map
    std::vector<cv::Point2f> image_points;  // 2D pixel locations
    std::vector<int> kept_ids;

    this->last_inlier_ids.clear();
    this->last_inlier_pts.clear();

    for (size_t i = 0; i < current_matches.size(); i++)
    {
        int id = current_matches[i].id;
        if (global_map.find(id) != global_map.end())
        {
            cv::Point3d pt3d = global_map[id].pos;
            object_points.push_back(cv::Point3f(pt3d.x, pt3d.y, pt3d.z));
            image_points.push_back(current_matches[i].left_pt);
            kept_ids.push_back(id);
        }
    }

    std::vector<int> inliers;
    if (object_points.size() >= 4)
    {
        // solvePnPRansac solves transformation matrix from world to camera frame
        // No need to propogate T since you're comparing to global map object points in world frame
        // inliers represent indices of points that are considered reliable (3D points and 2D points reliably match)
        bool success = cv::solvePnPRansac(object_points, image_points, K_left, D_left, this->rvec, this->tvec, true, 100, 3.0F, 0.99, inliers);

        if (success) 
        {
            std::vector<cv::Point3f> object_points_inliers;
            std::vector<cv::Point2f> image_points_inliers;
            vector<int> inlier_ids;
            vector<bool> is_inlier(kept_ids.size(), false);

            for (int &inlier : inliers) // keep feature and object inliers to feed into pose refinement
            {
                int id = kept_ids[inlier];
                inlier_ids.push_back(id);
                is_inlier[inlier] = true;
                object_points_inliers.push_back(object_points[inlier]);
                image_points_inliers.push_back(image_points[inlier]);
                global_map[id].outlier_count = 0;
            }

            this->last_inlier_ids = inlier_ids;
            this->last_inlier_pts = image_points_inliers;

            for (size_t i = 0; i < kept_ids.size(); i++) // landmarks RANSAC saw but rejected as outliers this frame
            {
                if (!is_inlier[i])
                    global_map[kept_ids[i]].outlier_count++;
            }

            cv::Mat R_C_W_provisional;
            cv::Rodrigues(this->rvec, R_C_W_provisional);
            camera_pose[this->frame_id] = { R_C_W_provisional.clone(), this->tvec.clone() }; // provisional camera pose to estimate parallax

            vector<Point2f> undistorted_left_inliers;
            undistortPoints(image_points_inliers, undistorted_left_inliers, K_left, D_left, Mat(), Mat()); // Pass in undistorted points in camera frame not rectified frame
            for (size_t a = 0; a < undistorted_left_inliers.size(); a++)
            {
                Observation feature_observation;
                feature_observation.frame_id = this->frame_id;
                feature_observation.frame_location = undistorted_left_inliers[a]; // store normalized coordinates that already used intrisic and distortion matrices

                auto& landmark = global_map[inlier_ids[a]];
                if (!landmark.observations.empty())
                {
                    double parallax = computeParallax(landmark.pos, landmark.observations.back(), feature_observation); // only add observations with large enough parallax
                    if (parallax < MIN_PARALLAX_APPEND_RAD)
                        continue;
                }

                auto& obs_vector = landmark.observations;
                if (obs_vector.size() >= MAX_OBSERVATIONS)
                    obs_vector.erase(obs_vector.begin()); // use sliding window if landmark observations exceeds limit (delete oldest, add newest)

                obs_vector.push_back(feature_observation);
            }
        
            for (auto it = global_map.begin(); it != global_map.end();) // erase landmarks that are considered outliers for 3 or more consecutive frames
            {
                if (it->second.outlier_count >= FRAME_THRESH)
                    it = global_map.erase(it);
                else
                    ++it;
            }

            cv::solvePnPRefineLM(object_points_inliers, image_points_inliers, K_left, D_left, this->rvec, this->tvec); // refine pose estimate by minimizing projection error using inliers from PnPRansac
            cv::Mat R_C_W;
            cv::Rodrigues(this->rvec, R_C_W);

            Pose inv_cam_pose;
            inv_cam_pose.translation = this->tvec.clone();
            inv_cam_pose.rotation = R_C_W.clone();
            camera_pose[this->frame_id] = inv_cam_pose;
            optimizeLandmark();

            // Invert to solve left camera -> world transformation
            // Current pose of left camera relative to world frame (starting pose of left camera)
            this->R_W_C = R_C_W.t();
            this->t_W_C = -R_W_C * this->tvec;

            Eigen::Matrix3d rotation;
            rotation << R_W_C.at<double>(0,0), R_W_C.at<double>(0,1), R_W_C.at<double>(0,2),
                        R_W_C.at<double>(1,0), R_W_C.at<double>(1,1), R_W_C.at<double>(1,2),
                        R_W_C.at<double>(2,0), R_W_C.at<double>(2,1), R_W_C.at<double>(2,2);
            Eigen::Quaterniond q_W_C(rotation); // body -> world
            
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp.sec = static_cast<int32_t>(timestep / 1000000000ULL);
            pose.header.stamp.nanosec = static_cast<uint32_t>(timestep % 1000000000ULL);
            pose.header.frame_id = "map";
            path.header.stamp = pose.header.stamp;

            // Match point cloud orientation in rviz
            pose.pose.position.x = t_W_C.at<double>(0);
            pose.pose.position.y = t_W_C.at<double>(1);
            pose.pose.position.z = t_W_C.at<double>(2);

            if (!path.poses.empty())
            {
                const auto& last_pos = path.poses.back().pose.position;
                Eigen::Vector3d curr(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                Eigen::Vector3d prev(last_pos.x, last_pos.y, last_pos.z);
                double dist = (curr - prev).norm();
                if (dist >= MAX_STEP_DIST)
                    return; // Too large a translation between last and current translation vectors
            }

            pose.pose.orientation.w = q_W_C.w();
            pose.pose.orientation.x = q_W_C.x();
            pose.pose.orientation.y = q_W_C.y();
            pose.pose.orientation.z = q_W_C.z();
            path.poses.push_back(pose);
            this->path_pub->publish(path);
        }
        else    
            cout<<"Frame "<<this->frame_id<<": Unable to solve pose estimation, ID: "<<this->next_id<<endl;
    }
    else
        cout<<"Frame "<<this->frame_id<<": Not enough matches to solve pose estimation"<<endl;

    cout<<"Frame "<<this->frame_id<<endl;
}

void StereoTracker::outputPose(string filename)
{
    std::ofstream csv_file(filename);
    csv_file << "timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z\n";
    csv_file << std::fixed << std::setprecision(6);

    cout << "VO estimate size: " << this->path.poses.size() << endl;

    Eigen::Quaterniond prev_q_room_V = Eigen::Quaterniond::Identity();
    bool have_prev_q_room_V = false;

    for (const auto &pose : this->path.poses)
    {
        // Camera pose in world frame
        Eigen::Vector3d p_W_C(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

        Eigen::Quaterniond q_W_C(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
        Eigen::Matrix3d R_W_C = q_W_C.toRotationMatrix();

        // Solves IMU body frame -> world transformation
        // Current pose of IMU body relative to world frame (starting pose of left camera)
        Eigen::Matrix3d R_W_B = R_W_C * R_C_B0_eigen;
        Eigen::Vector3d p_W_B = R_W_C * t_C_B0_eigen + p_W_C;

        // Vicon marker pose in the VO's world frame: compose the current body pose
        // with the marker's rigid mounting offset (T_BS from vicon0/sensor.yaml)
        Eigen::Matrix3d R_W_V = R_W_B * R_B_V;
        Eigen::Vector3d p_W_V = R_W_B * t_B_V + p_W_B;

        // Align into the Vicon room frame using the t=0 ground-truth anchor (vicon -> world then world -> room)
        Eigen::Matrix3d R_room_V = R_init * R_W_V;
        Eigen::Vector3d p_room_V = R_init * p_W_V + t_init;
        Eigen::Quaterniond q_room_V(R_room_V);

        if (p_room_V.norm() > 1e3)
            continue;

        if (have_prev_q_room_V && q_room_V.dot(prev_q_room_V) < 0.0) // -q = q, flip to keep sign continuous across frames
            q_room_V.coeffs() *= -1.0;
        prev_q_room_V = q_room_V;
        have_prev_q_room_V = true;
        uint64_t timestamp_ns = static_cast<uint64_t>(pose.header.stamp.sec) * 1000000000ULL
                            + static_cast<uint64_t>(pose.header.stamp.nanosec);

        csv_file << timestamp_ns << ","
                << p_room_V.x() << ","
                << p_room_V.y() << ","
                << p_room_V.z() << ","
                << q_room_V.w() << ","
                << q_room_V.x() << ","
                << q_room_V.y() << ","
                << q_room_V.z() << "\n";
    }
    csv_file.close();
}

double StereoTracker::reprojectionError(string left_img_path, const vector<Point2f> curr_left_feature_pts, const vector<int> curr_feature_ids)
{
    cv::Mat img_left = cv::imread(left_img_path, IMREAD_GRAYSCALE);
    if (img_left.empty())
    {
        cerr << "Could not open or find the image!" << endl;
        return 0;
    }

    vector<Point2d> image_points;
    vector<Point3d> object_points;
    vector<Point2d> projected_points;
    vector<int> used_ids;
    for (size_t a = 0; a < curr_feature_ids.size(); a++)
    {
        if (!global_map.count(curr_feature_ids[a]))
            continue;
        
        image_points.push_back(curr_left_feature_pts[a]);
        object_points.push_back(global_map[curr_feature_ids[a]].pos);
        used_ids.push_back(curr_feature_ids[a]);
    }

    double mean_abs_err = 0;
    cv::projectPoints(object_points, rvec, tvec, K_left, D_left, projected_points); // need to undistort and use intrinsic since reading in raw image points
    for (size_t i = 0; i < projected_points.size(); i++)
    {
        cv::line(img_left, image_points[i], projected_points[i], cv::Scalar(255), 1);
        cv::circle(img_left, image_points[i], 3, cv::Scalar(255), -1);
        cv::circle(img_left, projected_points[i], 3, cv::Scalar(0), -1);

        double error = norm(projected_points[i] - image_points[i]);

        if (error >= REPROJ_ERR)
            global_map[used_ids[i]].reprojection_count++;
        else
            global_map[used_ids[i]].reprojection_count = 0;
        
        if (global_map[used_ids[i]].reprojection_count >= FRAME_THRESH)
            global_map.erase(used_ids[i]);

        mean_abs_err += error;
    }
    mean_abs_err = projected_points.size() > 0 ? mean_abs_err / projected_points.size() : 0.0;
    cv::imshow("Reprojection error", img_left);
    cv::waitKey(25);
    return mean_abs_err;
}

double StereoTracker::reprojectionError(const vector<Point2f> curr_left_feature_pts, const vector<int> curr_feature_ids)
{
    vector<Point2d> image_points;
    vector<Point3d> object_points;
    vector<Point2d> projected_points;
    vector<int> used_ids;
    for (size_t a = 0; a < curr_feature_ids.size(); a++)
    {
        if (!global_map.count(curr_feature_ids[a]))
            continue;
        
        image_points.push_back(curr_left_feature_pts[a]);
        object_points.push_back(global_map[curr_feature_ids[a]].pos);
        used_ids.push_back(curr_feature_ids[a]);
    }

    double mean_abs_err = 0;
    cv::projectPoints(object_points, rvec, tvec, K_left, D_left, projected_points); // undistort as well since you triangulated with undistorted points
    for (size_t i = 0; i < projected_points.size(); i++)
    {
        double error = norm(projected_points[i] - image_points[i]);
        if (error >= REPROJ_ERR)
            global_map[used_ids[i]].reprojection_count++;
        else
            global_map[used_ids[i]].reprojection_count = 0;
        
        if (global_map[used_ids[i]].reprojection_count >= FRAME_THRESH)
            global_map.erase(used_ids[i]);

        mean_abs_err += error;
    }
    mean_abs_err = projected_points.size() > 0 ? mean_abs_err / projected_points.size() : 0.0;
    return mean_abs_err;
}

double StereoTracker::computeError(const cv::Point3d& pos, vector<Observation>& observations)
{
    double total_err = 0.0;
    cv::Mat P_W = (cv::Mat_<double>(3, 1) << pos.x, pos.y, pos.z);

    for (const auto& observation : observations)
    {
        Pose curr_camera_pose = camera_pose[observation.frame_id];
        cv::Mat P_C = curr_camera_pose.rotation * P_W + curr_camera_pose.translation;

        double zc = P_C.at<double>(2);
        if (zc <= 1e-4) continue;

        double u_proj = P_C.at<double>(0) / zc; // normalize xy by z depth for projected point since you stored normalized point in feature_observation.frame_location, no need for distortion or intrinsic matrices
        double v_proj = P_C.at<double>(1) / zc;

        cv::Mat residual = (cv::Mat_<double>(2, 1) <<
            (observation.frame_location.x - u_proj),
            (observation.frame_location.y - v_proj));

        total_err += cv::norm(residual);
    }
    return total_err;
};

double StereoTracker::computeParallax(const cv::Point3d& pos, const Observation& obs1, const Observation& obs2)
{
    cv::Mat P_W = (cv::Mat_<double>(3,1) << pos.x, pos.y, pos.z);

    const Pose& pose1 = camera_pose[obs1.frame_id];
    const Pose& pose2 = camera_pose[obs2.frame_id];

    cv::Mat C1 = -pose1.rotation.t() * pose1.translation; // camera center in world frame
    cv::Mat C2 = -pose2.rotation.t() * pose2.translation;

    cv::Mat ray1 = P_W - C1;
    cv::Mat ray2 = P_W - C2;

    double cos_parallax = ray1.dot(ray2) / (cv::norm(ray1) * cv::norm(ray2));
    cos_parallax = std::clamp(cos_parallax, -1.0, 1.0); // guard fp rounding past [-1,1]
    return std::acos(cos_parallax); // radians
}

void StereoTracker::optimizeLandmark()
{
    const int MAX_ITERATIONS = 5; // check parallax angle between firsst and last observation to see if worht optimizing

    for (auto& [id, landmark] : global_map)
    {
        if (landmark.observations.size() < OBSERVATION_THRESH) 
            continue;
        
        if (landmark.observations.back().frame_id <= landmark.last_optimized_frame) // no new observation since last optimization
            continue;
        
        double parallax = computeParallax(landmark.pos, landmark.observations.front(), landmark.observations.back());
        if (parallax < MIN_PARALLAX_REFINE_RAD) // parallax between oldestt and newest observations isnt enough to justfiy optimizing
            continue;
        
        double current_error = computeError(landmark.pos, landmark.observations);
        double lambda = 1e-4; // dx = b / lambda, the larger lambda, the smaller the step size correction
        landmark.last_optimized_frame = this->frame_id;
        for (int iter = 0; iter < MAX_ITERATIONS; ++iter)
        {
            double total_error = 0.0;
            double max_error = 0.0;
            double sum_zc = 0.0;
            int valid_obs = 0;

            cv::Mat H = cv::Mat::zeros(3, 3, CV_64F);
            cv::Mat b = cv::Mat::zeros(3, 1, CV_64F);

            for (const auto& observation : landmark.observations)
            {
                Pose curr_camera_pose = camera_pose[observation.frame_id];
                cv::Mat P_W = (cv::Mat_<double>(3, 1) << landmark.pos.x, landmark.pos.y, landmark.pos.z);
                cv::Mat P_C = curr_camera_pose.rotation * P_W + curr_camera_pose.translation;

                double xc = P_C.at<double>(0);
                double yc = P_C.at<double>(1);
                double zc = P_C.at<double>(2);

                if (zc <= 1e-4) 
                    continue; 
                
                double u_proj = xc / zc;
                double v_proj = yc / zc;

                cv::Mat J_inter = (cv::Mat_<double>(2, 3) << 
                    1/zc,   0.0, -xc/(zc*zc),
                      0.0, 1/zc, -yc/(zc*zc));

                cv::Mat J = -1.0 * J_inter * curr_camera_pose.rotation;
                
                cv::Mat residual = (cv::Mat_<double>(2, 1) << 
                    (observation.frame_location.x - u_proj),
                    (observation.frame_location.y - v_proj));

                double error = cv::norm(residual);
                total_error += error;
                max_error = max(max_error, error);
                sum_zc += zc;
                valid_obs++;

                H += J.t() * J;
                b += J.t() * residual;
            }

            cv::Mat H_damped = H + lambda * cv::Mat::eye(3, 3, CV_64F);
            cv::Mat dx;
            double mean_error = valid_obs > 0 ? total_error / valid_obs : 0.0;
            bool solved = cv::solve(H_damped, b, dx, cv::DECOMP_LU);
            if (!solved)
                break;

            double dx_norm = norm(dx);
            double avg_zc = valid_obs > 0 ? sum_zc / valid_obs : 1.0;
            const double MAX_DX_FRACTION = 0.5; // cap correction to a fraction of average landmark depth
            double max_dx = MAX_DX_FRACTION * avg_zc;
            if (dx_norm > max_dx)
            {
                dx *= (max_dx / dx_norm);
                dx_norm = max_dx;
            }

            Point3d candidate_pos = landmark.pos;
            candidate_pos.x -= dx.at<double>(0);
            candidate_pos.y -= dx.at<double>(1);
            candidate_pos.z -= dx.at<double>(2);
            double new_error = computeError(candidate_pos, landmark.observations);

            optimization_log
            << numb_keyframes << ","
            << id << ","
            << landmark.observations.size() << ","
            << iter << ","
            << mean_error << ","
            << max_error << ","
            << dx_norm << ","
            << landmark.pos.x << ","
            << landmark.pos.y << ","
            << landmark.pos.z
            << "\n";

            if (new_error < current_error)
            {
                landmark.pos = candidate_pos;
                current_error = new_error;
                lambda *= 0.5;
                if (cv::norm(dx) < 1e-3) // landmark has converged, stop iterations
                    break;
            }
            else
            {
                lambda *= 2.0;
            }
        }
    }
}