#include "vo.hpp"

StereoTracker::StereoTracker(string nodeName) : Node(nodeName) 
{
    this->point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vo/point_cloud", 10);
    this->R_W_C = cv::Mat::eye(3, 3, CV_64F);
    this->t_W_C = cv::Mat::zeros(3, 1, CV_64F);
    // TODO: For initial frame, rotate to body->world frame and append that to estimate vector and also give in initial timestamp
}

void StereoTracker::processFrame(string left_img_path, string right_img_path)
{
    current_matches.clear();
    cv::Mat left_img = cv::imread(left_img_path, IMREAD_GRAYSCALE);
    cv::Mat right_img = cv::imread(right_img_path, IMREAD_GRAYSCALE);

    if (left_img.empty() || right_img.empty())
    {
        cerr << "Could not open or find the images!" << endl;
        return;
    }

    if (prev_left_img.empty())
    {
        triangulate(left_img, right_img, prev_left_feature_pts, max_features, Mat()); // detect initial features
        return;
    }

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

    vector<cv::Point2f> kept_left_pts;
    vector<int> kept_ids;
    for (size_t i = 0; i < prev_left_feature_pts.size(); i++)
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

    if (kept_left_pts.size() < (size_t)(max_features * 0.7)) // if there arent enough tracked features, get new ones
    {
        Mat mask = Mat::ones(left_img.size(), CV_8UC1); // one channel canvas of 1's size of image
        for (const auto& pt : kept_left_pts)
            circle(mask, pt, 10, 0, -1); // fill a circle of radius 10 with 0, this blocks out 10 px radius around existing features
        
        vector<Point2f> new_pts;
        int features_to_find = max_features - kept_left_pts.size();
        triangulate(left_img, right_img, new_pts, features_to_find, mask); // find new features with the mask
        for (const auto& pt : new_pts)
        {
            kept_left_pts.push_back(pt);
            kept_ids.push_back(next_id);
            next_id++;
        }
    }
    prev_left_img = left_img;
    prev_left_feature_pts = kept_left_pts;
    prev_feature_ids = kept_ids;
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

void StereoTracker::triangulate(cv::Mat& left_img, cv::Mat& right_img, vector<Point2f>& left_pts, const int numb_features, const cv::Mat& mask)
{
    if (mask.empty())
        cv::goodFeaturesToTrack(left_img, left_pts, numb_features, 0.01, 10); // detect initial features
    else
        cv::goodFeaturesToTrack(left_img, left_pts, numb_features, 0.01, 10, mask);

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
        for (size_t i = 0; i < this->prev_left_feature_pts.size(); i++)
        {
            this->prev_feature_ids[i] = this->next_id;
            this->next_id++;
        }
        this->prev_left_img = left_img;
    }

    vector<Point2f> undistorted_left_pts;
    vector<Point2f> undistorted_right_pts;
    // transforms 2D point into normalized 3D point through K^-1
    undistortPoints(left_pts, undistorted_left_pts, K_left, D_left, Mat(), Mat());
    undistortPoints(right_pts, undistorted_right_pts, K_right, D_right, Mat(), Mat());

    int numb_matches = min(kept_left_pts.size(), kept_right_pts.size());
    for (int i = 0; i < numb_matches; i++)
    {
        // can do this since left and right pixel coordinates are seperated by constant offset (x=200 in left image represents 200+disparity in right image)
        // only works if both cameras share same dimensions (length x width)
        double norm_disparity = undistorted_left_pts[i].x - undistorted_right_pts[i].x;
        
        if (norm_disparity <= 1e-6) 
            continue; 

        // use z scaling factor to calculate 3D point
        double z = b / norm_disparity;
        cv::Mat P_C = (cv::Mat_<double>(3, 1) << undistorted_left_pts[i].x * z, undistorted_left_pts[i].y * z, z); // 3D point in current left camera frame
        cv::Mat P_W_mat = R_W_C * P_C + t_W_C; // 3D point in world frame
        cv::Point3d P_W(P_W_mat.at<double>(0), P_W_mat.at<double>(1), P_W_mat.at<double>(2));
        global_map[prev_feature_ids[i]] = P_W; // 3D points in saved to global map in world frame
    }
    // publishPointCloud();
}

void StereoTracker::publishPointCloud()
{
    auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pc2_msg->header.stamp = rclcpp::Clock().now();
    pc2_msg->header.frame_id = "camera_left_frame";

    // FIX: PointCloud2Modifier lives directly under sensor_msgs, not msg!
    sensor_msgs::PointCloud2Modifier modifier(*pc2_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(global_map.size());

    // FIX: PointCloud2Iterator lives directly under sensor_msgs!
    sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2_msg, "z");

    for (const auto& [id, pt] : global_map)
    {
        // match to rviz frame for pointcloud visualization
        *iter_x = static_cast<float>(pt.z);
        *iter_y = static_cast<float>(pt.x);
        *iter_z = static_cast<float>(-pt.y);

        ++iter_x; ++iter_y; ++iter_z;
    }

    this->point_cloud_pub->publish(*pc2_msg);
}

void StereoTracker::getPose(double timestamp)
{
    std::vector<cv::Point3f> object_points; // 3D points in global map
    std::vector<cv::Point2f> image_points;  // 2D pixel locations

    for (size_t i = 0; i < current_matches.size(); i++)
    {
        int id = current_matches[i].id;

        if (global_map.find(id) != global_map.end())
        {
            cv::Point3d pt3d = global_map[id];
            object_points.push_back(cv::Point3f(pt3d.x, pt3d.y, pt3d.z));
            image_points.push_back(current_matches[i].left_pt);
        }
    }
    cv::Mat rvec, tvec;
    std::vector<int> inliers;
    if (object_points.size() >= 4)
    {
        // solvePnPRansac solves transformation matrix from world to camera frame
        // No need to propogate T since you're comparing to global map object points in world frame
        bool success = cv::solvePnPRansac(object_points, image_points, K_left, D_left, rvec, tvec, false, 100, 8.0F, 0.99,inliers);

        if (success) 
        {
            cv::Mat R_C_W;
            cv::Rodrigues(rvec, R_C_W);

            // Invert to solve left camera -> world transformation
            // Current pose of left camera relative to world frame (starting pose of left camera)
            this->R_W_C = R_C_W.t();
            this->t_W_C = -R_W_C * tvec;

            // Solves IMU body frame -> world transformation
            // Current pose of IMU body relative to world frame (starting pose of left camera)
            cv::Mat R_W_B = R_W_C * R_C_B;
            cv::Mat t_W_B = R_W_C * t_C_B + t_W_C;

            Eigen::Vector3d p_W_B(t_W_B.at<double>(0), t_W_B.at<double>(1), t_W_B.at<double>(2)); // body -> world
            Eigen::Matrix3d rotation;
            rotation << R_W_B.at<double>(0,0), R_W_B.at<double>(0,1), R_W_B.at<double>(0,2),
                        R_W_B.at<double>(1,0), R_W_B.at<double>(1,1), R_W_B.at<double>(1,2),
                        R_W_B.at<double>(2,0), R_W_B.at<double>(2,1), R_W_B.at<double>(2,2);
            Eigen::Quaterniond q_W_B(rotation); // body -> world

            Pose curr_pose;
            curr_pose.timestamp = timestamp;
            curr_pose.pos = p_W_B;
            curr_pose.orient = q_W_B;
            this->estimate.push_back(curr_pose);
        }
        else    
            cout<<"Frame "<<this->iteration<<": Unable to solve pose estimation"<<endl;
    }
    else
        cout<<"Frame "<<this->iteration<<": Not enough matches to solve pose estimation"<<endl;

    this->iteration++;
}

void StereoTracker::outputPose(string filename)
{
    std::ofstream csv_file(filename);
    csv_file << "timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z\n";
    csv_file << std::fixed << std::setprecision(6);

    cout<<"VO estimate size: "<<this->estimate.size()<<endl;
    for (auto pose : this->estimate)
    {
        Eigen::Vector3d vicon_pos = (R_init * pose.pos) + t_init;
        Eigen::Quaterniond vicon_orient = q_init * pose.orient;
        csv_file << pose.timestamp << ","<< vicon_pos.x() << ","<< vicon_pos.y() << ","<< vicon_pos.z() << ","
        << vicon_orient.w() << ","<< vicon_orient.x() << ","<< vicon_orient.y() << ","<< vicon_orient.z() << "\n";
    }
    csv_file.close();
}