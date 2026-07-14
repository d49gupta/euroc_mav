#include "vo.hpp"

StereoTracker::StereoTracker(string nodeName) : Node(nodeName) 
{
    this->point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vo/point_cloud", 10);
    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("/vo/path", 10);
    path.header.frame_id = "map";

    this->rvec = cv::Mat::zeros(3, 1, CV_64F);
    this->tvec = cv::Mat::zeros(3, 1, CV_64F);
    this->R_W_C = cv::Mat::eye(3, 3, CV_64F);
    this->t_W_C = cv::Mat::zeros(3, 1, CV_64F);
}

void StereoTracker::processFrame(string left_img_path, string right_img_path)
{
    current_matches.clear();
    cv::Mat left_img = cv::imread(left_img_path, IMREAD_GRAYSCALE);
    cv::Mat right_img = cv::imread(right_img_path, IMREAD_GRAYSCALE);
    vector<cv::Point2f> kept_left_pts;
    vector<int> kept_ids;
    bool keyframe = false;

    if (left_img.empty() || right_img.empty())
    {
        cerr << "Could not open or find the images!" << endl;
        return;
    }

    if (prev_left_img.empty())
    {
        triangulate(left_img, right_img, prev_left_feature_pts, max_features, Mat(), kept_left_pts, kept_ids); // detect initial features
        this->prev_left_img = left_img;
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

    getPose();
    if (kept_left_pts.size() < (size_t)(max_features * 0.7)) // if there arent enough tracked features, get new ones
    {
        Mat mask = Mat::ones(left_img.size(), CV_8UC1); // one channel canvas of 1's size of image
        for (const auto& pt : kept_left_pts)
            circle(mask, pt, 10, 0, -1); // fill a circle of radius 10 with 0, this blocks out 10 px radius around existing features
        
        vector<Point2f> new_pts;
        int features_to_find = max_features - kept_left_pts.size();
        triangulate(left_img, right_img, new_pts, features_to_find, mask, kept_left_pts, kept_ids); // find new features with the mask
        keyframe = true;
    }

    if (keyframe)
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
    undistortPoints(left_pts, undistorted_left_pts, K_left, D_left, Mat(), Mat()); // Pass in default rotation/translation since euroc cams are already rectifieds
    undistortPoints(kept_right_pts, undistorted_right_pts, K_right, D_right, Mat(), Mat());

    int numb_matches = min(kept_left_pts.size(), kept_right_pts.size());
    for (int i = 0; i < numb_matches; i++)
    {
        // can do this since left and right pixel coordinates are seperated by constant offset (x=200 in left image represents 200+disparity in right image)
        // only works if both cameras share same dimensions (length x width)
        double norm_disparity = undistorted_left_pts[i].x - undistorted_right_pts[i].x;
        
        if (!std::isfinite(norm_disparity) || norm_disparity <= 1e-3) 
            continue; 

        // use z scaling factor to calculate 3D point
        double z = b / norm_disparity;
        if (z < 0 || z > 5)
            continue;

        cv::Mat P_C = (cv::Mat_<double>(3, 1) << undistorted_left_pts[i].x * z, undistorted_left_pts[i].y * z, z); // 3D point in current left camera frame
        cv::Mat P_W_mat = R_W_C * P_C + t_W_C; // 3D point in world frame
        cv::Point3d P_W(P_W_mat.at<double>(0), P_W_mat.at<double>(1), P_W_mat.at<double>(2));

        Landmark new_landmark;
        new_landmark.pos = P_W;
        if (this->prev_left_img.empty())
                global_map[i] = new_landmark; // For initial frame
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
        *iter_x = static_cast<float>(pt.pos.z);
        *iter_y = static_cast<float>(pt.pos.x);
        *iter_z = static_cast<float>(-pt.pos.y);

        ++iter_x; ++iter_y; ++iter_z;
    }

    this->point_cloud_pub->publish(*pc2_msg);
}

void StereoTracker::getPose()
{
    std::vector<cv::Point3f> object_points; // 3D points in global map
    std::vector<cv::Point2f> image_points;  // 2D pixel locations
    std::vector<int> kept_ids;

    for (size_t i = 0; i < current_matches.size(); i++)
    {
        int id = current_matches[i].id;

        if (global_map.find(id) != global_map.end())
        {
            global_map[id].outlier_count++;
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
        bool success = cv::solvePnPRansac(object_points, image_points, K_left, D_left, this->rvec, this->tvec, false, 100, 10.0F, 0.99, inliers);
        // TODO: Write to csv for all frames the inliers, MAE, size of object points

        if (success) 
        {
            std::vector<cv::Point3f> object_points_inliers;
            std::vector<cv::Point2f> image_points_inliers;

            for (int &inlier : inliers)
            {
                object_points_inliers.push_back(object_points[inlier]);
                image_points_inliers.push_back(image_points[inlier]);
                global_map[kept_ids[inlier]].outlier_count = 0;
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
            pose.header.stamp = rclcpp::Clock().now();
            pose.header.frame_id = "map";
            path.header.stamp = pose.header.stamp;

            // Match point cloud orientation in rviz
            pose.pose.position.x = t_W_C.at<double>(2);
            pose.pose.position.y = t_W_C.at<double>(0);
            pose.pose.position.z = -t_W_C.at<double>(1);

            pose.pose.orientation.w = q_W_C.w();
            pose.pose.orientation.x = q_W_C.x();
            pose.pose.orientation.y = q_W_C.y();
            pose.pose.orientation.z = q_W_C.z();
            path.poses.push_back(pose);
            this->path_pub->publish(path);
        }
        else    
            cout<<"Frame "<<this->iteration<<": Unable to solve pose estimation, ID: "<<this->next_id<<endl;
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

    cout << "VO estimate size: " << this->path.poses.size() << endl;

    for (const auto &pose : this->path.poses)
    {
        // Camera pose in world frame
        Eigen::Vector3d p_W_C(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

        Eigen::Quaterniond q_W_C(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
        Eigen::Matrix3d R_W_C = q_W_C.toRotationMatrix();

        Eigen::Matrix3d R_C_B_eigen;
        R_C_B_eigen <<
            R_C_B.at<double>(0,0), R_C_B.at<double>(0,1), R_C_B.at<double>(0,2),
            R_C_B.at<double>(1,0), R_C_B.at<double>(1,1), R_C_B.at<double>(1,2),
            R_C_B.at<double>(2,0), R_C_B.at<double>(2,1), R_C_B.at<double>(2,2);

        Eigen::Vector3d t_C_B_eigen(
            t_C_B.at<double>(0),
            t_C_B.at<double>(1),
            t_C_B.at<double>(2));

        // Solves IMU body frame -> world transformation
        // Current pose of IMU body relative to world frame (starting pose of left camera)
        Eigen::Matrix3d R_W_B = R_W_C * R_C_B_eigen;
        Eigen::Vector3d p_W_B = R_W_C * t_C_B_eigen + p_W_C;

        // World -> Vicon
        Eigen::Vector3d p_V_B = R_init * p_W_B + t_init;
        Eigen::Quaterniond q_W_B(R_W_B);
        Eigen::Quaterniond q_V_B = q_init * q_W_B;

        if (p_V_B.norm() > 1e3)
            continue;

        csv_file << pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9 << ","
                << p_V_B.x() << ","
                << p_V_B.y() << ","
                << p_V_B.z() << ","
                << q_V_B.w() << ","
                << q_V_B.x() << ","
                << q_V_B.y() << ","
                << q_V_B.z() << "\n";
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
    cv::projectPoints(object_points, rvec, tvec, K_left, D_left, projected_points);
    for (size_t i = 0; i < projected_points.size(); i++)
    {
        cv::line(img_left, image_points[i], projected_points[i], cv::Scalar(255), 1);
        cv::circle(img_left, image_points[i], 3, cv::Scalar(255), -1);
        cv::circle(img_left, projected_points[i], 3, cv::Scalar(0), -1);

        double error = norm(projected_points[i] - image_points[i]);

        if (error > 10.0)
            global_map[used_ids[i]].reprojection_count++;
        else
            global_map[used_ids[i]].reprojection_count = 0;
        
        if (global_map[used_ids[i]].reprojection_count >= FRAME_THRESH)
            global_map.erase(used_ids[i]);

        mean_abs_err += error;
    }
    mean_abs_err = mean_abs_err / projected_points.size();
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
    cv::projectPoints(object_points, rvec, tvec, K_left, D_left, projected_points);
    for (size_t i = 0; i < projected_points.size(); i++)
    {
        double error = norm(projected_points[i] - image_points[i]);
        if (error > 10.0)
            global_map[used_ids[i]].reprojection_count++;
        else
            global_map[used_ids[i]].reprojection_count = 0;
        
        if (global_map[used_ids[i]].reprojection_count >= FRAME_THRESH)
            global_map.erase(used_ids[i]);

        mean_abs_err += error;
    }
    mean_abs_err = mean_abs_err / projected_points.size();
    return mean_abs_err;
}