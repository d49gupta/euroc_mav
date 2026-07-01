#include "vo.hpp"

StereoTracker::StereoTracker(string nodeName) : Node(nodeName) 
{
    this->point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vo/point_cloud", 10);
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
    undistortPoints(left_pts, undistorted_left_pts, K_left, D_left, Mat(), Mat());
    undistortPoints(right_pts, undistorted_right_pts, K_right, D_right, Mat(), Mat());

    int numb_matches = min(kept_left_pts.size(), kept_right_pts.size());
    for (int i = 0; i < numb_matches; i++)
    {
        double norm_disparity = undistorted_left_pts[i].x - undistorted_right_pts[i].x;
        
        if (norm_disparity <= 1e-6) 
            continue; 

        double z = b / norm_disparity;
        cv::Point3d point_3d(undistorted_left_pts[i].x * z, undistorted_left_pts[i].y * z, z); // decide if you want to transform from left to body frame
        global_map[prev_feature_ids[i]] = point_3d;
    }
}

void StereoTracker::publishPointCloud()
{
    auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pc2_msg->header.stamp = rclcpp::Clock().now();
    pc2_msg->header.frame_id = "map";

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
        *iter_x = static_cast<float>(pt.x);
        *iter_y = static_cast<float>(pt.y);
        *iter_z = static_cast<float>(pt.z);

        ++iter_x; ++iter_y; ++iter_z;
    }

    this->point_cloud_pub->publish(*pc2_msg);
}