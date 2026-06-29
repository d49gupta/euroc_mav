#include "vo.hpp"

void StereoTracker::undistort(const std::vector<cv::Point2f>& left_pts, const std::vector<cv::Point2f>& right_pts)
{
    undistorted_left_pts.clear();
    undistorted_right_pts.clear();
    undistortPoints(left_pts, undistorted_left_pts, K_left, D_left, Mat(), Mat());
    undistortPoints(right_pts, undistorted_right_pts, K_right, D_right, Mat(), Mat());
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
        cv::goodFeaturesToTrack(left_img, prev_left_feature_pts, max_features, 0.01, 10); // detect initial features
        prev_feature_ids.resize(prev_left_feature_pts.size());
        for (size_t i = 0; i < prev_left_feature_pts.size(); i++)
        {
            prev_feature_ids[i] = next_id;
            next_id++;
        }
        prev_left_img = left_img;   
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
    vector<cv::Point2f> kept_right_pts;
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
            kept_right_pts.push_back(curr_right_pts[i]);
            kept_ids.push_back(prev_feature_ids[i]);
        }
    }
    undistort(kept_left_pts, kept_right_pts);

    if (kept_left_pts.size() < (size_t)(max_features * 0.7)) // if there arent enough tracked features, get new ones
    {
        Mat mask = Mat::ones(left_img.size(), CV_8UC1); // one channel canvas of 1's size of image
        for (const auto& pt : kept_left_pts)
            circle(mask, pt, 10, 0, -1); // fill a circle of radius 10 with 0, this blocks out 10 px radius around existing features
        
        vector<Point2f> new_pts;
        int features_to_find = max_features - kept_left_pts.size();
        cv::goodFeaturesToTrack(left_img, new_pts, features_to_find, 0.01, 10, mask); // find new features with the mask
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