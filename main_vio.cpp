#include "csv_reader.hpp"
#include "vio.hpp"
#include "image_reader.hpp"
#include "filters.hpp"
#include "dataset_config.hpp"
#include <utility>

#define IMU_CALIB_SIZE 200.0
// Initial filtered IMU data with 2nd order Chebyshev LP filter
Chebyshev2Filter filter_gyro_x;
Chebyshev2Filter filter_gyro_y;
Chebyshev2Filter filter_gyro_z;

Chebyshev2Filter filter_accel_x;
Chebyshev2Filter filter_accel_y;
Chebyshev2Filter filter_accel_z;

void CHEBY_LP_FILT(IMUSample& data)
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const string dataset_root = "src/vicon_room1/vicon_room1/V1_01_easy/V1_01_easy";

    // read in IMU data
    IMUReader imu_reader(dataset_root + "/mav0/imu0/data.csv");
    imu_reader.readCSV();

    // read in camera image paths
    std::unique_ptr<imageReader> image_reader = std::make_unique<imageReader>(dataset_root);
    image_reader->datasetLoader();

    if (imu_reader.data.empty() || image_reader->frame_cam_names.empty())
    {
        std::cerr << "No data found." << std::endl;
        return 0;
    }

    // anchor dataset intrinsics/extrinsics/ground-truth to the first camera frame's timestamp
    DatasetConfig config = loadDatasetConfig(dataset_root, image_reader->frame_cam_timestamp[0][0]);

    int camera_frame = 0;
    auto estimator = std::make_shared<VisualInertialOdometry>("stereo_tracker", config);
    estimator->init_params();

    // initial feature detection and map, no state estimate available
    estimator->processFrame(image_reader->frame_cam_names[camera_frame][0], image_reader->frame_cam_names[camera_frame][1], image_reader->frame_cam_timestamp[camera_frame][0]);
    camera_frame++;

    Eigen::Vector3d sum_gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d sum_accel = Eigen::Vector3d::Zero();
    for (int a = 0; a < IMU_CALIB_SIZE; a++)
    {
        // CHEBY_LP_FILT(imu_reader.data[a]); // Filter IMU data in place (pass by reference)
        sum_gyro += imu_reader.data[a].gyro;
        sum_accel += imu_reader.data[a].accel;
    }

    // Initial state estimate through IMU calibration
    stateEstimate init_state;
    init_state.timestamp = imu_reader.data[IMU_CALIB_SIZE-1].timestamp;
    Eigen::Vector3d init_gyro_bias = sum_gyro / IMU_CALIB_SIZE;
    init_state.gyro_bias = init_gyro_bias;
    Eigen::Vector3d init_accel_bias = config.init_accel_bias; // initial accel bias from ground truth state estimate for better filtering
    init_state.accel_bias = init_accel_bias;
    estimator->state_estimate.push_back(init_state);
    estimator->last_estimate = init_state;
    estimator->g_ref = -sum_accel / IMU_CALIB_SIZE + init_accel_bias;
    int last_slam_frame = 0;

    // MAIN LOOP
    for (size_t index = IMU_CALIB_SIZE; index < imu_reader.data.size() && rclcpp::ok(); index++)
    // for (size_t index = IMU_CALIB_SIZE; index < 1000 && rclcpp::ok(); index++)
    {
        
        // CHEBY_LP_FILT(imu_reader.data[index]);
        estimator->applyPendingCorrection(); // pick up a solved loop-closure correction, if slamLoop() staged one
        uint64_t t_now = imu_reader.data[index].timestamp;
        uint64_t t_last = imu_reader.data[index-1].timestamp;
        double dt = static_cast<double>(t_now - t_last) / 1e9;
        estimator->predict(imu_reader.data[index], dt);
        estimator->update_covariance(imu_reader.data[index], dt);
        estimator->last_estimate.timestamp = t_now;

        if (static_cast<size_t>(camera_frame) < image_reader->frame_cam_timestamp.size() && imu_reader.data[index].timestamp >= image_reader->frame_cam_timestamp[camera_frame][0])        
        {
                uint64_t t_cam = image_reader->frame_cam_timestamp[camera_frame][0];
                estimator->processFrame(image_reader->frame_cam_names[camera_frame][0], image_reader->frame_cam_names[camera_frame][1], t_cam);
                estimator->update();
                cout<<"Frame "<<camera_frame<<", Estimate size: "<<estimator->state_estimate.size()<<endl;
                camera_frame++;
                int diff = estimator->frame_id - last_slam_frame;

                if (estimator->keyframe || diff >= MAX_KEYFRAME_GAP)
                {
                    estimator->enqueueKeyframe(estimator->createNode());
                    estimator->resetCovariance(estimator->P_keyframe);
                    last_slam_frame = estimator->frame_id;
                }

            }
            estimator->state_estimate.push_back(estimator->last_estimate);
            rclcpp::spin_some(estimator);
        }

    estimator->outputPose("src/build/vo_estimate.csv");
    estimator->outputState("src/build/vio_estimate.csv");
    rclcpp::shutdown();
    return 0;
}