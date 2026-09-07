#include "csv_reader.hpp"
#include "ahrs.hpp"

int main() 
{
    IMUReader imu_reader("src/vicon_room1/vicon_room1/V1_01_easy/V1_01_easy/mav0/imu0/data.csv");
    GTReader gt_reader("src/vicon_room1/vicon_room1/V1_01_easy/V1_01_easy/mav0/vicon0/data.csv");

    imu_reader.readCSV();
    cout<<"Data size: "<<imu_reader.data.size()<<endl;

    gt_reader.readCSV();
    cout<<"Data size: "<<gt_reader.gt_orient_data.size()<<endl;

    if (imu_reader.data.empty() || gt_reader.gt_orient_data.empty())
        std::cout << "No data found in CSV." << std::endl;

    // iter1 type is BASE_AHRS since its the abstract class, so iter1 can take the form of any class
    // that inherits abstract so long as it uses methods from the abstract class only no specific ones
    // from inherited classes. This means assuming methods are in abstarct class, it can call method from
    // two different inherited classes without changing type. Note that if ahrs_estimate calls a custom non virtual 
    // class in one inherited class, using the base type to call this ahrs_estimate will still work
    std::unique_ptr<MADGWICK_AHRS> iter1 = std::make_unique<MADGWICK_AHRS>(imu_reader.data);
    iter1->ahrs_estimate();
    iter1->estimate_position();
    iter1->align_estimate();

    iter1->outputPose("src/build/imu_estimate.csv", iter1->timestamp, iter1->pos_estimate, iter1->euler_estimate);
    iter1->outputPose("src/build/vicon_estimate.csv", gt_reader.gt_timestamp, gt_reader.gt_pos_data, gt_reader.gt_euler_data);
    iter1->quat_output("src/build/quat_estimate.csv", iter1->timestamp, iter1->orient_estimate);
    return 0;
}