#include "csv_reader.hpp"

int main() 
{
    csvReader reader("../vicon_room1/vicon_room1/V1_01_easy/V1_01_easy/mav0/imu0/data.csv");
    reader.readCSV();
    cout<<"Data size: "<<reader.data.size()<<endl;

    if (!reader.data.empty()) {
        reader.printRow(reader.data[0]);
        reader.printHeaders(reader.data[0]);
    } else {
        std::cout << "No data found in CSV." << std::endl;
    }

    return 0;
}