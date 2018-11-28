#ifndef _GEOREF
#define _GEOREF

#include "math.hpp"
#include "data.hpp"

#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <chrono>

//Tasks
//Get an origin from the first IMU packet
//Sync IMU and LiDAR coordinates
//??Math??
//Point cloud

//General formula:
//<Origin> + <Origin Offset> + <Point relative to drone> + <IMU-Lidar offset>

inline void print(const std::string& s){
    std::cout << s << std::endl;
}

//BEGIN OLD WAY

//Constants for data entry

using lidar_entry = std::array<double, 50>;

void georefMath(const std::vector<lidar_entry>& lidarData, 
				const std::vector<imu_data>& imuData, const std::string& output);

#endif
