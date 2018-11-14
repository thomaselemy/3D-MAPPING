#ifndef _GEOREF
#define _GEOREF

#include <iostream>
#include <vector>
#include <array>
#include <string>

//Tasks
//Get an origin from the first IMU packet
//Sync IMU and LiDAR coordinates
//??Math??
//Point cloud

//Constants for data entry
namespace imu_entry_index{
	constexpr auto latitude = 0;
	constexpr auto longitude = 1;
	constexpr auto altitude = 2;
	constexpr auto w = 3;
	constexpr auto x = 4;
	constexpr auto y = 5;
	constexpr auto z = 6;
	constexpr auto roll = 7;
	constexpr auto pitch = 8;
	constexpr auto yaw = 9;
	constexpr auto time = 10;
}

inline void print(const std::string& s){
    std::cout << s << std::endl;
}

using lidar_entry = std::array<double, 50>;
using imu_entry = std::array<double, 11>;

void georefMath(const std::vector<lidar_entry>& lidarData, 
				const std::vector<imu_entry>& imuData, const std::string& output);

#endif
