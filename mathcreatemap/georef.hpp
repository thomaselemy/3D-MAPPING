#ifndef _GEOREF
#define _GEOREF

#include "math.hpp"

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

//Angles of the 16 individual lasers are provided by Velodyne documentation.
constexpr std::array<double, 16> documentedAngles = 
		{ -15, 1, -13, -3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15 };

struct imu_data{
	double latitude, longitude, altitude;
	double pitch, yaw, roll;
	std::chrono::milliseconds time;
};

struct lidar_data{
	double distance, omega, alpha;
	std::chrono::milliseconds time;
	
	operator vector3(){
		double X = distance * cos(omega) * sin(alpha);
		double Y = distance * cos(omega) * cos(alpha);
		double Z = distance * sin(omega);
		return {X, Y, Z};
	}
};

//BEGIN OLD WAY

//Constants for data entry

using lidar_entry = std::array<double, 50>;

void georefMath(const std::vector<lidar_entry>& lidarData, 
				const std::vector<imu_data>& imuData, const std::string& output);

#endif
