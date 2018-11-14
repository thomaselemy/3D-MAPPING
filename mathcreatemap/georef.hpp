#ifndef _GEOREF
#define _GEOREF

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

struct imu_data{
	double latitude, longitude, altitude;
	double pitch, yaw, roll;
	std::chrono::milliseconds time;
};

struct lidar_data{
	double distance, omega, alpha;
	std::chrono::milliseconds time;
};

struct vector3{
	double x, y, z;
	
	vector3 operator+ (const vector3& rhs) const {
		return {x + rhs.x, y + rhs.y, z + rhs.z};
	}
	
	vector3 operator- (const vector3& rhs) const {
		return {x - rhs.x, y - rhs.y, z - rhs.z};
	}
	
	vector3 operator* (double rhs) const {
		return {x * rhs, y * rhs, z * rhs};
	}
	
	void operator*= (double rhs){
		x *= rhs;
		y *= rhs;
		z *= rhs;
	}
};


constexpr inline double lerp_percent(long long start, long long end, 
									long long current){
	return static_cast<double>(current - start) / (end - start);
}

inline vector3 lerp(vector3 start, vector3 end, double percent){
	return start + ((end - start) * percent);
}

inline void print(const std::string& s){
    std::cout << s << std::endl;
}

//Angles of the 16 individual lasers are provided by Velodyne documentation.
constexpr std::array<double, 16> documentedAngles = 
		{ -15, 1, -13, -3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15 };

//BEGIN OLD WAY

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

using lidar_entry = std::array<double, 50>;
using imu_entry = std::array<double, 11>;

void georefMath(const std::vector<lidar_entry>& lidarData, 
				const std::vector<imu_entry>& imuData, const std::string& output);

#endif
