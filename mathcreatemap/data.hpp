#ifndef _DATA_
#define _DATA_

#include "math.hpp"

#include <array>
#include <chrono>

//Angles of the 16 individual lasers are provided by Velodyne documentation.
constexpr std::array<double, 16> documented_angles = 
		{ -15, 1, -13, -3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15 };

struct imu_data{
	double latitude, longitude, altitude;
	double pitch, yaw, roll;
	std::chrono::milliseconds time;
};

//Does not work if flown near the international date line or either pole
inline imu_data lerp_imu(const imu_data& start, const imu_data& end, double percent){
	imu_data toRet;
	
	toRet.latitude = lerp_value(start.latitude, end.latitude, percent);
	toRet.longitude = lerp_value(start.longitude, end.longitude, percent);
	toRet.altitude = lerp_value(start.altitude, end.altitude, percent);
	toRet.pitch = lerp_value(start.pitch, end.pitch, percent);
	toRet.yaw = lerp_value(start.yaw, end.yaw, percent);
	toRet.roll = lerp_value(start.roll, end.roll, percent);
	
	using namespace std::chrono;
	toRet.time = duration_cast<milliseconds>(start.time + ((end.time - start.time)*percent));
	
	return toRet;
}

inline double haversine_dist(const imu_data& start, const imu_data& end){
	
	auto sine_squared = [](double val) -> double {
		return sin(val) * sin(val);
	};		
	
	double lat_diff = ConvertToRadians(end.latitude - start.latitude);
	double lon_diff = ConvertToRadians(end.longitude - start.longitude);
	
	double left_under_rad = sine_squared(lat_diff / 2);
	double right_under_rad = sine_squared(lon_diff / 2);
	right_under_rad *= cos(ConvertToRadians(start.latitude)); 
	right_under_rad *= cos(ConvertToRadians(end.latitude));
	
	constexpr unsigned long radius_earth_meters = 6'371'000;
	return 2 * radius_earth_meters * asin(sqrt(left_under_rad + right_under_rad));
}

//x = longitude, y = latitude
inline vector3 between_imu_meters(const imu_data& start, const imu_data& end){
	imu_data fake_x_start; //x (longitude) is the same as the start
	fake_x_start.latitude = end.latitude;
	fake_x_start.longitude = start.longitude;
	
	double dist_long = haversine_dist(start, fake_x_start);
	
	imu_data fake_y_start;
	fake_y_start.latitude = start.latitude;
	fake_y_start.longitude = end.longitude;
	
	double dist_lat = haversine_dist(start, fake_y_start);
	
	return {dist_lat, dist_long, end.altitude - start.altitude};
}


struct lidar_data{
	std::array<double, 32> distance, reflectivity;
	double alpha; //azimuth
	std::chrono::milliseconds time;
};

#endif
