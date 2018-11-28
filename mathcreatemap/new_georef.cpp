#include "new_georef.hpp"

#include "math.hpp"

#include <chrono>

void georefMath(const std::vector<lidar_data>& lidar, 
				const std::vector<imu_data>& imu, 
				const std::string& filename_out){

	unsigned imu_behind_index = 0;

	imu_data first_used;
	
	for(unsigned lidar_index = 0; lidar_index < lidar.size(); lidar_index++){
	
		//TODO: IMU bound check
	
		using namespace std::chrono;
		milliseconds lidar_time = lidar[lidar_index].time;
		milliseconds behind_imu = imu[imu_behind_index].time;
		milliseconds ahead_imu = imu[imu_behind_index + 1].time;
	
		while(lidar_time < behind_imu){
			imu_behind_index--;
			behind_imu = imu.at(imu_behind_index).time;
			ahead_imu = imu.at(imu_behind_index + 1).time;
		}
	
		while(lidar_time > ahead_imu){
			imu_behind_index++;
			behind_imu = imu.at(imu_behind_index).time;
			ahead_imu = imu.at(imu_behind_index + 1).time;
		}
		
		if(lidar_index == 0){
			first_used = imu.at(imu_behind_index);
		}
		
		double lerp_pos = lerp_percent(behind_imu.count(), ahead_imu.count(), 
										lidar_time.count());
										
		imu_data local_imu = lerp_imu(imu[imu_behind_index], 
								imu.at(imu_behind_index + 1), lerp_pos);
	
		vector3 from_origin = between_imu_meters(first_used, local_imu);
	
	}	

}
