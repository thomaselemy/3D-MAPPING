#ifndef _NEW_GEOREF_
#define _NEW_GEOREF_

#include "data.hpp"

#include <string>
#include <vector>

void georefMath(const std::vector<lidar_data>& lidar, 
				const std::vector<imu_data>& imu, const std::string& filename_out);

#endif
