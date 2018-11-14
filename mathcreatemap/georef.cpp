#include "georef.hpp"
#include "math.hpp"
//Documentation: https://howardhinnant.github.io/date/tz.html
#include "date/tz.h"
//System libraries
#include <chrono>
#include <fstream>
#include <iomanip>

inline auto to_sys_time(const double val){
	
	using namespace std::chrono;
	using ms = duration<double, std::milli>;
	
    using namespace date;
    sys_time<milliseconds> toRet(round<milliseconds>(ms{val}));
	return toRet;
}

void point_cloud_math(std::ostream& output, const lidar_entry& lidar, 
				const imu_entry& imu, double omega, double distance){
	
	//begin pt cloud math (done in radians)
	const auto alpha = ConvertToRadians(lidar[0] / 100);

	auto X = distance * cos(omega) * sin(alpha);
	auto Y = distance * cos(omega) * cos(alpha);
	auto Z = distance * sin(omega);
               
	const auto pitch = ConvertToRadians(imu[imu_entry_index::pitch]);
	//X transform (pitch + y_offset)
	auto X1 = X;
	auto Y1 = Y * cos(pitch) - Z * sin(pitch);
	auto Z1 = Y * sin(pitch) + Z * cos(pitch);

	const auto roll = ConvertToRadians(imu[imu_entry_index::roll]);
	//Y transform (roll)
	X = X1 * cos(roll) - Z1 * sin(roll);
	Y = Y1;
	Z = -X1 * sin(roll) + Z1 * cos(roll);

	const auto yaw = ConvertToRadians(imu[imu_entry_index::yaw]);
	//Z transform (yaw)
	X1 = X * cos(yaw) - Y * sin(yaw);
    Y1 = X * sin(yaw) + Y * cos(yaw);
       
    constexpr auto latOffset = 0;
    constexpr auto lonOffset = 0;
	constexpr auto altOffset = 0;
    //Position offset
    X1 += lonOffset;
    Y1 -= latOffset;
               
    using namespace std;
    output << setw(12) << right << setprecision(5) << fixed << X1 << " " 
    << setw(12) << right << setprecision(5) << fixed << Y1 << " " 
    << setw(12) << right << setprecision(5) << fixed << Z << " " 
    << setw(12) << right << setprecision(3);
                
    //Both are in radians
    if (yaw > ConvertToRadians(30)) {
    	output << 100 << endl;
    } else {
		output << 0 << endl;
    }
    //end pt cloud math
}

void georefMath(const std::vector<lidar_entry>& lidarData, 
				const std::vector<imu_entry>& imuData,
				const std::string& output){

	std::ofstream ptCloudOFS(output);
	std::array<double, documentedAngles.size()> laserAngle{};
	
	std::transform(documentedAngles.begin(), documentedAngles.end(), 
		laserAngle.begin(), ConvertToRadians);

#pragma region VARIABLES FOR GEOREFERENCING MATH

    constexpr auto testTime = std::chrono::microseconds(500);
    
    unsigned lRow = 0;			//for traversing LIDAR matrix rows
    unsigned lCol = 3;			//for traversing LIDAR matrix columns

#pragma endregion

#pragma region GEOREF MATH
    print("Start math");
				
    for (unsigned imuRow = 0; imuRow < imuData.size() - 1; imuRow++){
	    //prevents loop from throwing an index oob error
        if (lRow + 1 >= lidarData.size()) { 
       		print("Index Out of Bounds"); break; 
       	} 
		
        //Might be 20 seconds behind imu data, so add 20 seconds
        long long lidarTime = lidarData[lRow][lCol] + 20000000l; //microseconds

		//Time stamps needed for time stamp synchronization
        //put the values on a comparable scale
        const auto hour_floor = [](auto imu_time){
			using namespace std::chrono;
			return imu_time - date::floor<hours>(imu_time);
		};
        
        const auto imuTA_msPH = hour_floor(to_sys_time(
        						imuData[imuRow][imu_entry_index::time]
        						));
        const auto imuTB_msPH = hour_floor(to_sys_time(
        						imuData[imuRow + 1][imu_entry_index::time]
        						));
        
		using namespace date;
		using namespace std::chrono;
		//go to next lidarTime until it's greater than imuTimeA
        while (microseconds(lidarTime) < imuTA_msPH){ 
			
			//The next data point's timestamp is three columns away. 
			//Refer to the Matrix organization document
            lCol += 3;	

			//lCol has reached the end of the row
            if (lCol >= lidarData[lRow].size()){ 
                lRow++;
                lCol = 3;
            }

            lidarTime = lidarData[lRow][lCol];
        }

		//Put here for the auto parameter
		const auto differ = [](auto imuTime, long long lidarTime){
			return abs(imuTime - microseconds(lidarTime));
		};

		//while the lidarTime is between the two imu ts, keep incrementing through lidarTime
		while (microseconds(lidarTime) >= imuTA_msPH 
			&& microseconds(lidarTime) < imuTB_msPH){
        
            //Store the row number for which IMU data values to do the math with
			
			//Assume B is closer
			auto imuRowSelect = imuRow + 1;
			auto difference = differ(imuTB_msPH, lidarTime);
			
            if (differ(imuTA_msPH, lidarTime) <= difference) { 

				//lidarTime is closer to imuA than imuB
               	imuRowSelect = imuRow; 
				difference = differ(imuTA_msPH, lidarTime);
            }

            if (difference < testTime) {

                const auto distance = lidarData[lRow][lCol - 2];
               
                if (distance == 0) {	//skipping the data point if the distance is zero
                    lCol += 3;	//the next data point's timestamp is three columns away. Refer to the Matrix organization document
                    if (lCol >= lidarData[lRow].size()) { lRow++; lCol = 3; }

                    lidarTime = lidarData[lRow][lCol];
                    continue;
                }

				point_cloud_math(ptCloudOFS, lidarData[lRow], 
								imuData[imuRowSelect], 
								laserAngle.at((lCol / 3) - 1), distance
								);

                //increment lidarTime here
                lCol += 3;	//the next data point's timestamp is three columns away. Refer to the Matrix organization document
                if (lCol >= lidarData[lRow].size()) { lRow++; lCol = 3; }

                lidarTime = lidarData[lRow][lCol];

            } else {
                //increment lidarTime here
                lCol += 3;	//the next data point's timestamp is three columns away. Refer to the Matrix organization document
                if (lCol >= lidarData[lRow].size()) { lRow++; lCol = 3; }

                lidarTime = lidarData[lRow][lCol];
                std::cout << "lidartime: " << lidarTime << std::endl;
            }
        }
    }
    
#pragma endregion
    
	ptCloudOFS.close();
}
