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
    sys_time<milliseconds> toRet {round<milliseconds>(ms{val})};
    
	return toRet;
}

void georefMath(const std::vector<std::array<double, 50>>& lidarData, 
				const std::vector<std::array<double, 11>>& imuData,
				const std::string& output){

	std::ofstream ptCloudOFS(output);

	//Angles of the 16 individual lasers are provided by Velodyne documentation.
	constexpr std::array<double, 16> documentedAngles = 
		{ 15, -1, 13, 3, 11, -5, 9, -7, 7, -9, 5, -11, 3, -13, 1, -15 };
  
	std::array<double, 16> laserAngle{};
	
	std::transform(documentedAngles.begin(), documentedAngles.end(), 
		laserAngle.begin(), ConvertToRadians);

#pragma region VARIABLES FOR GEOREFERENCING MATH

    constexpr auto latOffset = 0;
    constexpr auto lonOffset = 0;
    
    constexpr auto testTime = std::chrono::microseconds(500);
    constexpr auto testAngle = 30;
    
    unsigned lRow = 0;			//for traversing LIDAR matrix rows
    unsigned lCol = 3;			//for traversing LIDAR matrix columns

#pragma endregion

#pragma region GEOREF MATH
    print("Start math");
				
    for (unsigned imuRow = 0; imuRow < imuData.size(); imuRow++){
	    //prevents loop from throwing an index oob error
        if (lRow + 1 >= lidarData.size()) { 
       		print("Index Out of Bounds"); break; 
       	} 
		
        //Might be 20 seconds behind imu data, so add 20 seconds
        long long lidarTime = lidarData[lRow][lCol] + 20000000l; //microseconds

		//Time stamps needed for time stamp synchronization
        //put the values on a comparable scale
        constexpr auto hour_floor = [](auto imu_time){
			using namespace std::chrono;
			return imu_time - date::floor<hours>(imu_time);
		};
        
        const auto imuTA_msPH = hour_floor(to_sys_time(imuData[imuRow][10]));
        const auto imuTB_msPH = hour_floor(to_sys_time(imuData[imuRow + 1][10]));
        
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

		//Put here for the auto parameters
		constexpr auto differ = [](auto imuTime, auto lidarTime){
			return abs(imuTime - microseconds(lidarTime));
		};

		//while the lidarTime is between the two imu ts, keep incrementing through lidarTime
		while (microseconds(lidarTime) >= imuTA_msPH 
			&& microseconds(lidarTime) < imuTB_msPH){
        
            //will store the row number for which IMU data values to do the georef math with
			auto imuRowSelect = imuRow + 1;
			
			//Assume B is closer
			auto difference = differ(imuTB_msPH, lidarTime);
			
            if (differ(imuTA_msPH, lidarTime) <= difference) { 

				//lidarTime is closer to imuA than imuB
               	imuRowSelect = imuRow; 
				difference = differ(imuTA_msPH, lidarTime);
         
            }

            if (difference < testTime) {

                //begin pt cloud math
                //Unused commented out
                //auto lat = imuData[imuRowSelect][0];
                //auto lon = imuData[imuRowSelect][1];
                //auto alt = imuData[imuRowSelect][2];
                auto roll = ConvertToRadians(imuData[imuRowSelect][7]);
                auto pitch = ConvertToRadians(imuData[imuRowSelect][8]);
                auto yaw = ConvertToRadians(imuData[imuRowSelect][9]);
                
                auto alpha = ConvertToRadians(lidarData[lRow][0] / 100);
                auto distance = lidarData[lRow][lCol - 2];
                //auto timeStamp = lidarData[lRow][lCol];
                auto omega = laserAngle.at((lCol / 3) - 1);

                if (distance == 0) {	//skipping the data point if the distance is zero
                    lCol = lCol + 3;	//the next data point's timestamp is three columns away. Refer to the Matrix organization document
                    if (lCol > 48) { lRow++; lCol = 3; }

                    lidarTime = lidarData[lRow][lCol];
                    continue;
                }

                auto X = distance * sin(alpha) * cos(omega);
                auto Y = distance * cos(omega) * cos(alpha);
                auto Z = -distance * sin(omega);

               	//auto X1 = X * cos(yaw) - Y * sin(yaw);
                //auto Y1 = X * sin(yaw) + Y * cos(yaw);
               
                //X transform (pitch + y_offset)
                auto X1 = X;
                auto Y1 = Y * cos(pitch) - Z * sin(pitch);
                auto Z1 = Y * sin(pitch) + Z * cos(pitch);

                //Y transform (roll)
                X = X1 * cos(roll) - Z1 * sin(roll);
                Y = Y1;
                Z = -X1 * sin(roll) + Z1 * cos(roll);

                //Z transform (yaw)
                X1 = X * cos(yaw) - Y * sin(yaw);
                Y1 = X * sin(yaw) + Y * cos(yaw);
                //Z1 = Z;

				constexpr auto altOffset = 0;
                //Position offset
                X1 += lonOffset;
                Y1 -= latOffset;
                //Z1 += altOffset;
                
                using namespace std;
                ptCloudOFS << setw(12) << right << setprecision(5) << fixed 
                << X1 << " " << setw(12) << right << setprecision(5) << fixed 
               	<< Y1 << " " << setw(12) << right << setprecision(5) << fixed 
                << Z << " " << setw(12) << right << setprecision(3);
                
                if (ConvertToDegrees(yaw) > testAngle) {
                	ptCloudOFS << 100 << endl;
                } else {
                	ptCloudOFS << 0 << endl;
                }
                //end pt cloud math

                //increment lidarTime here
                lCol += 3;	//the next data point's timestamp is three columns away. Refer to the Matrix organization document
                if (lCol > 48) { lRow++; lCol = 3; }

                lidarTime = lidarData[lRow][lCol];

            } else {
                //increment lidarTime here
                lCol += 3;	//the next data point's timestamp is three columns away. Refer to the Matrix organization document
                if (lCol > 48) { lRow++; lCol = 3; }

                lidarTime = lidarData[lRow][lCol];
                std::cout << "lidartime: " << lidarTime;
            }
        }
    }
    
#pragma endregion
    
	ptCloudOFS.close();
}
