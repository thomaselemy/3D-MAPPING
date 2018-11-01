#include "georef.hpp"
#include "math.hpp"
//The 1 library I want to remove
#include "tz.h"
//System libraries
#include <chrono>
#include <fstream>
#include <iomanip>

void georefMath(const std::vector<std::array<double, 50>>& lidarData, 
				const std::vector<std::array<double, 11>>& imuData,
				const std::string& output){

	std::ofstream ptCloudOFS(output);
	std::ofstream test("testData.txt");

	//Angles of the 16 individual lasers are provided by Velodyne documentation.
    //double laserAngle[16] = { 105, 89, 103, 93, 101, 85, 99, 83, 97, 81, 95, 79, 93, 77, 91, 75 };
    //double laserAngle[16] = { 15, -1, 13,   3, 11,  -5, 9,  -7, 7, -9, 5,  -11, 3, -13, 1, -15 };
    //guess: the bottom array is just the top - 90
	std::array<double, 16> laserAngle = { 15, -1, 13, 3, 11, -5, 9, -7, 7, -9, 5, -11, 3, -13, 1, -15 };
    for (unsigned ctr = 0; ctr < laserAngle.size(); ctr++){
        laserAngle[ctr] = ConvertToRadians(laserAngle[ctr]);
    }

#pragma region VARIABLES FOR GEOREFERENCING MATH
    double lat = 0;
    double lon = 0;
    double alt = 0;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    double latLength = 0;
    double lonLength = 0;

    double latOffset = 0;
    double lonOffset = 0;

    double imuTimeA = 0;	//IMU time stamp A for time stamp synchronization
    double imuTimeB = 0;	//IMU time stamp B for time stamp synchronization
    long long lidarTime = 0;	//LIDAR time stamp for time stamp synchronization
    int imuRowSelect = 0;	//will store the row number for which IMU data values to do the georef math with
    int lRow = 0;			//for traversing LIDAR matrix rows
    int lCol = 3;			//for traversing LIDAR matrix columns

    double alpha = 0;
    double distance = 0;
    double timeStamp = 0;
    double omega = 0;
    double X = 0;
    double Y = 0;
    double Z = 0;
    int lzr = 0;
#pragma endregion

#pragma region GEOREF MATH
    print("***Start math");

    bool timeFlag;
    constexpr auto testTime = 500;
    constexpr auto testAngle = 30;
				
    for (int imuRow = 0; imuRow < imuData.size(); imuRow++){
	    //prevents loop from throwing an index oob error
        if (imuRow + 1 >= imuData.size() || lRow + 1 >= lidarData.size()) { 
       		print("IOOB SAVE"); break; 
       	} 

		//LET'S GET THOSE TIMESTAMPS
        imuTimeA = imuData[imuRow][10];
        imuTimeB = imuData[imuRow + 1][10];
        //might be 20 seconds behind imu data
        lidarTime = lidarData[lRow][lCol] + 20000000; 

        //put the values on a comparable scale
        using namespace std::chrono;
        using ms = duration<double, std::milli>;
        
        using namespace date;
        sys_time<milliseconds> imuTA{ round<milliseconds>(ms{imuTimeA}) };
        sys_time<milliseconds> imuTB{ round<milliseconds>(ms{imuTimeB}) };

        auto imuTA_msPH = imuTA - floor<hours>(imuTA);
        auto imuTB_msPH = imuTB - floor<hours>(imuTB);

		//go to next lidarTime until it's greater than imuTimeA
        while (microseconds(lidarTime) < imuTA_msPH){ 
			
			//The next data point's timestamp is three columns away. 
			//Refer to the Matrix organization document
            lCol += 3;	

			//lCol has reached the end of the row
            if (lCol > 48){ 
                lRow++;
                lCol = 3;
            }

            lidarTime = lidarData[lRow][lCol];
        }

		//while the lidarTime is between the two imu ts, keep incrementing through lidarTime
        while (microseconds(lidarTime) >= imuTA_msPH && microseconds(lidarTime) < imuTB_msPH){	
        
            timeFlag = false;
			
			//lidarTime is closer to imuA than imuB
            if (abs(imuTA_msPH - microseconds(lidarTime)) <= abs(imuTB_msPH - microseconds(lidarTime))) { 

               	imuRowSelect = imuRow; 
				//use imuTimeA
                if (abs(imuTA_msPH - microseconds(lidarTime)) < (microseconds(testTime))) {
                    timeFlag = true;
                }

			//lidarTime is closer to imuB than imuA
            }else{										
				//use imuTimeB
                imuRowSelect = imuRow + 1;	

                if (abs(imuTB_msPH - microseconds(lidarTime)) < (microseconds(testTime))) {
                    timeFlag = true;
                }

            }

            if (timeFlag) {

                //begin pt cloud math
                lat = imuData[imuRowSelect][0];
                lon = imuData[imuRowSelect][1];
                alt = imuData[imuRowSelect][2];
                roll = ConvertToRadians(imuData[imuRowSelect][7]);
                pitch = ConvertToRadians(imuData[imuRowSelect][8]);
                yaw = ConvertToRadians(imuData[imuRowSelect][9]);
                
                alpha = ConvertToRadians(lidarData[lRow][0] / 100);
                distance = lidarData[lRow][lCol - 2];
                timeStamp = lidarData[lRow][lCol];
                lzr = (lCol / 3) - 1;
                omega = laserAngle[lzr];

                if (distance == 0) {	//skipping the data point if the distance is zero
                    lCol = lCol + 3;	//the next data point's timestamp is three columns away. Refer to the Matrix organization document
                    if (lCol > 48) { lRow++; lCol = 3; }

                    lidarTime = lidarData[lRow][lCol];
                    continue;
                }

                X = distance * sin(alpha) * cos(omega);
                Y = distance * cos(omega) * cos(alpha);
                Z = -distance * sin(omega);

               	auto X1 = X * cos(yaw) - Y * sin(yaw)/* + lonOffset*/;
                auto Y1 = X * sin(yaw) + Y * cos(yaw)/* - latOffset*/;
               
                //X transform (pitch + y_offset)
                X1 = X;
                Y1 = Y * cos(pitch) - Z * sin(pitch);
                auto Z1 = Y * sin(pitch) + Z * cos(pitch);

                //Y transform (roll)
                X = X1 * cos(roll) - Z1 * sin(roll);
                Y = Y1;
                Z = -X1 * sin(roll) + Z1 * cos(roll);

                //Z transform (yaw)
                X1 = X * cos(yaw) - Y * sin(yaw);
                Y1 = X * sin(yaw) + Y * cos(yaw);
                Z1 = Z;

				int altOffset;
                //Position offset
                X1 = X1 + lonOffset;
                Y1 = Y1 - latOffset;
                Z1 = Z1 + altOffset;
                
                if (ConvertToDegrees(yaw) > testAngle) {
                	using namespace std;
                    ptCloudOFS << setw(12) << right << setprecision(5) << fixed 
                    << X1 << " " << setw(12) << right << setprecision(5) << fixed 
                    << Y1 << " " << setw(12) << right << setprecision(5) << fixed 
                    << Z << " " << setw(12) << right << setprecision(3) << 100 
                    << endl;
                } else {
                	using namespace std;
                    ptCloudOFS << setw(12) << right << setprecision(5) << fixed 
                    << X1 << " " << setw(12) << right << setprecision(5) << fixed 
                    << Y1 << " " << setw(12) << right << setprecision(5) << fixed 
                    << Z << " " << setw(12) << right << setprecision(3) << 0 << endl;
                }
                //end pt cloud math


                //increment lidarTime here
                lCol += 3;	//the next data point's timestamp is three columns away. Refer to the Matrix organization document
                if (lCol > 48) { lRow++; lCol = 3; }

                lidarTime = lidarData[lRow][lCol];
                lidarTime = lidarTime;

            } else {
                //increment lidarTime here
                lCol += 3;	//the next data point's timestamp is three columns away. Refer to the Matrix organization document
                if (lCol > 48) { lRow++; lCol = 3; }

                lidarTime = lidarData[lRow][lCol];
                std::cout << "lidartime: " << lidarTime;
                test << std::endl;
            }
        }
    }
	test.close();
	ptCloudOFS.close();
	
#pragma endregion

}
