#include "georef.hpp"
#include "math.hpp"
//System libraries
#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <string>

auto LineCount(std::ifstream& file){
    
    using namespace std;
    unsigned n = 0;
    string s;
    while (getline(file, s)){ n++; }
    cout << "Lines counted: " << n << endl;

    return n;
}

int main() {

	using namespace std;

    //opens the files to read/write
    ifstream lidarIFS("lidarData.txt");
    ifstream imuIFS("IMU.txt");
    
    print("***Counting lines from input files and creating matrices");

    auto nLidarLines = LineCount(lidarIFS);	
    //Skip every 13th line (which is a timestamp) 
    //then multiply by 2 (each line in the text file takes up two lines matrix)
    nLidarLines = (nLidarLines - (nLidarLines / 13)) * 2;
    
    auto nGpsLines = (nLidarLines - (12 * (nLidarLines / 13)));
    auto nImuLines = LineCount(imuIFS);

    //Resets the the file stream pointer to the beginning of the file
    lidarIFS.clear();
    lidarIFS.seekg(0);
    imuIFS.clear();
    imuIFS.seekg(0);
	print("Done Counting");
	
#pragma region ARRAY DECLARATIONS

	vector<array<double, 50>> lidarData (nLidarLines);
	vector<array<string, 13>> lidarGPS (nGpsLines);
	vector<array<double, 11>> imuData (nImuLines);

    //Angles of the 16 individual lasers are provided by Velodyne documentation.
    //double laserAngle[16] = { 105, 89, 103, 93, 101, 85, 99, 83, 97, 81, 95, 79, 93, 77, 91, 75 };
    //double laserAngle[16] = { 15, -1, 13,   3, 11,  -5, 9,  -7, 7, -9, 5,  -11, 3, -13, 1, -15 };
    //guess: the bottom array is just the top - 90
    array<double, 16> laserAngle = { 15, -1, 13, 3, 11, -5, 9, -7, 7, -9, 5, -11, 3, -13, 1, -15 };
    for (unsigned ctr = 0; ctr < laserAngle.size(); ctr++){
        laserAngle[ctr] = ConvertToRadians(laserAngle[ctr]);
    }
#pragma endregion

#pragma region VARIABLES FOR DATA INPUT
    const string ANGLE_DET = "angle=";
    const string TIME_DET = "time=";
    const string GPS_DET = "GPS=";

    string cur;			//Stores a line from the LIDAR text file. It is replaced with the following line during every looping of the while loop.
    int row = 0;		//Row value for the lidarData two-dimensional array.
    int col = 0;		//Column value "										".
    int gRow = 0;		//Row value for the lidarGPS two-dimensional array.
    int charPos = 0;	//Modified throughout the program to know where to start reading data.
    int curTime = 0;	//Stores the value of the most recently encountered LIDAR time value.
    //int gpsTime = 0;	//Stores the value of the most recently encountered GPS time stamp, as identified by the VLP-16 documentation.
#pragma endregion

    print("***Processing LIDAR data");

    while (getline(lidarIFS, cur)){
        //Seeks angle_det at the beginning of a line, stores angle value and the following distance and reflectivity points.
        //Interpolates missing angle values, as described in the VLP-16 documentation
        if (cur.substr(0, 6) == ANGLE_DET){
        	//Load the angle value
            lidarData[row][col] = stod(cur.substr(6, 11));

            unsigned cursor = 0;
            for (unsigned i = 1; i < 96; i++) {
                col++;
				
				//Indicates the end of one sequence of lazer firings
                if (i == 49){ 
                	//Go down one row for the next set of distance+reflectivity values
                    row++;
                    
                    //Do not use 0 b/c that is where the interpolated result will be stored	
                    col = 1;	

                    //azimuth interpolation
                    //Check to avoid access violation error or index out of bound error
                    if (row >= 3){	
                    
                    	//360 * 100
                    	constexpr auto magicVal = 36000;
                        auto azi1 = lidarData[row - 3][0];
                        auto azi3 = lidarData[row - 1][0];

                        if (azi3 < azi1){
                            azi3 += magicVal;
                        }

                        auto azi2 = (azi1 + azi3) / 2;

						//account for rollover. values are not to exceed 35999.
                        if (azi2 >= magicVal) {
                            azi2 -= magicVal;
                        }
						
						//assign the missing angle value with the interpolated one
                        lidarData[row - 2][0] = azi2; 
                    }
                }

				//This is to avoid any column reserved for time stamps. 
				//See the lidarData Matrix Organization spreadsheet
                if (i % 3 != 0){ 
                	//To move through the text file value by value 
                	//11 characters apart, offset 18 to not read the first item
                    charPos = 18 + (11 * cursor);	
                    //Getting distance value
                    lidarData[row][col] = stod(cur.substr(charPos, 11)); 
                    cursor++;
                }
            }
            row++;
            
            //Reset to 0. 
            //when it reads an angle value next, col will be set to the first column
            col = 0; 
        
        /*Seeks time_det at the beginning of a line, 
        stores the time value and calculates the exact time for each data point, 
        as described in the VLP-16 documentation*/
        }else if (cur.substr(0, 5) == TIME_DET){
            curTime = stod(cur.substr(5, 11));

            for (int i = 23; i >= 0; i--) {
                
                const auto rowIndex = (row - 24) + i;
                
                lidarData[rowIndex][49] = curTime;

                for (unsigned j = 1; j < 17; j++){
					//Should these variables be moved to their only usage
                    auto sequence_index = i;
                    auto data_pt_index = j - 1;

                    lidarData[rowIndex][j * 3] = curTime 
                    						+ (55.296 * sequence_index) 
                   							+ (2.304 * data_pt_index);
                }
            }
        

        //Seeks GPS_DET at the beginning of a line, stores the entire GPS sentence in a string matrix with each row being it's
        //own sentence. Details are in the VLP-16 documentation and Matrix Organization spreadsheet
       }else if (cur.substr(0, 4) == GPS_DET) {
            
            //Avoid an exception when the lidar capture code has a typo in the GPS line
            if (cur.substr(0, 8) != "GPS= $GP"){
            
                //TODO: Have this continue to gather the GPS data after the system typo
                print("GPS ERROR");
                break;
            }
            //gpsTime = stod(cur.substr(12, 6));

            lidarGPS[gRow][0] = cur.substr(12, 6);	//GPS time
            lidarGPS[gRow][1] = cur.substr(19, 1);	//Validity, A or V
            lidarGPS[gRow][2] = cur.substr(21, 9);	//Current Latitude
            lidarGPS[gRow][3] = cur.substr(31, 1);	//N or S
            lidarGPS[gRow][4] = cur.substr(33, 10);	//Current Longitude
            lidarGPS[gRow][5] = cur.substr(44, 1);	//E or W
            lidarGPS[gRow][6] = cur.substr(46, 5);	//Speed in knots
            lidarGPS[gRow][7] = cur.substr(52, 5);	//True course
            lidarGPS[gRow][8] = cur.substr(58, 6);	//Date Stamp
            lidarGPS[gRow][9] = cur.substr(65, 5);	//Variation
            lidarGPS[gRow][10] = cur.substr(71, 1);	//E or W
            lidarGPS[gRow][11] = cur.substr(73, 4);	//checksum
            lidarGPS[gRow][12] = curTime;			//timestamp from LIDAR

            gRow++;

        }

    }

    print("DONE");

    //reset for the next while loop that takes in the IMU data
    row = 0;
    //col = 0; //col is not used after this point

    print("Processing IMU data");

    while (getline(imuIFS, cur)){
    
        imuData[row][0] = stod(cur.substr(0, 15));	//latitude
        imuData[row][1] = stod(cur.substr(16, 15));	//longitude
        imuData[row][2] = stod(cur.substr(31, 15));	//altitude
        imuData[row][3] = stod(cur.substr(46, 15)); //w
        imuData[row][4] = stod(cur.substr(61, 15)); //x
        imuData[row][5] = stod(cur.substr(76, 15)); //y
        imuData[row][6] = stod(cur.substr(91, 15)); //z
        imuData[row][7] = stod(cur.substr(106, 15)); //roll
        imuData[row][8] = stod(cur.substr(121, 15)); //pitch
        imuData[row][9] = stod(cur.substr(136, 15)); //yaw
        imuData[row][10] = stod(cur.substr(151, 21)); //time stamp

        row++;
    }

    print("DONE");

	georefMath(lidarData, imuData, laserAngle, "trial_.txt");
}


