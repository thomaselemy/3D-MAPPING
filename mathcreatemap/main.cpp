#include "georef.hpp"
#include "math.hpp"

//System libraries 
//Reading / writing
#include <iostream>
#include <fstream>
//Storage
#include <array>
#include <vector>
#include <string>
//Multi-threading
#include <thread>
#include <future>

unsigned LineCount(std::ifstream& file){
    
    unsigned num = 0;
    std::string s;
    while (getline(file, s)){ num++; }
	
	//Resets the the file stream pointer to the beginning of the file
    file.clear();
    file.seekg(0);
    
    return num;
}

auto split(const std::string& data, const std::string& delim){
	
	std::vector<std::string> toRet;
	size_t pos, lastPos = 0;
	
	while((pos = data.find(delim, lastPos)) != std::string::npos){
		toRet.push_back(data.substr(lastPos, pos));
		lastPos = pos + delim.length();
	}
	
	return toRet;
}

auto loadIMUData(const std::string& file_name){

	using namespace std;
	
	vector<imu_data> imuData{};

	ifstream imuIFS(file_name);
	if(!imuIFS){ 
		cerr << "Could not open file " << file_name << endl;
	}
	
	string cur;
	while (getline(imuIFS, cur)){
     
        double lat = stod(cur.substr(0, 15));
        double lon = stod(cur.substr(16, 15));
        double alt = stod(cur.substr(31, 15));
        
        double pitch = stod(cur.substr(121, 15));
        double yaw = stod(cur.substr(136, 15));
        double roll = stod(cur.substr(106, 15));
        
        long time = stol(cur.substr(151, 21));
        imuData.push_back({lat, lon, alt, 
        	ConvertToRadians(pitch), 
        	ConvertToRadians(yaw), 
        	ConvertToRadians(roll), 
        	std::chrono::milliseconds(time)
        });
    }
    
    imuIFS.close();
    
    return imuData;
}


int main() {

	const std::string LIDAR_SOURCE = "lidarData.txt";
	const std::string IMU_SOURCE = "IMU.txt";
	const std::string OUTPUT_FILE = "trial_.txt";
    const std::string ANGLE_DET = "angle=";
    const std::string TIME_DET = "time=";

    unsigned row = 0;		//Row value for the lidarData two-dimensional array.
    unsigned col = 0;		//Column value "										".
    double curTime = 0;	//Stores the value of the most recently encountered LIDAR time value.
  
	using namespace std;
    cout << "Processing two data streams: " << endl 
    << "\tLIDAR data from " << LIDAR_SOURCE << endl
    << "\tIMU data from " << IMU_SOURCE << endl;
	
	auto imu_reader = async(loadIMUData, IMU_SOURCE);

	//opens the files to read/write
    ifstream lidarIFS(LIDAR_SOURCE);
    if(!lidarIFS){
    	cerr << "Could not open file " << LIDAR_SOURCE << endl;
    }
    
    const auto nLidarLines = LineCount(lidarIFS);	
    
	//Skip every 13th line (which is a timestamp) 
    //then multiply by 2 (each line in the text file takes up two lines matrix)
	vector<lidar_entry> lidarData ((nLidarLines - (nLidarLines / 13)) * 2);
    
    
    /*
   {
   	angle= azimuth
   	dist[0]  reflec[0] {0..32}
   }12
   
   time
    */
   
    string cur;			//Stores a line from the LIDAR text file. It is replaced with the following line during every loop.
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
                        const auto azi1 = lidarData[row - 3][0];
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
                    //Getting distance value
                    lidarData[row][col] = 
                    				stod(cur.substr(18 + (11 * cursor), 11)); 
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
                
                const unsigned rowIndex = (row - 24) + i;
                
                lidarData[rowIndex][49] = curTime;

                for (unsigned j = 1; j < 17; j++){
			
                    lidarData[rowIndex][j * 3] = curTime 
                    						+ (55.296 * i) 
                   							+ (2.304 * (j - 1));
                }
            }

       } else {            
			cout << "This line is useless:" << cur << endl;
       }

    }

	print("Waiting for IMU Data");

	const auto imuData = imu_reader.get();
   
    print("Done reading files");

	georefMath(lidarData, imuData, OUTPUT_FILE);
}

