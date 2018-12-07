#include "new_georef.hpp"
//#include "math.hpp"
#include "data.hpp"

//System libraries 
//Reading / writing
#include <iostream>
#include <fstream>
//Storage
#include <array>
#include <vector>
#include <string>
#include <chrono>
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

std::vector<std::string> split(const std::string& data, const std::string& delim){
	
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

	const std::string LIDAR_SOURCE = "lidar_data.txt";
	const std::string IMU_SOURCE = "IMU.txt";
	const std::string OUTPUT_FILE = "trial_.txt";
    const std::string ANGLE_DET = "angle=";
    const std::string TIME_DET = "time =";

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
    
	vector<lidar_data> lidarData(LineCount(lidarIFS)/12);
    
    /*
   {
   	angle= azimuth
   	dist[0]  reflec[0] {0..32}
   }12
   
   time
    */
   
   constexpr std::chrono::milliseconds default_time(-1);
   
	size_t unused_lines = 0;   
    string cur;			//Stores a line from the LIDAR text file. It is replaced with the following line during every loop.
    while (getline(lidarIFS, cur)){
        
        if (cur.substr(0, ANGLE_DET.size()) == ANGLE_DET){
        	
        	lidar_data current;
        	current.alpha = std::stod(cur.substr(ANGLE_DET.size()));
        	
        	for(size_t i = 0; i < current.distance.size(); i++){
        		lidarIFS >> current.distance[i] >> current.reflectivity[i]; 
        	}
        	
        	current.time = default_time;
        	
        	lidarData.push_back(current);
        	
		}else if (cur.substr(0, TIME_DET.size()) == TIME_DET){
            
            long val = std::stol(cur.substr(TIME_DET.size()));
            
            size_t pos = lidarData.size()-1;
            while(lidarData[pos].time == default_time){
            	lidarData[pos].time = std::chrono::milliseconds(val);
            	pos--;
            }

		} else if(!cur.empty()){            
			cout << "This line is useless:" << cur << endl;
			unused_lines++;
		}

    }

	std::cout << "Unused Lines: " << unused_lines << std::endl;
	std::cout << "Waiting for IMU Data" << std::endl;

	const auto imuData = imu_reader.get();
   
	std::cout << "Done reading files" << std::endl;

	georefMath(lidarData, imuData, OUTPUT_FILE);
}

