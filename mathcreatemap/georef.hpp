#ifndef _GEOREF
#define _GEOREF

#include <iostream>
#include <vector>
#include <array>
#include <string>

inline void print(const std::string& s){
    std::cout << s << std::endl;
}

void georefMath(const std::vector<std::array<double, 50>>& lidarData, 
				const std::vector<std::array<double, 11>>& imuData,
				const std::string& output);

#endif
