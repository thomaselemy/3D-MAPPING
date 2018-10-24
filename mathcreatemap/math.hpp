#ifndef _MATH_MINE
#define _MATH_MINE

#include <cmath>

constexpr auto PI = 3.141592653589793238463;

constexpr inline double ConvertToRadians(const double angle){
    return (angle * PI) / 180;
}

constexpr inline double ConvertToDegrees(const double angle){
    return (angle / PI) * 180;
}

#endif
