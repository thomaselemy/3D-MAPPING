#ifndef _MATH_MINE
#define _MATH_MINE

#include <cmath>

constexpr auto PI = 3.141592653589793238463;

constexpr inline auto ConvertToRadians(const double angle){
    return (angle * PI) / 180;
}

constexpr inline auto ConvertToDegrees(const double angle){
    return (angle / PI) * 180;
}

#endif
