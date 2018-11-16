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

constexpr inline double lerp_percent(long long start, long long end, 
									long long current){
	return static_cast<double>(current - start) / (end - start);
}

struct vector3{
	double x, y, z;
	
	vector3 operator+ (const vector3& rhs) const {
		return {x + rhs.x, y + rhs.y, z + rhs.z};
	}
	
	vector3 operator- (const vector3& rhs) const {
		return {x - rhs.x, y - rhs.y, z - rhs.z};
	}
	
	vector3 operator* (double rhs) const {
		return {x * rhs, y * rhs, z * rhs};
	}
	
	void operator*= (double rhs){
		x *= rhs;
		y *= rhs;
		z *= rhs;
	}
};

inline vector3 lerp(vector3 start, vector3 end, double percent){
	return start + ((end - start) * percent);
}



#endif
