#pragma once
#include <cmath>

//angle range (-pi,pi]
inline double angleWrap(double a)
{
	while (a > M_PI) a -= 2.0 * M_PI;
	while (a <= -M_PI) a += 2.0 * M_PI;
	return a;
}

inline double clamp(double x, double val_min, double val_max)
{
	if (x < val_min) return val_min;
	if (x > val_max) return val_max;
	return x;
}