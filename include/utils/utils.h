#ifndef UTILS_H
#define UTILS_H
namespace utils{
	
	double deg2rad(double deg){
		return deg/180.0*M_PI;
	}

	double rad2deg(double rad){
		return rad/M_PI*180;
	}
}
#endif