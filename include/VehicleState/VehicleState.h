#include <math.h>

class VehicleState{

	double _x 	= 0.0;
	double _y 	= 0.0;
	double _z 	= 0.0;
	double _phi = 0.0;
	double _u 	= 0.0; 
	double _v 	= 0.0; 
	double _w 	= 0.0; 
	double _r 	= 0.0; 
	double _T 	= -1.0;

public:

	VehicleState(){
		double _x 	= 0.0;
		double _y 	= 0.0;
		double _z 	= 0.0;
		double _phi = 0.0;
		double _u 	= 0.0; 
		double _v 	= 0.0; 
		double _w 	= 0.0; 
		double _r 	= 0.0; 
		double _T 	= -1.0;
	};

	VehicleState(double x, double y, double z, double phi, double u, double v, double w, double r, double T){
		_x	= x;
		_y 	= y;
		_z 	= z;
		_phi= phi;
		_u 	= u;
		_v 	= v;
		_w 	= w;
		_r 	= r;
		_T 	= T;
	};

	void updateState(double u, double v, double w, double r, double T){
		if ( _T != -1.0 ){
			_x 	= _x + _u*(T-_T) * cos(_phi) - _v*(T-_T) * sin(_phi);
			_y 	= _y + _u*(T-_T) * sin(_phi) + _v*(T-_T) * cos(_phi);
			_z 	= _z + _w*(T-_T);
			_phi= _phi + _r*(T-_T);
		}
		_u 	= u;
		_v 	= v;
		_w 	= w;
		_r 	= r;
		_T 	= T;
	};

	double x(){
		return _x;
	};

	double y(){
		return _y; 
	};

	double z(){
		return _z; 
	};

	double phi(){
		return _phi; 
	};
};