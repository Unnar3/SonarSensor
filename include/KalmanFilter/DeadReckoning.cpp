#include "KalmanFilter/DeadReckoning.h"
#include <iostream> 
#include <math.h>
#include <random>

void DeadReckoning::predictState(double logTime){
	VectorXd sf(8);
	sf << 0,0,0,0,0,0,0,0;
	VectorXd sn(8);
	sn << 0,0,0,0,0,0,0,0;

	MatrixXd Q = MatrixXd::Identity(4, 4)*10;

	// Gaussian white noise additive
	std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> d(0,0.01);
	double su = d(gen);
	double sv = d(gen);
	double sr = d(gen);
	double sw = d(gen);

	double dT = logTime - lastLogTime;
	double dT2= dT*dT/2;
	
	// update based on previous state
	sf(0) = dT * (cos(state(3))*state(4) - sin(state(3))*state(5));
	sf(1) = dT * (sin(state(3))*state(4) + cos(state(3))*state(5));
	sf(2) = dT * state(6);
	sf(3) = dT * state(7);

	// update based on white noise
	sn(0) = dT2 * (cos(state(3))*su - sin(state(3))*sv);
	sn(1) = dT2 * (sin(state(3))*su + cos(state(3))*sv);
	sn(2) = dT2 * sw;
	sn(3) = dT2 * sr;
	sn(4) = dT * su;
	sn(5) = dT * sv;
	sn(6) = dT * sr;
	sn(7) = dT * sw;

	// Create jacobian matrices from current state
	MatrixXd F = DeadReckoning::jacobianF(state, dT);
	MatrixXd G = DeadReckoning::jacobianG(state, dT);

	// Combine into new prediction
	state = state + sf + sn;

	// Update covariance P
	P = F*P*F.transpose() + G*Q*G.transpose();

	std::cout << "P:" << std::endl;
	std::cout << P << std::endl;

	// std::cout << state << std::endl;
	// std::cout << " " << std::endl;

	// Update lastLogTime
	lastLogTime = logTime;
};

void DeadReckoning::updateStateDVL(VectorXd update, double logtime){
	MatrixXd H;
	H.setZero(4,8);
	H(0,4) = 1;
	H(1,5) = 1;
	H(2,6) = 1;
	H(3,2) = 1;
	std::cout << "H: " << std::endl;
	std::cout << H << std::endl;

	MatrixXd R;
	R.setZero(4,4);
	R(0,0) = 0.001;
	R(1,1) = 0.001;
	R(2,2) = 0.001;
	R(3,3) = 0.001;

	VectorXd y = update - H*state;

	MatrixXd S = H*P*H.transpose() + R;
	MatrixXd K = P*H.transpose()*S.inverse();
	state = state + K*y;
	P = (MatrixXd::Identity(8, 8) - K*H)*P;
	std::cout << "State: " << state << std::endl;
	std::cout << "" << std::endl;
};

void DeadReckoning::updateStateMTi(VectorXd update, double logtime){
	MatrixXd H(1,8);
	H << 0,0,0,1,0,0,0,0;
	MatrixXd R(1,1); 
	R(0,0) = 0.001;
	VectorXd y = update - H*state;
	MatrixXd S = H*P*H.transpose() + R;
	MatrixXd K = P*H.transpose()*S.inverse();
	state = state + K*y;
	P = (MatrixXd::Identity(8, 8) - K*H)*P;
	std::cout << "state: " << state << std::endl;
	std::cout << "" << std::endl;
}


MatrixXd DeadReckoning::jacobianF(VectorXd st, double dT){
	MatrixXd F;
	F.setZero(8,8);
	F(0,0) = 1; 
	F(0,3) = dT*(-st(4)*sin(st(3)) - st(5)*cos(st(3))); 
	F(0,4) = dT*cos(st(3));
	F(0,5) = -dT*sin(st(3));
	F(1,1) = 1;
	F(1,3) = dT*(st(4)*cos(st(3)) - st(5)*sin(st(3)));
	F(1,4) = dT*sin(st(3));
	F(1,5) = dT*cos(st(3));
	F(2,2) = 1;
	F(2,6) = dT;
	F(3,3) = 1;
	F(3,7) = dT;
	F(4,4) = 1;
	F(5,5) = 1;
	F(6,6) = 1;
	F(7,7) = 1;
	return F;
};
MatrixXd DeadReckoning::jacobianG(VectorXd st, double dT){
	double dT2 = dT*dT/2;
	MatrixXd G;
	G.setZero(8,4);
	G(0,0) = dT2 * cos(st(3));
	G(0,1) = -dT2 * sin(st(3));
	G(1,0) =  dT2 * sin(st(3));
	G(1,1) = dT2 * cos(st(3));
	G(2,2) = dT2;
	G(3,3) = dT2;
	G(4,0) = dT;
	G(5,1) = dT;
	G(6,2) = dT;
	G(7,3) = dT;
	return G;
};