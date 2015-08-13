#ifndef DEADRECKONING_H
#define DEADRECKONING_H

#include <Eigen/Dense>
#include <iostream> 

using namespace Eigen;

class DeadReckoning{

	// State vector.
	// state = [x y z theta u v r w phi_0]'
	double lastLogTime;
	VectorXd state;
	MatrixXd P;

public:
	DeadReckoning(){
		state.resize(8);
		P.resize(8,8);
		P.setZero(8,8);
	};
	DeadReckoning(VectorXd initialize, double sigmaPhi, double logTime){
		state = initialize;
		P = MatrixXd::Identity(8,8)*0.001;
		// P.resize(8,8);
		// P.setZero(8,8);
		std::cout << P << std::endl;
		std::cout << " " << std::endl;
		lastLogTime = logTime;
	};

	void predictState(double logTime);
	void updateStateDVL(VectorXd update, double logtime);
	void updateStateMTi(VectorXd update, double logtime);

	VectorXd returnState(){ return state; }

private:
	MatrixXd jacobianF(VectorXd st, double dT);
	MatrixXd jacobianG(VectorXd st, double dT);

};

#endif