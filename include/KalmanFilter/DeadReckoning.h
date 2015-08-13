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
		state.resize(9);
		P.resize(9,9);
		P.setZero(9,9);
	};
	DeadReckoning(VectorXd initialize, double sigmaPhi, double logTime){
		state = initialize;
		P.resize(9,9);
		P.setZero(9,9);
		P(8,8) = sigmaPhi;
		std::cout << P << std::endl;
		lastLogTime = logTime;
	};

	void predictState(double logTime);
	void updateStateDVL(VectorXd update);
	void updateStateMTi(VectorXd update);

	VectorXd returnState(){ return state; }

private:
	MatrixXd jacobianF(VectorXd st, double dT);
	MatrixXd jacobianG(VectorXd st, double dT);

};

#endif