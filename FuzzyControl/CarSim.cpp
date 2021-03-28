#include <Apps/Car.h>
#include "CarSim.h"
#include <cmath>
#include <FuzzyControl/CarFuzzy.h>

namespace fuzzy {
	CarSim::CarSim(float len, float period)
		:length(len),period(period)
	{

	}
	void CarSim::SetVariables(Eigen::Vector3f& var)
	{
		this->x = var;
	}

	Eigen::Vector3f& CarSim::ComputeDotVar(float theta, float v_r)
	{
		float theta_R = theta / 180 * PI;
		float phi_R = phi / 180 * PI;
		float v_f = v_r / cos(phi_R);
		dot_u[0] = -v_f * cos(theta_R + phi_R);
		dot_u[1] = -v_f * sin(theta_R + phi_R);
		dot_u[2] = -v_r * tan(phi_R) / length / PI * 180;
		return dot_u;
	}

	/*
	* method = 0  平行泊车
	* method = 1  垂直泊车
	*/
	float CarSim::Compute(float v, int method)
	{
		Eigen::Matrix<float, 3, 4> K;
		
		K.col(0) = ComputeDotVar(x[2], v);
		K.col(1) = ComputeDotVar(x[2] + K(2, 0) * period / 2, v);
		K.col(2) = ComputeDotVar(x[2] + K(2, 1) * period / 2, v);
		K.col(3) = ComputeDotVar(x[2] + K(2, 2) * period, v);

		x = x + period / 6 * (K.col(0) + 2 * K.col(1) + 2 * K.col(2) + K.col(3));
		
		Car::parkSimParam.x = x[0];
		Car::parkSimParam.y = x[1];
		Car::parkSimParam.theta = x[2];

//		float fuzzyInput[] = { x[0],x[1],x[2] };
//		if (method == 0)   //选择泊车方式
//			phi = fuzzy::parallel::CarFuzzy.Compute(fuzzyInput);
//		else if (method == 1)
//			phi = fuzzy::vertical::CarFuzzy.Compute(fuzzyInput);
//		return phi;
	}
}
