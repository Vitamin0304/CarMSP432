#ifndef CAR_SIM_H_
#define CAR_SIM_H_

#include <Eigen/Core>
#define PI 3.1415926535f
namespace fuzzy {
	class CarSim
	{
	public:
		CarSim(float len, float period);
		void SetVariables(Eigen::Vector3f& var);
		float Compute(float v, int method);
		Eigen::Vector3f x;
		float phi = 0;
	private:
		float length;
		
		
		float period;
		Eigen::Vector3f dot_u;

		Eigen::Vector3f& ComputeDotVar(float theta, float v_r);
	};
}

#endif
