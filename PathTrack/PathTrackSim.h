#ifndef PATHTRACK_PATHTRACKSIM_H_
#define PATHTRACK_PATHTRACKSIM_H_
#include "PathGenerator.h"
#include <list>
namespace pathTrack {

	class CarSim 
	{
	public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		CarSim(float period);
		void SetVariables(Vector3f& var);
		float ComputeSim(float v, int method);
		float ComputeSim_IMU(float v, int method);
		void Reset();

		Vector3f x;
//		Vector3f x_delay;
//		Vector3f x_hat;
//		list<float> phi_delay;

		float phi = 0;
        float theta0 = 0;
	private:
		float period;
		Vector3f dot_x;
		Vector3f& ComputeDotVar(float theta, float v_r, float _phi);
		Vector3f& ComputeDotVar_IMU(float v_r);
	};

	class PathTrackSim
	{
	public:
		PathTrackSim(float r);
		~PathTrackSim();
		char* Init(Vector3f& initParam, int method, float h);
		bool ComuputeError(Vector2f& z, float& e);
		float Stanley(Vector2f& z, float theta, float e,float v, bool direction);

		PathGenerator* pathGenerator;
		CarSim* carSim;

		uint16_t nowStep = 0;
	};
}

#endif /* PATHTRACK_PATHTRACKSIM_H_ */
