#include "PathTrackSim.h"
#include <Apps/Encoder.h>
extern "C" {
#include "Apps/my_uart.h"
}
namespace pathTrack {
	PathTrackSim::PathTrackSim(float r)
	{
		pathGenerator = new PathGenerator(r);
		carSim = new CarSim(delay_s);
	}
	PathTrackSim::~PathTrackSim()
	{
	    delete pathGenerator;
	    delete carSim;
	}

	char* PathTrackSim::Init(Vector3f& initParam, int method, float h)
	{
	    nowStep = 0;
	    carSim->Reset();
		carSim->SetVariables(initParam);
		return pathGenerator->Generate(initParam, method, h);
	}
	bool PathTrackSim::ComuputeError(Vector2f& z, float& e)
	{
		uint16_t& pathLengh = pathGenerator->stepSum;
		Vector3f& x = carSim->x;
		float (*path)[2] = pathGenerator->path;
		float distMin = -1.0f;
		float dist = -1;
		uint16_t stepMin = 0;
		Vector3f P1R(0, 0, 0);
		bool stop = false;
        if (nowStep >= pathLengh-1)
        {
            stop = true;
            return stop;
        }
		for (int i = nowStep; i < nowStep + 5; ++i)
		{
			if (i<0 || i>=pathLengh)
				continue;
			P1R[0] = x[0] - path[i][0];
			P1R[1] = x[1] - path[i][1];
			dist = P1R.dot(P1R);
			if (distMin == -1.0f || dist < distMin)
			{
				distMin = dist;
				stepMin = i;
			}
		}
		nowStep = stepMin;
		if (nowStep >= pathLengh - 1)
		{
			nowStep = pathLengh - 2;
			stop = true;
		}
		P1R[0] = x[0] - path[nowStep][0];
		P1R[1] = x[1] - path[nowStep][1];
		z[0] = path[nowStep + 1][0] - path[nowStep][0];
		z[1] = path[nowStep + 1][1] - path[nowStep][1];
		
		Vector3f P1P2(z[0], z[1], 0);
		e = (P1R.cross(P1P2))[2] / P1P2.norm();

		return stop;
	}
	float PathTrackSim::Stanley(Vector2f& z, float theta, float e, float v, bool direction)
	{
	    if(v > 0 ^ direction)  //实际速度方向和指定方向不同时停止转弯
	        return 0;

		float k = 0.8;
		if (direction == false)
			z = -z;
		else
			e = -e;
		float theta_line = atan2f(z[1], z[0]);
		float theta_error = mod2pi(theta_line - theta);

		float delta_e;
        if(fabs(v)<0.01)
            delta_e = 0;
        else
            delta_e = atanf(k * e / fabsf(v));

		if (direction)
			return -theta_error + delta_e;
		else
			return theta_error + delta_e;
	}

	CarSim::CarSim(float period)
		:period(period) 
	{

	}

    Vector3f x_delay;
    Vector3f x_hat;
    list<float> phi_delay;
    list<float> v_delay;

	void CarSim::SetVariables(Vector3f& var)
	{
		this->x = var;
        theta0 = -IMU_Angle[2] + x[2];
        phi_delay.assign(10, 0);
        v_delay.assign(10, 0);
        x_delay = var;
        x_hat = var;
	}

	Vector3f& CarSim::ComputeDotVar(float theta, float v_r, float _phi)
	{
		float l = 0.142;
		dot_x[0] = v_r * cos(theta);
		dot_x[1] = v_r * sin(theta);
		dot_x[2] = -v_r * tan(_phi) / l;
		return dot_x;
	}
	/*
	* method = 0  平行泊车
	* method = 1  垂直泊车
	*/
	float CarSim::ComputeSim(float v, int method)
	{
		Eigen::Matrix<float, 3, 4> K;

		K.col(0) = ComputeDotVar(x[2], v, phi);
		K.col(1) = ComputeDotVar(x[2] + K(2, 0) * period / 2, v, phi);
		K.col(2) = ComputeDotVar(x[2] + K(2, 1) * period / 2, v, phi);
		K.col(3) = ComputeDotVar(x[2] + K(2, 2) * period, v, phi);

		x = x + period / 6 * (K.col(0) + 2 * K.col(1) + 2 * K.col(2) + K.col(3));
		return 0;
	}

    Vector3f& CarSim::ComputeDotVar_IMU(float v_r)
    {
        dot_x[0] = v_r * cos(x[2]);
        dot_x[1] = v_r * sin(x[2]);
        dot_x[2] = 0;
        return dot_x;
    }
    /*
    * method = 0  平行泊车
    * method = 1  垂直泊车
    */
    float CarSim::ComputeSim_IMU(float v_r, int method)
    {
//        theta导数的估计
//        float dot_theta_hat = -v_r * tan(phi) / 0.15;
        float alpha = abs(tan(phi))/0.53;

        if(alpha > 0.8)
            alpha = 0.8;
        else if(alpha < 0)
            alpha = 0;

        alpha = 0;

        Eigen::Matrix<float, 3, 4> K;

        K.col(0) = ComputeDotVar(x[2], v_r, phi);
        K.col(1) = ComputeDotVar(x[2] + K(2, 0) * period / 2, v_r, phi);
        K.col(2) = ComputeDotVar(x[2] + K(2, 1) * period / 2, v_r, phi);
        K.col(3) = ComputeDotVar(x[2] + K(2, 2) * period, v_r, phi);

        x_hat += period / 6 * (K.col(0) + 2 * K.col(1) + 2 * K.col(2) + K.col(3));
//
//        phi_delay.push_front(phi);
//        float phi_end = *(--phi_delay.end());
//        phi_delay.pop_back();
//
//        v_delay.push_front(v_r);
//        float v_end = *(--v_delay.end());
//        v_delay.pop_back();
//
//        K.col(0) = ComputeDotVar(x_delay[2], v_end, phi_end);
//        K.col(1) = ComputeDotVar(x_delay[2] + K(2, 0) * period / 2, v_end, phi_end);
//        K.col(2) = ComputeDotVar(x_delay[2] + K(2, 1) * period / 2, v_end, phi_end);
//        K.col(3) = ComputeDotVar(x_delay[2] + K(2, 2) * period, v_end, phi_end);
//
//        Vector3f x_delay_int = period / 6 * (K.col(0) + 2 * K.col(1) + 2 * K.col(2) + K.col(3));
//        x_delay += x_delay_int;

        x[2] =  IMU_Angle[2] + theta0;
        x += period * ComputeDotVar_IMU(v_r);

        x = alpha * x_hat + (1-alpha) * x;
//        x += x_hat_int - x_delay_int;

        return 0;
    }
	void CarSim::Reset()
	{
	    phi = 0;
	}
}
