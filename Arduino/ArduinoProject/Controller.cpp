#include "Controller.h"
#include "robot.h"



Controller::Controller()
{
	x1 = 0;
	x2 = 0;
}

float Controller::NextState(float error)
{

	//float u = -a / (4 + a + b)*(error + 2 * e1 + e2) - (2 * a - 8) / (4 + a + b)*u1 + u2;
	//float u = -error*System.getGPinFloat(0) + u1*System.getGPinFloat(1) - (u2 - u1)*System.getGPinFloat(2);
	
	/// Update state:
	float x1_new = 1* x1 + 0 * x2 + 0.2 * error;
	float x2_new = 0 * x1 + 0 * x2 + 0*error;
	float u = 0.15*x1 + 0*x2 + 3 * error;
	x1 = x1_new;
	x2 = x2_new;

	/*
	float u = 1500*sin(x1*(x1*0.1+1));
	x1 += 0.01;
	*/

	/*
	if (u > 3000)
	{
		u = 3000; 
	}
	if (u < -3000)
	{
		u = -3000; 
	}*/
	System.setGPoutFloat(6, x1);
	return u;
}

void Controller::Reset()
{
	x1 = 0;
	x2 = 0;
}

String Controller::State()
{
	String s = "";
	return s; // + "u1: " + u1 + ", u2:" + u2 + ", e1: " + e1 + ", e2: " + e2;
	//return s.concat("u1: ").concat(u1).concat(", u2:").concat(u2).concat(", e1: ").concat(e1).concat(", e2: ").concat(e2);
}

Controller::~Controller()
{
}


Controller2::Controller2()
{
	q1 = 0;
	q2 = 0;
	q3 = 0;
}

/// Give all in si units
float Controller2::NextState(float velocity, float position, float theta)
{
	// Estimate State
	// A = [[1, 0, 0][0, 1.004, 0.01001],[0, 0.8186, 1.004]]
	// B = [[0, -0.08345, -0.03409]]
	// C = [[ 1, 0, 0][ 0, 1, 0]]
	// D = [[0, 0]]
	// L = [[0.9410, 0, 0][0, 0.9540, 1.6372]]
	// K = [-3.1748, -4.4525, -0.3453]
	float err1 = position - 1 * q1;
	float err2 = theta - 1 * q2;
	float q1n = 1 * q1 + err1 * 0.6180;
	float q2n = 1.004*q2 + 0.01001*q3 - 0.08345*velocity + 0.6387 * err2;
	float q3n = 0.8186*q2 + 1.004 * q3 - 0.03409 * velocity + 1.6372*err2;

	q1 = q1n;
	q2 = q2n;
	q3 = q3n;

	// calculate control
	return -(-0.9008 * q1 - 2.4713 * q2 - 0.2545 * q3);

}

void Controller2::Reset()
{
	q1 = 0;
	q2 = 0;
	q3 = 0;
}

Controller2::~Controller2()
{
}


float EstimateFriction(float input, float velocity)
{
	float g[6] = { 3000 , 0.3 , 0.08 , 1500 , 1, 1 };
	float Vel_new = velocity + (input > 0 ? 4 : -4);
	float T = g[0] * (tanh(g[1] * Vel_new) - tanh(g[2] * Vel_new))		// Stribeck effect.
		+ g[3] * tanh(g[4] * Vel_new)								// Coulomb effect.
		+ g[5] * Vel_new;											// Viscous dissipation term.
	return T;
}

float EstimateFriction2(float input, float direction)
{
	float T = input*1.5 + 2200 * (input > 0 ? 1 : -1);	// viscous + coulomb effect
	T += (input*direction > 0 ? 0 : (input > 0 ? 1 : -1)) * 2000;	// strikeback effect
	return T;
}