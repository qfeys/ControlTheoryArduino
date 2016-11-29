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

void Controller::Set(float undamptFreq, float damping, float sampleTime)
{
	a = sampleTime*sampleTime*undamptFreq*undamptFreq;
	b = 4 * damping*undamptFreq;
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


float EstimateFriction(float input, float velocity)
{
	float g[6] = { 3000 , 0.3 , 0.08 , 1500 , 1, 1 };
	float Vel_new = velocity + (input>0?4:-4);
	float T = g[0] * (tanh(g[1] * Vel_new) - tanh(g[2] * Vel_new))		// Stribeck effect.
		+ g[3] * tanh(g[4] * Vel_new)								// Coulomb effect.
		+ g[5] * Vel_new;											// Viscous dissipation term.
	return T;
}