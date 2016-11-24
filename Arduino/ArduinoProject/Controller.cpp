#include "Controller.h"
#include "robot.h"



Controller::Controller()
{
	x1 = 0;
	//x2 = 0;
}

float Controller::NextState(float error)
{

	//float u = -a / (4 + a + b)*(error + 2 * e1 + e2) - (2 * a - 8) / (4 + a + b)*u1 + u2;
	//float u = -error*System.getGPinFloat(0) + u1*System.getGPinFloat(1) - (u2 - u1)*System.getGPinFloat(2);

	/// Update state:
	float x1_new = 0.99* x1 + 0.125 * error;
	float u = 0.08*x1 + 0 * error;
	x1 = x1_new;

	//float u = x1 + 2;
	//if (u >= 1000) { u = -1000; }
	//x1 = u;

	if (u > 3000)
	{
		u = 3000; 
	}
	if (u < -3000)
	{
		u = -3000; 
	}
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
