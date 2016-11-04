#include "Controller.h"



Controller::Controller()
{
	e1 = 0;
	e2 = 0;
	u1 = 0;
	u2 = 0;
}

float Controller::NextState(float error)
{

	float u = a / (4 + a + b)*(error + 2 * e1 + e2) + (2 * a - 8) / (4 + a + b)*u1 + u2;
	e2 = e1;
	e1 = error;
	u2 = u1;
	u1 = u;
	return u;
}

void Controller::Set(float undamptFreq, float damping, float sampleTime)
{
	a = sampleTime*sampleTime*undamptFreq*undamptFreq;
	b = 2 * damping*undamptFreq*sampleTime;
}

Controller::~Controller()
{
}
