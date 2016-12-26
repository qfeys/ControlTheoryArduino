#pragma once
#include <WString.h>
class Controller
{
private:
	// State memory
	float x1;
	float x2;
public:
	Controller();
	float NextState(float);
	void Reset();
	String State();
	~Controller();
};

/// controller for the pendulum
class Controller2
{
public:
	Controller2();
	~Controller2();
	float NextState(float, float, float);
	void Reset();

private:
	// State memory
	float q1;
	float q2;
	float q3;

	float omega;
};

float EstimateFriction(float, float);
float EstimateFriction2(float, float);