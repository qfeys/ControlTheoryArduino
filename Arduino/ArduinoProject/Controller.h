#pragma once
class Controller
{
private:
	// Model Variables
	float a;
	float b;
	// State memory
	float e1;
	float e2;
	float u1;
	float u2;
public:
	Controller();
	float NextState(float);
	void Set(float, float, float);
	~Controller();
};

