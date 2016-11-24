#pragma once
#include <WString.h>
class Controller
{
private:
	// Model Variables
	float a;
	float b;
	// State memory
	float x1;
	//float x2;
public:
	Controller();
	float NextState(float);
	void Set(float, float, float);
	void Reset();
	String State();
	~Controller();
};

