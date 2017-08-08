#pragma once
class Tester
{
public:
	Tester();
	~Tester();
	int TestRandom();
	int TestRandom2();
	int TestSinus();
private:
	int counter;
	bool forward;
	int voltage;
	int setpoint;
};

