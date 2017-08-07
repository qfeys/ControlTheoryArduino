#pragma once
class Tester
{
public:
	Tester();
	~Tester();
	int TestMotor1();
private:
	int counter;
	bool forward;
	int voltage;
};

