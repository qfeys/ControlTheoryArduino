#pragma once
class Tester
{
public:
	Tester();
	~Tester();
	int TestRandom();
	int TestSinus();
private:
	int counter;
	bool forward;
	int voltage;
};

