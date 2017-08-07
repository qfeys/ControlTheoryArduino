#include "Tester.h"
#include "robot.h"



Tester::Tester()
{
}


Tester::~Tester()
{
}

int Tester::TestMotor1()
{
	counter--;
	if (counter <= 0)
	{
		counter = random(40) + 1;
		// forward = !forward;
		voltage = random(-6000, 6000);
	}
	return voltage;
}
