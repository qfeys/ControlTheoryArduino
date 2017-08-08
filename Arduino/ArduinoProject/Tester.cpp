#include "Tester.h"
#include "robot.h"
#include <math.h>



Tester::Tester()
{
}


Tester::~Tester()
{
}

int Tester::TestRandom()
{
	counter--;
	if (counter <= 0)
	{
		counter = random(90) + 10;
		// forward = !forward;
		voltage = random(-3000, 3000);
	}
	return voltage;
}

int Tester::TestRandom2()
{
	counter--;
	voltage += (setpoint - voltage) / 5;
	if (counter <= 0)
	{
		counter = 5;
		setpoint += random(-500, 500) - setpoint / 20;
	}
	return voltage;
}

int Tester::TestSinus()
{
	counter++;
	return sin(counter / 50.0) * 3000;
}
