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
		counter = 3; // random(10) + 1;
		// forward = !forward;
		voltage = random(-6000, 6000);
	}
	return voltage;
}

int Tester::TestSinus()
{
	counter++;
	return sin(counter / 50.0) * 3000;
}
