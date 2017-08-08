#include "Controller.h"
#include "robot.h"



Controller::Controller()
{
	x1 = 0;
	x2 = 0;
}

float Controller::NextState(float error)
{
	/// Update state:	In this implementation, x2 is not used
	float x1_new = 1 * x1 + 2 * error;
	float u = 1.8*x1 + 13.8 * error;
	x1 = x1_new;

	System.setGPoutFloat(7, x1);
	return u;
}

float Controller::NextState(float error, float A, float B, float C, float D)
{
	/// Update state:	In this implementation, x2 is not used
	float x1_new = A * x1 + B * error;
	float u = C*x1 + D * error;
	x1 = x1_new;

	System.setGPoutFloat(7, x1);
	return u;
}

void Controller::Reset()
{
	x1 = 0;
	x2 = 0;
}

Controller::~Controller()
{
}


Controller2::Controller2()
{
	q1 = 0;
	q2 = 0;
	q3 = 0;
}

/// Give all in si units
float Controller2::NextState(float velocity, float position, float theta)
{
	// Estimate State
	float A[3][3] = { {1, 0, 0},{0, 1.004, 0.01001},{0, 0.8186, 1.004} };
	float B[3]    = { 0, -0.08345, -0.03409 };
	float C[2][3] = { { 1, 0, 0},{ 0, 1, 0}};
	float D[2]    = { 0, 0 };
	float L[2][3] = { {0.6180, 0, 0},{0, 0.6387, 1.6372} };
	float K[3]    = { -0.9008, -2.4713, -0.2545 };

	float err1 = position - C[0][0] * q1 - C[0][1] * q2 - C[0][2] * q3;
	float err2 = theta    - C[1][0] * q1 - C[1][1] * q2 - C[1][2] * q3;
	float q1n = A[0][0] * q1 + A[0][1] * q2 + A[0][2] * q3 + B[0] * velocity + L[0][0] * err1 + L[1][0] * err2;
	float q2n = A[1][0] * q1 + A[1][1] * q2 + A[1][2] * q3 + B[1] * velocity + L[0][1] * err1 + L[1][1] * err2;
	float q3n = A[2][0] * q1 + A[2][1] * q2 + A[2][2] * q3 + B[2] * velocity + L[0][2] * err1 + L[1][2] * err2;

	q1 = q1n;
	q2 = q2n;
	q3 = q3n;
	System.setGPoutFloat(1, err1); // Sending the position error

	// calculate control
	return -(K[0] * q1 + K[1] * q2 + K[2] * q3);

}

void Controller2::Reset()
{
	q1 = 0;
	q2 = 0;
	q3 = 0;
}

Controller2::~Controller2()
{
}

/// Depricated
float EstimateFriction(float input, float velocity)
{
	float g[6] = { 3000 , 0.3 , 0.08 , 1500 , 1, 1 };
	float Vel_new = velocity + (input > 0 ? 4 : -4);
	float T = g[0] * (tanh(g[1] * Vel_new) - tanh(g[2] * Vel_new))		// Stribeck effect.
		+ g[3] * tanh(g[4] * Vel_new)								// Coulomb effect.
		+ g[5] * Vel_new;											// Viscous dissipation term.
	return T;
}

float EstimateFriction2(float input, float direction)
{
	float T = input*1.3 + 500 * (input > 0 ? 1 : -1);	// viscous + coulomb effect
	//float T = input*1 + 1000 * (input > 0 ? 1 : -1);	// viscous + coulomb effect
	T += (input*direction > 0 ? 0 : (input > 0 ? 1 : -1)) * 2000;	// strikeback effect
	return T;
}