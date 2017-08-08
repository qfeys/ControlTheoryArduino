#include "SensorEstimator.h"



SensorEstimator::SensorEstimator(float l)
{
	d = l;
}

float SensorEstimator::EstimateNext(float e, float l)
{
	// Estimate State
	float A[2][2] = { { 1, 0 },{ 0, 1 } };
	float B[2] = { 0, 0.01};
	float C[2][2] = { { 0, 1 },{ 1, -1} };
	float D[2] = { 0.01, -0.01 };
	float L[2][2] = { { 0.04, 0.04 },{ 0.04, 0.0 } };
	float K[2] = { 0, 1 };

	float errE = e - C[0][0] * d - C[0][1] * s;
	float errL = l - C[1][0] * d - C[1][1] * s;
	float dn = A[0][0] * d + A[0][1] * s  + B[0] * v + L[0][0] * errE + L[1][0] * errL;
	float sn = A[1][0] * d + A[1][1] * s  + B[1] * v + L[0][1] * errE + L[1][1] * errL;

	v = (sn - s) / 0.01;
	d = dn;
	s = sn;

	// calculate control
	return (K[0] * d + K[1] * s);
}

void SensorEstimator::Reset(float l)
{
	d = l;
}


SensorEstimator::~SensorEstimator()
{

}

/// Converts the laser output to mm
float ConvertLaserData(float data)
{
	// 3 - 640
	// 30 - 80
	return 338.57 - 0.482 * data;
}