#pragma once
class SensorEstimator
{
public:
	SensorEstimator(float l);
	float EstimateNext(float e, float l);
	~SensorEstimator();
private:
	float d;
	float s;
	float v;
};

float ConvertLaserData(float data);