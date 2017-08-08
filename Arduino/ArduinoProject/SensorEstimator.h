#pragma once
class SensorEstimator
{
public:
	SensorEstimator(float l);
	float EstimateNext(float e, float l, float v);
	~SensorEstimator();
private:
	float d;
	float s;
};

