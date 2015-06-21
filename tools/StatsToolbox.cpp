#include "StatsToolbox.h"

StatsToolbox::StatsToolbox()
{
}

void StatsToolbox::init()
{
	this->_pi = 3.14159265359;
	srand(Time(NULL))
}

double StatsToolbox::randu(double minValue, double maxValue)
{
	double result = double(rand()+1)/double(RAND_MAX);
	result = result*(maxValue - minValue) + minValue;
	return result;
}
int StatsToolbox::randu(int minValue, int maxValue)
{
	double result = randu(double(minValue), double(maxValue))
	return round(result);
}
	
double StatsToolbox::randn(double mean, double std)
{
	double U1 = randu();
	double U2 = randu();
		
	X = sqrt(-2*log(U1))*cos(2*_pi*U2);
	Y = sqrt(-2*log(U1))*sin(2*_pi*U2);
	X = X*std + mean;
	Y = Y*std + mean;
	
	return X;
}
