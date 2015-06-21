#ifndef STATSTOOLBOX_H
#define STATSTOOLBOX_H

#include <stdlib.h>
#include <time.h>
#include <math.h>

class StatsToolbox
{
	public:
		StatsToolbox();
		~StatsToolbox(){}
		
		void init();
		
		double randu(double minValue=0, double maxValue=1); //uniform random number generator ]minValue,maxValue)
		int randu(int minValue=0, int maxValue=10); //uniform random number generator ]minValue,maxValue)
		
		//can be improved
		double randn(double mean, double std); //normal random number generator
	
	protected:
		
		double _pi;
	
};

#endif
