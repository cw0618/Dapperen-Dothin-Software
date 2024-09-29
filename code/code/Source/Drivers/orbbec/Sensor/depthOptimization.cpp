#include "depthOptimization.h"
#include <math.h>

bool depthOptTableInited = false;
unsigned short depthOptTable[4001] = {0};
#define DEPTH_SCALE_MM 0
#define DEPTH_SCALE_100UM 1

void depthOptTableInit(const double param1, const double param2, const double param3)
{
	if(depthOptTableInited)
		return;

	double a = param3;
	double b = param2 + 1;
	double c = 0;

	for(int i = 0; i < 100; i++)
		depthOptTable[i] = 0;

//#pragma omp parallel for
	for(int i = 100; i < 4001; i++)
	{
		unsigned short d = i;
		c = param1 - d;
		double delta = b * b - 4 * a * c;
		if(delta > 0)
			depthOptTable[i] = (unsigned short)((-b + sqrt(delta))/(2 * a));
		else
			depthOptTable[i] = d;
	}
}

int depthOptimization(unsigned short* orginDepth, unsigned int numDepth, unsigned short* optDepth,
			const double param1, const double param2, const double param3,unsigned int depthScale)
{
	if(depthOptTableInited == false)
	{
		depthOptTableInit(param1,param2,param3);
		depthOptTableInited = true;
	}

//#pragma omp parallel for
	if (depthScale == DEPTH_SCALE_100UM)
	{
		for (unsigned int i = 0; i < numDepth; i++)
		{
			unsigned short d = orginDepth[i] /10;

			if (d <= 4000)
			{
				optDepth[i] = depthOptTable[d] *10;
			}
			else
			{
				optDepth[i] = 0;
			}
		}
	}
	else
	{
		for (unsigned int i = 0; i < numDepth; i++)
		{
			unsigned short d = orginDepth[i];

			if (d <= 4000)
			{
				optDepth[i] = depthOptTable[d];
			}
			else
			{
				optDepth[i] = 0;
			}
		}
	}

	return 0;
}
