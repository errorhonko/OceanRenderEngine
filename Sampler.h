#pragma once
#include "Point2f.h"
class Sampler
{
public:
	
	virtual ~Sampler() =default ;
	
	virtual float Get1D() = 0;
	virtual Point2f Get2D() = 0;
private:

};

