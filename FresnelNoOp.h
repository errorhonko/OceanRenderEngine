#pragma once
#include "BxDF.h"
#include "Fresnel.h"
class FresnelNoOp :public Fresnel
{
public:
	FresnelNoOp() {};
	Spectrum Evaluate(float cosThetaI) const
	{
		return Spectrum(1.0f);
	}
};

