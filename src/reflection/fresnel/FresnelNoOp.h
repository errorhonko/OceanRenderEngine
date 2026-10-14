#pragma once
#include "reflection/bxdfs/BxDF.h"
#include "reflection/fresnel/Fresnel.h"
class FresnelNoOp :public Fresnel
{
public:
	FresnelNoOp() {};
	Spectrum Evaluate(float cosThetaI) const
	{
		return Spectrum(1.0f);
	}
};

