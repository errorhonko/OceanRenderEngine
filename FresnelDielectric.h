#pragma once
#include "Fresnel.h"
#include "BxDF.h"
class FresnelDielectric :public Fresnel
{
public:
	FresnelDielectric(float etaI, float etaT) :etaI(etaI), etaT(etaT) {};
	Spectrum Evaluate(float cosThetaI) const
	{
		return FrDielectric(cosThetaI, etaI, etaT);
	}
private:
	float etaI, etaT;
};

