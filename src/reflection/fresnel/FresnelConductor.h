#pragma once
#include "reflection/fresnel/Fresnel.h"
#include "reflection/bxdfs/BxDF.h"

class FresnelConductor : public Fresnel
{
public:
	FresnelConductor(const Spectrum& etaI, const Spectrum& etaT,
		const Spectrum& k) :etaI(etaI), etaT(etaT), k(k) {
	};
	Spectrum Evaluate(float cosThetaI) const override
	{
		return FrConductor(std::fabs(cosThetaI), etaI, etaT, k);
	}

private:
	Spectrum etaI, etaT, k;
};

