#pragma once
#include "BxDF.h"
#include "Fresnel.h"
#include "FresnelDielectric.h"
#include "BRDFUtils.h"
class SpecularTransmission:public BxDF
{
public:
	SpecularTransmission(const Spectrum& T, 
		float etaA, float etaB, TransportMode mode) :
		BxDF(BxDFType(BSDF_TRANSMISSION | BSDF_SPECULAR)),
		T(T), etaA(etaA), etaB(etaB), fresnel(etaA,etaB), mode(mode) {
	}
private:
	const Spectrum T;
	const float etaA, etaB;
	const TransportMode mode;
	const FresnelDielectric fresnel;

};

