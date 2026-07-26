#pragma once
#include "CoefficientSpectrum.h"
#include "RGBSpectrum.h"
#include "Vector3f.h"
#include "Point2f.h"
using Spectrum = RGBSpectrum;
class BxDF
{
	enum BxDFType
	{
		BSDF_REFLECTION = 1 << 0,
		BSDF_TRANSMISSION = 1 << 1,
		BSDF_DIFFUSE = 1 << 2,
		BSDF_GLOSSY = 1 << 3,
		BSDF_SPECULAR = 1 << 4,
		BSDF_ALL_TYPES = BSDF_DIFFUSE | BSDF_GLOSSY | 
						BSDF_SPECULAR | BSDF_REFLECTION | BSDF_TRANSMISSION,
	};
public:	
	BxDF(BxDFType type) : type(type) {}
	bool MatchesFlags(BxDFType t) const
	{
		return (type & t) == type;
	}
	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi) const = 0;
	virtual Spectrum Sample_f(const Vector3f& wo, Vector3f* wi, const Point2f& sample, float* pdf, BxDFType* sampledType = nullptr) const;
	virtual Spectrum rho(const Vector3f& wo, int nSamples, const Point2f* samples) const;
	float FrDielectric(float cosThetaI, float etaI, float etaT)
	{
		cosThetaI = clamp(cosThetaI, -1.0f, 1.0f);
		bool entering = cosThetaI > 0.0f;
		if (!entering)
		{
			std::swap(etaI, etaT);
			cosThetaI = std::fabs(cosThetaI);
		}
		float sinThetaI = std::sqrt(std::max(0.0f, 1.0f - cosThetaI * cosThetaI));
		float sinthetaT = etaI / etaT * sinThetaI;
		float cosThetaT = std::sqrt(std::max(0.0f, 1.0f - sinthetaT * sinthetaT));
		float Rparl = (etaT * cosThetaI - etaI * cosThetaT) / (etaT * cosThetaI + etaI * cosThetaT);
		float Rperp = (etaI * cosThetaI - etaT * cosThetaT) / (etaI * cosThetaI + etaT * cosThetaT);
		return (Rparl * Rparl + Rperp * Rperp) / 2.0f;
	}
	Spectrum FrConductor(float cosThetaI, const Spectrum& etaI,const Spectrum& etaT, const Spectrum& k)
	{
		cosThetaI = clamp(cosThetaI, -1.0f, 1.0f);
		bool entering = cosThetaI > 0.0f;
		
	}
public:
	const BxDFType type;

};

