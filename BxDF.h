#pragma once
#include "CoefficientSpectrum.h"
#include "RGBSpectrum.h"
#include "Vector3f.h"
#include "Point2f.h"
#include "BRDFUtils.h"
#include <algorithm>
#include <cmath>
#include <optional>
#include <complex>
using std::clamp;
using namespace BRDFUtils;
using Spectrum = RGBSpectrum;
enum BxDFType
{
	Unset = 0,
	BSDF_REFLECTION = 1 << 0,
	BSDF_TRANSMISSION = 1 << 1,
	BSDF_DIFFUSE = 1 << 2,
	BSDF_GLOSSY = 1 << 3,
	BSDF_SPECULAR = 1 << 4,
	DiffuseReflection = BSDF_DIFFUSE | BSDF_REFLECTION,
	DiffuseTransmission = BSDF_DIFFUSE | BSDF_TRANSMISSION,
	GlossyReflection = BSDF_GLOSSY | BSDF_REFLECTION,
	GlossyTransmission = BSDF_GLOSSY | BSDF_TRANSMISSION,
	SpecularReflection = BSDF_SPECULAR | BSDF_REFLECTION,
	SpecularTransmission = BSDF_SPECULAR | BSDF_TRANSMISSION,
	BSDF_ALL_TYPES = BSDF_DIFFUSE | BSDF_GLOSSY |
	BSDF_SPECULAR | BSDF_REFLECTION | BSDF_TRANSMISSION,
};
enum class BxDFReflectionType
{
	Unset = 0,
	Reflection = 1 << 0,
	Transmission = 1 << 1,
	All = Reflection | Transmission
};
inline BxDFReflectionType operator|(BxDFReflectionType a, BxDFReflectionType b)
{
	return static_cast<BxDFReflectionType>(static_cast<int>(a) | static_cast<int>(b));
}
inline BxDFReflectionType operator&(BxDFReflectionType a, BxDFReflectionType b)
{
	return static_cast<BxDFReflectionType>(static_cast<int>(a) & static_cast<int>(b));
}
inline bool operator!(BxDFReflectionType a)
{
	return static_cast<int>(a) == 0;
}
struct BSDFSample
{
	BSDFSample(Spectrum f, Vector3f wi, float pdf, BxDFType flags, float eta=1.0f, bool pdfIsProportional=false)
		:f(f), wi(wi), pdf(pdf), flags(flags), eta(eta), pdfIsProportional(pdfIsProportional) {
	}
	BSDFSample();
	Spectrum  f;
	Vector3f wi;
	float pdf = 0;
	BxDFType flags;
	float eta = 1;
	bool pdfIsProportional = false;
};
class BxDF
{
public:
	

	BxDF(BxDFType type) : type(type) {}
	bool MatchesFlags(BxDFType t) const
	{
		return (type & t) == type;
	}
	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi, TransportMode mode) const = 0;
	virtual std::optional<BSDFSample> Sample_f(const Vector3f& wo,
		const Point2f& sample,
		TransportMode mode= TransportMode::Radiance,
		BxDFReflectionType sampledType = BxDFReflectionType::All) const=0;
	virtual Spectrum rho(const Vector3f& wo, int nSamples, const Point2f* samples) const=0;
	virtual float Pdf(const Vector3f& wo, const Vector3f& wi , 
		TransportMode mode,
		BxDFReflectionType sampleFlags = BxDFReflectionType::All) const=0;
public:
	const BxDFType type;

};
inline float FrDielectric(float cosThetaI, float etaI, float etaT)
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

inline float FrComplex(float cosTheta_i, std::complex<float> eta)
{
	cosTheta_i = std::clamp(cosTheta_i, 0.0f, 1.0f);
	float sin2Theta_i = std::max(0.0f, 1.0f - cosTheta_i * cosTheta_i);
	std::complex<float> sin2Theta_t = sin2Theta_i / (eta * eta);
	std::complex<float>cosTheta_t = std::sqrt(std::complex<float>(1.0f, 0.0f) - sin2Theta_t);
	std::complex<float> r_parl = (eta * cosTheta_i - cosTheta_t) / (eta * cosTheta_i + cosTheta_t);
	std::complex<float> r_perp = (cosTheta_i - eta * cosTheta_t) / (cosTheta_i + eta * cosTheta_t);
	return (std::norm(r_parl) + std::norm(r_perp)) * 0.5f;
}
inline Spectrum FrComplex(float cosTheta_i, const Spectrum& eta,
	const Spectrum& k)
{
	Spectrum result;
	for (size_t i = 0;i < result.c.size();i++)
	{
		result.c[i] = FrComplex(cosTheta_i,
			std::complex<float>(eta.c[i], k.c[i]));
	}
	return result;
}
inline Vector3f Reflect(const Vector3f& wo, const Vector3f& n)
{
	return -wo + 2.0f * wo.dot(n) * n;
}
inline bool Refract(const Vector3f& wi,  Vector3f& n
	, float eta, float* etap, Vector3f* wt)
{
	float cosTheta_i = n.dot(wi);
	if (cosTheta_i < 0)
	{
		eta = 1 / eta;
		cosTheta_i = -cosTheta_i;
		n = -n;
	}
	float sin2Theta_i = std::max(0.0f, 1.0f - cosTheta_i * cosTheta_i);
	float sin2Theta_t = sin2Theta_i / (eta * eta);
	float cosTheta_t = Safesqrt(1 - sin2Theta_t);
	*wt = -wi / eta + (cosTheta_i / eta - cosTheta_t) * n;
	if (etap)
		*etap = eta;
	return true;

}
