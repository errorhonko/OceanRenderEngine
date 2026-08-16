#pragma once
#include "BRDFUtils.h"
#include "BxDF.h"
using namespace BRDFUtils;	
class DiffuseBxDF :public BxDF
{
public:
	DiffuseBxDF(Spectrum R) :BxDF(BxDFType::DiffuseReflection), R(R) {}
	Spectrum f(const Vector3f& wo, const Vector3f& wi ,TransportMode mode) const override
	{
		if (!SameHemisphere(wo, wi))
			return Spectrum(0.f);
		return R * InvPi;
	}
	std::optional<BSDFSample> Sample_f(const Vector3f& wo, 
		const Point2f& sample, TransportMode mode,
		BxDFReflectionType sampledType=BxDFReflectionType::All) const override
	{
		if(!(sampledType & BxDFReflectionType::Reflection)) 
			return {};
		Vector3f wi = SampleCosineHemisphere(sample);
		if (wo.z < 0) wi.z *= -1;
		float pdf = CosineHemispherePdf(AbsCosTheta(wi));
		return BSDFSample(R*InvPi, wi, pdf,BxDFType::DiffuseReflection);
	}
	float Pdf(const Vector3f& wo, const Vector3f& wi, TransportMode mode,
		BxDFReflectionType sampleFlags = BxDFReflectionType::All) const override
	{
		if (!(sampleFlags & BxDFReflectionType::Reflection)||!SameHemisphere(wo,wi))
			return 0.f;
		return CosineHemispherePdf(AbsCosTheta(wi));
	}
	Spectrum rho(const Vector3f& wo, int nSamples, const Point2f* samples) const override
	{
		return R;
	}

private:
	Spectrum R;
};

