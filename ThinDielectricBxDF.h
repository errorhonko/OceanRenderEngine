#pragma once
#include "BxDF.h"
class ThinDielectricBxDF :public BxDF
{
public:
	ThinDielectricBxDF(float eta) :
		BxDF(BxDFType(BSDF_TRANSMISSION | BSDF_SPECULAR 
			| BSDF_REFLECTION)),
		eta(eta) {
	}
	Spectrum f(const Vector3f& wo, const Vector3f& wi,
		TransportMode mode) const override
	{
		return Spectrum(0.0f);
	}

	std::optional<BSDFSample> Sample_f(const Vector3f& wo,
		const Point2f& u, TransportMode mode = TransportMode::Radiance,
		BxDFReflectionType sampleFlags = BxDFReflectionType::All) const override
	{
		float R = FrDielectric(AbsCosTheta(wo), 1.0f, eta);
		float T = 1.0f - R;

		if (R < 1.0f)
		{
			R += Sqr(T) * R / (1.0f -  Sqr(R));
			T = 1.0f - R;
		}

		float pr = R, pt = T;
		if (!(sampleFlags & BxDFReflectionType::Reflection)) pr = 0.0f;
		if (!(sampleFlags & BxDFReflectionType::Transmission)) pt = 0.0f;
		if (pr == 0.0f && pt == 0.0f)
			return {};
		if (u.x < pr / (pr + pt))
		{
			Vector3f wi(-wo.x, -wo.y, wo.z);
			Spectrum fr(R / AbsCosTheta(wi));
			return BSDFSample(fr, wi, pr / (pr + pt), BxDFType::SpecularReflection);
		}
		else
		{
			Vector3f wi=-wo;
			Spectrum ft(T / AbsCosTheta(wi));
			return BSDFSample(ft, wi, pt / (pr + pt),
				BxDFType::SpecularTransmission);
		}
	}

	float Pdf(const Vector3f& wo, const Vector3f& wi,
		TransportMode mode,
		BxDFReflectionType sampleFlags = BxDFReflectionType::All) const override
	{
		return 0.0f;
	}
	Spectrum rho(const Vector3f& wo, int nSamples,
		const Point2f* samples) const override
	{
		return Spectrum(0.0f);
	}

private:
	float eta;
};

