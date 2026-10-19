#pragma once
#include "reflection/bxdfs/BxDF.h"
#include "reflection/microfacet/TrowbridgeReitzDistribution.h"
#include "reflection/microfacet/MicrofacetReflection.h"
class ConductorBxDF :public BxDF
{
public:
    using BxDF::Sample_f;
ConductorBxDF(const TrowbridgeReitzDistribution& distrib,
		const Spectrum& eta, const Spectrum& k)
		: BxDF(BxDFType::GlossyReflection)
		,mfDistrib(distrib), eta(eta), k(k) {}
Spectrum f(const Vector3f& wo, const Vector3f& wi, TransportMode mode) const override
	{
		if (!SameHemisphere(wo, wi)) return{};
		if (mfDistrib.EffectivelySmooth())return {};
		// Keep the existing two-hemisphere convention. GGX D/G are invariant
		// under reversing both directions; the shared kernel uses local +Z.
		const Vector3f upperWo = wo.z < 0 ? -wo : wo;
		const Vector3f upperWi = wo.z < 0 ? -wi : wi;
		return EvaluateMicrofacetConductorReflection(mfDistrib, upperWo, upperWi,
			[this](double cosine) { return FrComplex(float(cosine), eta, k); });
	}
std::optional<BSDFSample> Sample_f(const Vector3f& wo,
		const Point2f& u, TransportMode mode = TransportMode::Radiance,
		BxDFReflectionType sampleFlags = BxDFReflectionType::All) const override
	{
		if (!(sampleFlags & BxDFReflectionType::Reflection))
			return{};
		if (mfDistrib.EffectivelySmooth())
		{
			Vector3f wi(-wo.x, -wo.y, wo.z);
			Spectrum f = FrComplex(AbsCosTheta(wi), eta, k) / AbsCosTheta(wi);
			return BSDFSample(f, wi, 1, BxDFType::SpecularReflection);

		}
		Vector3f wm = mfDistrib.Sample_Wm(wo, u);
		Vector3f wi = Reflect(wo, wm);
		if (!SameHemisphere(wo, wi))return{};
		float pdf = mfDistrib.PDF(wo, wm) / (4 * AbsDot(wo, wm));
		
		
		return BSDFSample(f(wo,wi,mode), wi, pdf, BxDFType::GlossyReflection);
	}
float Pdf(const Vector3f& wo, const Vector3f& wi, TransportMode mode,
	BxDFReflectionType sampleFlags) const override
{
	if (!(sampleFlags & BxDFReflectionType::Reflection)) return 0;
	if (!SameHemisphere(wo, wi)) return 0;
	if (mfDistrib.EffectivelySmooth())return 0;
	Vector3f wm = wo + wi;
	if (wm.near_zero())return 0;
	wm = FaceForward(wm.normalize(), Vector3f(0, 0, 1));
	return mfDistrib.PDF(wo, wm) / (4 * AbsDot(wo, wm));
	
}
void Regularize() { mfDistrib.Regularize(); }
Spectrum rho(const Vector3f& wo, int nSamples, const Point2f* samples) const override
{
	return Spectrum(0.0f);
}

private:
	TrowbridgeReitzDistribution mfDistrib;
	Spectrum eta, k;

};

