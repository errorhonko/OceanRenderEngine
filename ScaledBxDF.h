#pragma once
#include "BxDF.h"
class ScaledBxDF : public BxDF
{
public:
	ScaledBxDF(BxDF* bxdf, const Spectrum& scale) :
		BxDF(bxdf->type), bxdf(bxdf), scale(scale) {}
	Spectrum f(const Vector3f& wo, const Vector3f& wi) const override
	{
		return scale * bxdf->f(wo, wi);
	}
private:
	BxDF* bxdf;
	Spectrum scale;
};
