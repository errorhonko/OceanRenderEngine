#pragma once
#include "BxDF.h"
#include "Fresnel.h"
#include "BRDFUtils.h"
#include <math.h>
#include <algorithm>
class SpecularReflection :
    public BxDF
{
public:
	SpecularReflection(const Spectrum& R, Fresnel* fresnel) :
		BxDF(BxDFType(BSDF_REFLECTION | BSDF_SPECULAR)),
		R(R), fresnel(fresnel) {
	}
	Spectrum f(const Vector3f& wo, const Vector3f& wi) const override
	{
		return Spectrum(0.0f);
	}
	std::optional<BSDFSample>  Sample_f(const Vector3f& wo, Vector3f* wi,
		const Point2f& sample, float* pdf, 
		BxDFType* sampledType= nullptr ) const override
	{
		*wi = Vector3f(-wo.x, -wo.y, wo.z);
		*pdf = 1.0f;
		if (sampledType) *sampledType = BxDFType::BSDF_SPECULAR;

		return fresnel->Evaluate(
			BRDFUtils::CosTheta(wo)) * R / std::fabs(BRDFUtils::CosTheta(*wi));
		
	}
private:
	const Spectrum R;
	const Fresnel* fresnel;

};

