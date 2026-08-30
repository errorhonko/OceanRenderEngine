#pragma once
#include "Light.h"
#include "Ray.h"
#include "BRDFUtils.h"

#include <algorithm>
#include <cmath>
#include <limits>

class UniformInfiniteLight : public Light
{
public:
    explicit UniformInfiniteLight(const Spectrum& radiance)
        : radiance(radiance)
    {
    }
	Spectrum Le(const Ray& ray) const override
	{
		(void)ray;
		return radiance;
	}

	std::optional<LightLiSample> SampleLi(
		const LightSampleContext& ctx,
		const Point2f& u) const override
	{
		(void)ctx;
		Vector3f wi = SampleUniformSphere(u);
		return LightLiSample{
			radiance,
			wi,
			UniformSpherePdf(),
			std::numeric_limits<float>::infinity()
		};
	}
	float PDF_Li(
		const LightSampleContext& ctx,
		const Vector3f& wi) const override
	{
		(void)ctx;
		if (wi.near_zero())
			return .0f;
		return UniformSpherePdf();
	}
	// UniformInfiniteLight
	LightType Type() const override
	{
		return LightType::Infinite;
	}
private:
    Spectrum radiance;

};