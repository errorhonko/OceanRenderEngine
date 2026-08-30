#pragma once
#include "Light.h"
#include <cmath>
#include <limits>
class DistantLight :public Light
{
public:
	DistantLight(const Vector3f& directionToLight,
		const Spectrum& radiance
	)
		:directionToLight(directionToLight.normalize()), radiance(radiance)
	{
	}

	std::optional<LightLiSample> SampleLi(
		const LightSampleContext& ctx,
		const Point2f& u) const override
	{
		(void)ctx;
		(void)u;
		if (directionToLight.near_zero())
			return std::nullopt;

		return LightLiSample{ radiance, directionToLight, 1.0f,   std::numeric_limits<float>::infinity() };
	}
	// DistantLight
	LightType Type() const override
	{
		return LightType::DeltaDirection;
	}
	float PDF_Li(
		const LightSampleContext& ctx,
		const Vector3f& wi) const override
	{
		(void)ctx;
		(void)wi;
		return 0.0f;
	}
private:
	Vector3f directionToLight;
	Spectrum radiance;
};
