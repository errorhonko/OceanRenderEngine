#pragma once
#include "Light.h"
#include <cmath>

class PointLight : public Light
{
public:
	PointLight(const Vector3f& position, const Spectrum& intensity)
		:position(position), intensity(intensity)
	{
	}
	std::optional <LightLiSample> SampleLi(
		const LightSampleContext& ctx,
		const Point2f& u) const override
	{
		Vector3f wi = position - ctx.p;
		float distanceSquared = wi.dot(wi);
		if (distanceSquared <= 0.0f)
			return std::nullopt;
		float distance = std::sqrt(distanceSquared);
		wi = wi.normalize();
		float pdf = 1.0f;
		Spectrum L = intensity / distanceSquared;
		return LightLiSample{ L, wi, pdf, distance };
	}
private:
	Vector3f position;
	Spectrum intensity;
};
