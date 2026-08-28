#pragma once
#include "light.h"
#include "Hittable.h"

#include <cmath>
#include <memory>

class DiffuseAreaLight : public Light
{
public:
	DiffuseAreaLight(
		std::shared_ptr<Hittable> shape,
		const Spectrum& radiance,
		bool twoSided = false)
		: shape(std::move(shape)),
		radiance(radiance),
		twoSided(twoSided)
	{
	}
	std::optional <LightLiSample> SampleLi(
		const LightSampleContext& ctx,
		const Point2f& u
	) const override
	{
		if (!shape)
			return std::nullopt;
		auto shapeSample = shape->Sample(u);

		if (!shapeSample || shapeSample->pdfArea <= .0f)
		{
			return std::nullopt;
		}

		Vector3f delta = shapeSample->p - ctx.p;

		float distanceSquared = delta.dot(delta);

		if (distanceSquared <= .0f)
			return std::nullopt;
		float distance = std::sqrt(distanceSquared);

		Vector3f wi = delta * (1.0f / distance);




		float cosLight = shapeSample->n.dot(-wi);
		Spectrum Le = L(shapeSample->p, shapeSample->n, -wi);

		if (Le.IsBlack())
			return std::nullopt;

		float absCosLight = std::fabs(cosLight);

		if (absCosLight <= .0f)
			return std::nullopt;

		float pdf = shapeSample->pdfArea * distanceSquared / absCosLight;

		if(pdf <=.0f||!std::isfinite(pdf))
		{
			return std::nullopt;
		}

		return LightLiSample{
			Le,
			wi,
			pdf,
			distance
		};
	}

	Spectrum L(const Vector3f& p, const Vector3f& n, const Vector3f& w) const override
	{
		(void)p;
		if (!twoSided && n.dot(w) <= .0f)
			return Spectrum(0.0f);
		return radiance;
	}
private:
	std::shared_ptr<Hittable> shape;
	Spectrum radiance;
	bool twoSided;
};
