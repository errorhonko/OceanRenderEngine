#pragma once
#include "Vector3f.h"
#include "BxDF.h"
#include "Point2f.h"
#include <optional>

class Ray;

struct LightSampleContext
{
	Vector3f p;
	Vector3f n;
	Vector3f ns;
};
struct  LightLiSample
{
	Spectrum L;
	Vector3f	wi;
	float pdf;
	float distance;
};
class Light
{
public:
	Light()= default;
	virtual ~Light() = default;
	virtual std::optional<LightLiSample> SampleLi(
		const LightSampleContext& ctx,
		const Point2f& u) const = 0;
	virtual Spectrum Le(const Ray& ray) const
	{
		(void)ray;
		return Spectrum(0.0f);
	}
private:

};
