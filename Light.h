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

enum class LightType
{
	DeltaPosition,
	DeltaDirection,
	Area,
	Infinite
};
inline bool IsDeltaLight(LightType type)
{
	return type == LightType::DeltaPosition || type == LightType::DeltaDirection;
}
class Light
{
public:
	Light() = default;
	virtual ~Light() = default;
	virtual LightType Type() const = 0;
	virtual std::optional<LightLiSample> SampleLi(
		const LightSampleContext& ctx,
		const Point2f& u) const = 0;
	virtual float PDF_Li(
		const LightSampleContext& ctx,
		const Vector3f& wi
	) const = 0;
	
	virtual Spectrum Le(const Ray& ray) const
	{
		(void)ray;
		return Spectrum(0.0f);
	}
	virtual Spectrum L(
		const Vector3f& p,
		const Vector3f& n,
		const Vector3f& w
	)const
	{
		(void)p;
		(void)n;
		(void)w;
		return Spectrum(0.0f);
	}
private:

};
