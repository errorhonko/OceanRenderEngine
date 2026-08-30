#pragma once
#include	"Ray.h"
#include	"HitRocord.h"
#include "Point2f.h"
#include <optional>
#include <memory>
#include <cmath>
#include <limits>
class Light;
struct ShapeSample
{
	Vector3f p;       // 采样到的表面位置
	Vector3f n;       // 该位置的法线
	float pdfArea;    // 关于表面积的 PDF
};

class Hittable
{
public:
	virtual ~Hittable() = default;
	virtual bool hit(
		const Ray& ray,
		float t_min,
		float  t_max,
		HitRecord& rec)const = 0;

	virtual std::optional<ShapeSample> Sample(
		const Point2f& u) const
	{
		(void)u;
		return std::nullopt;
	}

	virtual float SurfaceArea() const
	{
		return 0.0f;
	}
	void SetAreaLight(const std::shared_ptr<Light>& light)
	{
		areaLight = light;
	}

	virtual float PDF(const Vector3f& p, const Vector3f& wi) const
	{
		float area = SurfaceArea();
		if (area <= 0.0f || wi.near_zero())
			return 0.0f;

		Vector3f direction = wi.normalize();

		HitRecord rec;
		Ray ray(p, direction);

		if (!hit(
			ray,
			1e-4f,
			std::numeric_limits<float>::infinity(),
			rec
		))
		{
			return 0.0f;
		}
		Vector3f delta = rec.point - p;
		float distanceSquared = delta.dot(delta);
		float absCosLight = std::abs(rec.geometricNormal.dot(-direction));

		if (absCosLight <= .0f)
			return .0f;

		float pdf = distanceSquared / (absCosLight * area);
		return std::isfinite(pdf) ? pdf : 0.0f;
	}

protected :
	void SetHitAreaLight(HitRecord& rec) const
	{
		rec.areaLight = areaLight.lock();
	}

private:
	std::weak_ptr<Light> areaLight;
};

