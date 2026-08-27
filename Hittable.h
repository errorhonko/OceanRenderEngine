#pragma once
#include	"Ray.h"
#include	"HitRocord.h"
#include "Point2f.h"
#include <optional>

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

};

