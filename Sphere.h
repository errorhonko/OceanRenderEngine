#pragma once
#include "Hittable.h"
#include "BRDFUtils.h"
class Sphere :
    public Hittable
{
public :
	Vector3f center; // 球心坐标
	float radius; // 球的半径
	std::shared_ptr<Material> material; // 球的材质
	Sphere(const Vector3f& cen, float r, std::shared_ptr<Material> mat) : center(cen), radius(r), material(mat) {}
	Bounds3f Bounds() const override
	{
		const Vector3f radiusVector(
			radius,
			radius,
			radius);

		return Bounds3f(
			center - radiusVector,
			center + radiusVector);
	}
	bool hit(const Ray& ray, float t_min, float t_max, HitRecord& rec) const override
	{
		Vector3f v = ray.orig - center;
		float a = 1.0f;
		float b = 2.0f * ray.dir.dot(v);
		float c = v.dot(v) - radius * radius;

		float discriminant = b * b - 4.0f * a * c;
		if (discriminant < 0) return false;
		float sqrt_d = std::sqrt(discriminant);
		float root = (-b - sqrt_d) / (2.0f * a);
		if (root < t_min || root > t_max) {
			root = (-b + sqrt_d) / (2.0f * a);
			if (root < t_min || root > t_max) return false;
		}
		rec.t = root;
		rec.point = ray.at(rec.t);
		rec.geometricNormal =
			(rec.point - center) * (1.0f / radius);

		rec.normal = rec.geometricNormal;
		float phi = std::atan2(rec.normal.y, rec.normal.x);
		rec.dpdu = Vector3f(-std::sin(phi), std::cos(phi), 0.0f);
		rec.material = material;
		SetHitAreaLight(rec);
		return true;
	}

	float SurfaceArea() const override
	{
		return 4.0f * Pi * radius * radius;
	}

	std::optional<ShapeSample> Sample(const Point2f& u) const override
	{
		if (radius <= 0.0f)
			return std::nullopt;

		Vector3f n = SampleUniformSphere(u);
		Vector3f p = center + n * radius;
		return ShapeSample{ p, n, 1.0f / SurfaceArea() };
	}
};

