#pragma once
#include "Hittable.h"
class Sphere :
    public Hittable
{
public :
	Vector3f center; // 球心坐标
	float radius; // 球的半径
	Sphere(const Vector3f& cen, float r) : center(cen), radius(r) {}
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
		rec.normal = (rec.point - center) *(1/ radius);
		return true;
	}
};

