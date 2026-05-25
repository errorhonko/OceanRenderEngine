#pragma once
#include "Hittable.h"
class Triangle :
    public Hittable
{
public:
    Vector3f v0, v1, v2;
    Triangle(const Vector3f& p0, const Vector3f& p1, const Vector3f& p2)
        : v0(p0), v1(p1), v2(p2) {
    }

	bool hit(const Ray& r, float t_min, float t_max, HitRecord& rec) const override
	{
		Vector3f E1 = v1 - v0;
		Vector3f E2 = v2 - v0;
		Vector3f S1 = r.dir.cross(E2);
		float  det = E1.dot(S1);
		//if (det < 1e-5f) return false;//关闭双面渲染
		if (det > -1e-5f && det < 1e-5f) return false;
		float invDet = 1.0f / det;

		Vector3f S = r.orig - v0;
		float u = S.dot(S1) * invDet;
		if (u < 0.0f || u > 1.0f) return false;
		Vector3f S2= S.cross(E1);
		float v = r.dir.dot(S2) * invDet;
		if (v < 0.0f || u + v > 1.0f) return false;

		float t = E2.dot(S2) * invDet;
		if (t < t_min || t > t_max) return false;

		rec.t = t;
		rec.point = r.at(rec.t);
		rec.normal = E1.cross(E2).normalize();
		return true;
	}
};

