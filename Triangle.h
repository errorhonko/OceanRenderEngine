#pragma once
#include "Hittable.h"
#include "material.h"
class Triangle :
    public Hittable
{
public:
    Vector3f v0, v1, v2;
	Vector3f n0, n1, n2;
	Vector3f uv0, uv1, uv2;
	std::shared_ptr<Material> mat;
    Triangle(const Vector3f& p0, const Vector3f& p1, const Vector3f& p2,
             const Vector3f& n0, const Vector3f& n1, const Vector3f& n2,
             const Vector3f& t0, const Vector3f& t1, const Vector3f& t2,
             std::shared_ptr<Material> m)
        : v0(p0), v1(p1), v2(p2), 
          n0(n0), n1(n1), n2(n2), 
          uv0(t0), uv1(t1), uv2(t2), 
          mat(m) {}

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
		float u = S.dot(S1) * invDet; // 顶点 v1 的权重
		if (u < 0.0f || u > 1.0f) return false;
		Vector3f S2= S.cross(E1);
		float v = r.dir.dot(S2) * invDet; // 顶点 v2 的权重
		if (v < 0.0f || u + v > 1.0f) return false;

		float t = E2.dot(S2) * invDet; // 光线求交参数 t
		if (t < t_min || t > t_max) return false;

		rec.t = t;
		rec.point = r.at(rec.t);
		rec.material = this->mat; // 赋值材质
		float alpha = 1.0f - u - v;

		rec.normal = ( n0 * alpha +  n1 * u + n2 * v).normalize();
		//rec.normal = E1.cross(E2).normalize();
		rec.u = alpha * (uv0.x) + u * (uv1.x) + v * (uv2.x);
		rec.v = alpha * (uv0.y) + u * (uv1.y) + v * (uv2.y);

		return true;
	}
};

