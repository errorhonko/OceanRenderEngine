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
	Bounds3f Bounds() const override
	{
		Bounds3f bounds(v0);

		bounds = Union(bounds, v1);
		bounds = Union(bounds, v2);

		return bounds;
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

		Vector3f ng = E1.cross(E2).normalize();
		rec.geometricNormal = ng;

		Vector3f ns = n0 * alpha + n1 * u + n2 * v;

		if (ns.near_zero())
		{
			ns = ng;
		}
		else
		{
			ns = ns.normalize();

			if (ns.dot(ng) < .0f)
			{
				ns = -ns;
			}
		}

		rec.normal = ns;
		//rec.normal = E1.cross(E2).normalize();

		float du1 = uv1.x - uv0.x;
		float dv1 = uv1.y - uv0.y;

		float  du2 = uv2.x - uv0.x;
		float dv2 = uv2.y - uv0.y;

		float uvDet = du1 * dv2 - dv1 * du2;

		if (std::fabs(uvDet) > 1e-8f)
		{
			float invUvDet = 1.0f / uvDet;
			rec.dpdu = (E1 * dv2 - E2 * dv1) * invUvDet;
		}
		else
		{
			Vector3f tangent = E1 - rec.normal * E1.dot(rec.normal);

			if (tangent.near_zero())
			{
				tangent = E2- rec.normal * E2.dot(rec.normal);
			}
			rec.dpdu = tangent.normalize();
		}

		rec.u = alpha * (uv0.x) + u * (uv1.x) + v * (uv2.x);
		rec.v = alpha * (uv0.y) + u * (uv1.y) + v * (uv2.y);
		SetHitAreaLight(rec);
		return true;
	}
	float SurfaceArea() const override
	{
		Vector3f E1 = v1 - v0;
		Vector3f E2 = v2 - v0;
		return 0.5f * E1.cross(E2).norm();
	}
	std::optional<ShapeSample> Sample(const Point2f& u) const override
	{
		float area = SurfaceArea();
		if(area<=.0f)
			return std::nullopt;

		float b0;
		float b1;

		if (u.x < u.y)
		{
			b0 = u.x * 0.5f;
			b1 = u.y - b0;
		}
		else
		{
			b1 = u.y * 0.5f;
			b0 = u.x - b1;
		}
		float b2 = 1.0f - b0 - b1;

		Vector3f p = v0 * b0 + v1 * b1 + v2 * b2;
		Vector3f n = (v1-v0).cross(v2-v0).normalize();

		return ShapeSample{ p, n, 1.0f / area };
	}
};

