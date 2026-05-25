#pragma once
#include "Ray.h"
#include "HitRocord.h"
class Material
{
public:
	virtual~Material() = default;
	// 输入：入射光 ray_in，碰撞现场 rec
	// 输出：能量衰减 attenuation，散射出的新光线 scattered
	virtual bool scatter(const Ray& ray_in,
		const HitRecord& rec, Vector3f& attenuation, Ray& scattered) const = 0;
	virtual float evalBRDF(const Vector3f& wi,
		const Vector3f& wo, const Vector3f& N, float u, float v) const
	{
		return 0.0f;
	}
		



};

