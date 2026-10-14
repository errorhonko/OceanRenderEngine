#pragma once
#include "reflection/materials/material.h"
#include "reflection/textures/Texture.h"
#include "core/ray/Ray.h"
#include "core/ray/HitRocord.h"
class Lambertian: public Material
{
public :
	std::shared_ptr<Texture> albedo;
	Lambertian(std::shared_ptr<Texture> a) : albedo(a) {};
	bool scatter(const Ray& ray_in, const HitRecord& rec, Vector3f& attenuation, Ray& scattered) const
	{
		Vector3f scatter_direction = rec.normal + Vector3f::random_unit_vector();
		if (scatter_direction.near_zero())
		{
			scatter_direction = rec.normal;
		}
		scattered = Ray(rec.point, scatter_direction);
		attenuation = albedo->value(rec.u, rec.v);
		return true;
	}
};

