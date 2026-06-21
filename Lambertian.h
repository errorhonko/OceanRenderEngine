#pragma once
#include "material.h"
#include "Texture.h"
class Lambertian: public Material
{
public :
	std::shared_ptr<Texture> albedo;
	Lambertian(std::shared_ptr<Texture> a) : albedo(a) {};
	bool scatter(const Ray& ray_in, const HitRecord& rec, Vector3f& attenuation, Ray& scattered) const override
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

