#pragma once
#include "Ray.h"
#include "HitRocord.h"
#include "Hittable.h"
#include "Sampler.h"
#include "camera.h"
#include <limits>
inline static constexpr float RayEpsilon = 1e-4f;
class Integrator
{
public:
	explicit Integrator(
		const Hittable& world)
		: world(world)
	{
	}
	virtual ~Integrator()=default;
	virtual void Render() = 0;
private:
protected:
	bool Intersect(
		const Ray& ray,
		HitRecord& rec
	) const
	{
		return	 world.hit(ray, RayEpsilon,
			std::numeric_limits<float> ::infinity(),
			rec);
	}
	Vector3f OffsetRayOrigin(
		const Vector3f& p,
		const Vector3f& geometricNormal,
		const Vector3f& direction
	) const
	{
		Vector3f offsetNormal = geometricNormal.dot(direction) >= 0.0f
			? geometricNormal
			: -geometricNormal;

		return p + offsetNormal * RayEpsilon;
	}
	bool Unoccluded(const Vector3f& p,
		const Vector3f& geometricNormal,
		const Vector3f& wi,
		float distance
	) const {
		if (std::isfinite(distance) &&
			distance <= 2.0f * RayEpsilon)
		{
			return true;
		}
		Ray shadowRay(
			OffsetRayOrigin(p, geometricNormal, wi),
			wi);
		float tMax = std::isfinite(distance)
			? distance - 2.0f* RayEpsilon
			: std::numeric_limits<float>::infinity();

		HitRecord shadowRec;

		return !world.hit(shadowRay, .0f, tMax, shadowRec);
	}
	const Hittable& world;

};

class RayIntegrator :public Integrator
{
public:
	RayIntegrator(
		const Hittable& world,
		Camera camera,
		std::shared_ptr<Sampler> sampler)
		: Integrator(world),
		camera(std::move(camera)),
		sampler(std::move(sampler))
	{
	}
	virtual Spectrum Li(Ray ray, Sampler& sampler) const = 0;

	void Render()override
	{
	};
protected:

	Camera camera;
	std::shared_ptr<Sampler> sampler;
};
