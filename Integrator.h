#pragma once
#include "Ray.h"
#include "HitRocord.h"
#include "Hittable.h"
#include "Sampler.h"
#include "camera.h"
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
		) const;
	bool Unoccluded(const Vector3f&p0,
		const Vector3f& p1) const;
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

	void Render()override;
protected:

	Camera camera;
	std::shared_ptr<Sampler> sampler;
};
