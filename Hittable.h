#pragma once
#include	"Ray.h"
#include	"HitRocord.h"
class Hittable
{
public:
	virtual ~Hittable() = default;
	virtual bool hit(const Ray& ray, float t_min, float t_max, HitRecord& rec) const = 0;
};

