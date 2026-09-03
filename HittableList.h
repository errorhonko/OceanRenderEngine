#pragma once
#include <vector>
#include <memory>
#include "Hittable.h"
class HittableList :public Hittable
{
public:
	std::vector<std::shared_ptr<Hittable>> objects;
	HittableList() {}
	HittableList(std::shared_ptr<Hittable> object) { add(object); }
	void clear() { objects.clear(); }
	void add(std::shared_ptr<Hittable> object) {
		objects.push_back(object);
	}
	Bounds3f Bounds() const override
	{
		Bounds3f bounds;

		for (const auto& object : objects)
		{
			bounds = Union(
				bounds,
				object->Bounds());
		}

		return bounds;
	}
	bool hit(const Ray& r, float t_min, float t_max, HitRecord& rec) const override
	{
		HitRecord temp_rec;
		bool hit_anything = false;
		float closest_so_far = t_max;
		for (const auto& object : objects) {
			if (object->hit(r, t_min, closest_so_far, temp_rec)) {
				hit_anything = true;
				closest_so_far = temp_rec.t;
				rec = temp_rec; // 更新 rec 以保存最近的命中记录
			}
		}
		return hit_anything;
	}
};

