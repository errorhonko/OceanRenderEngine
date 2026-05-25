#pragma once
#include"Vector3f.h"
class Ray
{
public:
	Vector3f orig; // 光线的起点
	Vector3f dir; // 光线的方向（应该是单位向量）
	Ray(const Vector3f& origin, const Vector3f& direction) : orig(origin), dir(direction.normalize()) {}

	Vector3f at(float t) const {
		return orig + dir * t; // 计算光线在参数 t 处的点
	}
};



