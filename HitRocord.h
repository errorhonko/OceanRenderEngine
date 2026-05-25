#pragma once
#include"Vector3f.h"
class HitRecord
{
public:
	float	t; // 光线与物体交点的参数 t
	Vector3f point; // 交点坐标
	Vector3f normal; // 交点处的法线

};

