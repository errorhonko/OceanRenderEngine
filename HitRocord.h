#pragma once
#include"Vector3f.h"
#include <memory>
class Material; 
class HitRecord
{
public:
	float	t; // 光线与物体交点的参数 t
	Vector3f point; // 交点坐标
	Vector3f normal; // 交点处的法线
	float u, v;// 交点处的纹理坐标
	std::shared_ptr<Material> material; // 交点处的材质
};

