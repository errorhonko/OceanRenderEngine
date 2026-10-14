#pragma once
#include"core/math/Vector3f.h"
#include <memory>
#include <utility>
#include "reflection/materials/material.h"
class Light;
class LidarScatteringModel;

class HitRecord
{
public:
	float t; // 光线与物体交点的参数 t
	Vector3f point; // 交点坐标
	float u, v;// 交点处的纹理坐标
	std::shared_ptr<Material> material; // 交点处的材质
    std::shared_ptr<const LidarScatteringModel> lidarScattering;

    // Every successful primitive hit sets BOTH fields, including explicit nulls.
    void SetSurfaceMaterials(std::shared_ptr<Material> renderMaterial,
        std::shared_ptr<const LidarScatteringModel> scattering = nullptr) noexcept
    {
        material = std::move(renderMaterial);
        lidarScattering = std::move(scattering);
    }
	Vector3f dpdu;
	std::shared_ptr<Light> areaLight;
	Vector3f geometricNormal; // ng：真实几何法线
	Vector3f normal;          // ns：着色法线
};
