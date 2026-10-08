#pragma once

#include "BeckmannDistribution.h"
#include "LidarScatteringModel.h"
#include "OceanSlopeVariance.h"
#include "OceanNormalDistribution.h"
#include "MicrofacetReflection.h"
#include <cmath>
#include <stdexcept>

class OceanBeckmannLidarScattering final
    : public LidarScatteringModel
{
public:
    OceanBeckmannLidarScattering(
        const OceanSlopeVariance& unresolvedSlopes,
        double etaIncident,
        double etaTransmitted)
        : unresolvedSlopes(unresolvedSlopes),
        etaIncident(etaIncident),
        etaTransmitted(etaTransmitted)
    {
        if (!std::isfinite(etaIncident) ||
            !std::isfinite(etaTransmitted) ||
            etaIncident <= 0.0 ||
            etaTransmitted <= 0.0)
        {
            throw std::invalid_argument(
                "Dielectric indices must be finite and positive.");
        }

        // 复用 Beckmann 的检查：
        // 方差必须为正且有限，协方差矩阵必须正定。
        // 当前有限粗糙度模型不处理零方差的 Delta 反射。
        const BeckmannDistribution validated(
            unresolvedSlopes.varianceX,
            unresolvedSlopes.varianceZ,
            unresolvedSlopes.covarianceXZ);

        (void)validated;
    }

    double Evaluate(
        const HitRecord& hit,
        const Vector3f& incidentDirection,
        const Vector3f& outgoingDirection,
        float wavelengthNm) const override
    {
        auto normalizeChecked = [](const Vector3f& v)
            {
                const double length =
                    std::hypot(double(v.x), double(v.y), double(v.z));

                if (!std::isfinite(length) || length == 0.0)
                {
                    throw std::invalid_argument(
                        "Ocean scattering vector must be finite and nonzero.");
                }

                return Vector3f(
                    float(double(v.x) / length),
                    float(double(v.y) / length),
                    float(double(v.z) / length));
            };

        if (!std::isfinite(wavelengthNm) || wavelengthNm <= 0.0f)
        {
            throw std::invalid_argument(
                "Wavelength must be finite and positive.");
        }

        // 两个世界方向都指向表面外侧。
        const Vector3f wiWorld =
            normalizeChecked(incidentDirection);

        const Vector3f woWorld =
            normalizeChecked(outgoingDirection);

        const Vector3f ng =
            normalizeChecked(hit.geometricNormal);

        // 几何法线约束实际表面的正面。
        if (ng.dot(wiWorld) <= 0.0f ||
            ng.dot(woWorld) <= 0.0f)
        {
            return 0.0;
        }

        Vector3f surfaceNormal = hit.normal;

        const double normalLength = std::hypot(
            double(surfaceNormal.x),
            double(surfaceNormal.y),
            double(surfaceNormal.z));

        // 着色法线不可用时退回几何法线。
        if (!std::isfinite(normalLength) || normalLength == 0.0)
            surfaceNormal = ng;
        else
            surfaceNormal = normalizeChecked(surfaceNormal);

        if (surfaceNormal.dot(ng) < 0.0f)
            surfaceNormal = -surfaceNormal;

        // 当前海面模型要求世界 +Y 朝上，海面是 X/Z 上的高度场。
        const auto mapped =
            OceanNormalDistribution::FromWorldSlopeVarianceLinearized(
                unresolvedSlopes,
                surfaceNormal);

        // 必须使用映射坡度时的同一个坐标系。
        const Frame& frame = mapped.SurfaceFrame();

        const Vector3f wi = frame.ToLocal(wiWorld);
        const Vector3f wo = frame.ToLocal(woWorld);

        return EvaluateMicrofacetDielectricReflection(
            mapped.LocalDistribution(),
            wo,
            wi,
            etaIncident,
            etaTransmitted);
    }

private:
    // 未解析短波在世界水平 X/Z 坐标中的坡度统计，
    // 不是某个交点局部坐标中的 alpha。
    OceanSlopeVariance unresolvedSlopes;

    // 当前基线使用调用方提供的固定折射率。
    double etaIncident;
    double etaTransmitted;
};