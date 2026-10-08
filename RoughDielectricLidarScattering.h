#pragma once

#include "BRDFUtils.h"
#include "MicrofacetReflection.h"
#include "LidarScatteringModel.h"
#include "TrowbridgeReitzDistribution.h"

#include <cmath>
#include <stdexcept>

class RoughDielectricLidarScattering final
    : public LidarScatteringModel
{
public:
    RoughDielectricLidarScattering(
        double etaIncident,
        double etaTransmitted,
        float alphaX,
        float alphaY)
        : etaIncident(etaIncident),
        etaTransmitted(etaTransmitted),
        alphaX(alphaX),
        alphaY(alphaY),
        distribution(alphaX, alphaY)
    {
        if (!std::isfinite(etaIncident) ||
            !std::isfinite(etaTransmitted) ||
            etaIncident <= 0.0 ||
            etaTransmitted <= 0.0)
        {
            throw std::invalid_argument(
                "Dielectric indices must be positive.");
        }

        // 当前 Evaluate 接口无法表示完全光滑的 Delta 镜面。
        if (!std::isfinite(alphaX) ||
            !std::isfinite(alphaY) ||
            alphaX < 1.0e-3f ||
            alphaY < 1.0e-3f)
        {
            throw std::invalid_argument(
                "Microfacet roughness must be at least 1e-3.");
        }
    }

    static RoughDielectricLidarScattering FromRoughness(
        double etaIncident,
        double etaTransmitted,
        float roughnessX,
        float roughnessY)
    {
        constexpr float minimumRoughness = 1.0e-6f;

        if (!std::isfinite(roughnessX) ||
            !std::isfinite(roughnessY) ||
            roughnessX < minimumRoughness ||
            roughnessY < minimumRoughness ||
            roughnessX > 1.0f ||
            roughnessY > 1.0f)
        {
            throw std::invalid_argument(
                "Microfacet roughness must be in [1e-6, 1].");
        }

        const float alphaX =
            TrowbridgeReitzDistribution::
            RoughnessToAlpha(roughnessX);

        const float alphaY =
            TrowbridgeReitzDistribution::
            RoughnessToAlpha(roughnessY);

        return RoughDielectricLidarScattering(
            etaIncident,
            etaTransmitted,
            alphaX,
            alphaY);
    }

    float AlphaX() const
    {
        return alphaX;
    }

    float AlphaY() const
    {
        return alphaY;
    }
    double Evaluate(
        const HitRecord& hit,
        const Vector3f& incidentDirection,
        const Vector3f& outgoingDirection,
        float) const override
    {
        if (!IsFinite(incidentDirection) ||
            !IsFinite(outgoingDirection) ||
            incidentDirection.near_zero() ||
            outgoingDirection.near_zero())
        {
            throw std::invalid_argument(
                "Scattering directions are invalid.");
        }

        Vector3f surfaceNormal = hit.normal;

        // 没有着色法线时退回几何法线。
        if (!IsFinite(surfaceNormal) ||
            surfaceNormal.near_zero())
        {
            surfaceNormal = hit.geometricNormal;
        }

        if (!IsFinite(surfaceNormal) ||
            surfaceNormal.near_zero())
        {
            throw std::invalid_argument(
                "Scattering normal is invalid.");
        }

        // 着色法线必须与几何法线位于同一半球。
        if (IsFinite(hit.geometricNormal) &&
            !hit.geometricNormal.near_zero() &&
            surfaceNormal.dot(
                hit.geometricNormal) < 0.0f)
        {
            surfaceNormal = -surfaceNormal;
        }

        const Frame frame =
            BuildSurfaceFrame(
                hit,
                surfaceNormal);

        const Vector3f wi =
            frame.ToLocal(
                incidentDirection.normalize());

        const Vector3f wo =
            frame.ToLocal(
                outgoingDirection.normalize());

        // Shared local-space BRDF; receiver geometry and energy stay outside.
        return EvaluateMicrofacetDielectricReflection(
            distribution,
            wo,
            wi,
            etaIncident,
            etaTransmitted);
    }

    static RoughDielectricLidarScattering FromRmsSlope(
        double etaIncident,
        double etaTransmitted,
        float rmsSlopeX,
        float rmsSlopeY)
    {
        if (!std::isfinite(rmsSlopeX) ||
            !std::isfinite(rmsSlopeY) ||
            rmsSlopeX < 0.0f ||
            rmsSlopeY < 0.0f)
        {
            throw std::invalid_argument(
                "RMS slopes must be finite and non-negative.");
        }

        constexpr float sqrtTwo =
            1.4142135623730951f;

        // TODO: alpha = sqrt(2) * rmsSlope 是 Beckmann 的参数关系，
        // 不是 GGX 的精确 RMS 坡度转换。后续删除此工厂或明确改名为
        // 经验映射；当前暂时保留计算逻辑。此提醒同时适用于 X/Y 两轴。
        const float convertedAlphaX =
            sqrtTwo * rmsSlopeX;

        const float convertedAlphaY =
            sqrtTwo * rmsSlopeY;

        // 当前 Evaluate() 不能表示完全光滑的 Delta 反射。
        if (convertedAlphaX < 1.0e-3f ||
            convertedAlphaY < 1.0e-3f)
        {
            throw std::invalid_argument(
                "RMS slopes are too small for the rough model.");
        }

        return RoughDielectricLidarScattering(
            etaIncident,
            etaTransmitted,
            convertedAlphaX,
            convertedAlphaY);
    }

private:
    static Frame BuildSurfaceFrame(
        const HitRecord& hit,
        const Vector3f& surfaceNormal)
    {
        const Vector3f normal =
            surfaceNormal.normalize();

        Vector3f tangent = hit.dpdu;

        // dpdu 不可用时退回自动构造的坐标系。
        if (!IsFinite(tangent) ||
            tangent.near_zero())
        {
            return Frame::FromZ(normal);
        }

        // 将 dpdu 投影到表面切平面，防止它包含法线分量。
        tangent =
            tangent -
            normal * tangent.dot(normal);

        if (!IsFinite(tangent) ||
            tangent.near_zero())
        {
            return Frame::FromZ(normal);
        }

        return Frame::FromXZ(
            tangent.normalize(),
            normal);
    }

    static bool IsFinite(
        const Vector3f& value)
    {
        return
            std::isfinite(value.x) &&
            std::isfinite(value.y) &&
            std::isfinite(value.z);
    }

    double etaIncident = 1.0;
    double etaTransmitted = 1.0;

    float alphaX = 0.0f;
    float alphaY = 0.0f;

    TrowbridgeReitzDistribution distribution;
};
