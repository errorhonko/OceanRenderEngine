#pragma once

#include "core/ray/HitRocord.h"
#include "core/math/Vector3f.h"

#include <cmath>
#include <stdexcept>

class LidarScatteringModel
{
public:
    virtual ~LidarScatteringModel() = default;

    // incidentDirection：表面指向发射器
    // outgoingDirection：表面指向接收器
    // 返回激光波长处的 BRDF，单位 sr^-1。
    virtual double Evaluate(
        const HitRecord& hit,
        const Vector3f& incidentDirection,
        const Vector3f& outgoingDirection,
        float wavelengthNm) const = 0;

    // Projected response per steradian used by the return estimator. Physical
    // models retain f_r * cos(theta_return). Empirical backscatter adapters may
    // override this entry point WITHOUT pretending their angular gain is a BRDF.
    virtual double EvaluateProjectedResponse(
        const HitRecord& hit, const Vector3f& wi, const Vector3f& wo,
        float wavelengthNm) const
    {
        const double f = Evaluate(hit, wi, wo, wavelengthNm);
        if (!std::isfinite(f) || f < 0)
            throw std::domain_error("LiDAR scattering model returned an invalid BRDF.");
        return f * hit.geometricNormal.normalize().dot(wo);
    }
};

class ConstantLidarScattering final
    : public LidarScatteringModel
{
public:
    explicit ConstantLidarScattering(
        double brdfAtWavelengthPerSr)
        : brdfAtWavelengthPerSr(
            brdfAtWavelengthPerSr)
    {
        if (!std::isfinite(
            brdfAtWavelengthPerSr) ||
            brdfAtWavelengthPerSr < 0.0)
        {
            throw std::invalid_argument(
                "LiDAR BRDF must be finite and nonnegative.");
        }
    }

    double Evaluate(
        const HitRecord&,
        const Vector3f&,
        const Vector3f&,
        float) const override
    {
        return brdfAtWavelengthPerSr;
    }

    double BrdfAtWavelengthPerSr() const
    {
        return brdfAtWavelengthPerSr;
    }

private:
    double brdfAtWavelengthPerSr = 0.0;
};
