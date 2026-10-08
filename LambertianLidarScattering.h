#pragma once

#include "LidarScatteringModel.h"

#include <cmath>
#include <numbers>
#include <stdexcept>

class LambertianLidarScattering final
    : public LidarScatteringModel
{
public:
    explicit LambertianLidarScattering(
        double reflectanceAtWavelength)
        : reflectanceAtWavelength(
            reflectanceAtWavelength)
    {
        if (!std::isfinite(
            reflectanceAtWavelength) ||
            reflectanceAtWavelength < 0.0 ||
            reflectanceAtWavelength > 1.0)
        {
            throw std::invalid_argument(
                "Lambertian reflectance must be in [0, 1].");
        }
    }

    double Evaluate(
        const HitRecord&,
        const Vector3f&,
        const Vector3f&,
        float) const override
    {
        return reflectanceAtWavelength /
            std::numbers::pi_v<double>;
    }

    double ReflectanceAtWavelength() const
    {
        return reflectanceAtWavelength;
    }

private:
    // 无量纲反射率 rho。
    double reflectanceAtWavelength = 0.0;
};