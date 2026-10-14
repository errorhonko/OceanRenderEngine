#pragma once

#include "reflection/lidar/LidarScatteringModel.h"
#include "core/spectrum/SpectralReflectanceCurve.h"

#include <numbers>
#include <stdexcept>
#include <utility>

// Lambertian baseline: measured spectral rho is interpreted as the diffuse
// hemispherical reflectance, so f_r(lambda) = rho(lambda) / pi [sr^-1].
// A spectral curve alone does NOT recover the measured material's angular BRDF.
class SpectralLambertianLidarScattering final : public LidarScatteringModel
{
public:
    // Own the curve; copies lvalues and moves rvalues. No file access per hit.
    explicit SpectralLambertianLidarScattering(SpectralReflectanceCurve curve)
        : curve(std::move(curve))
    {
        if (this->curve.SampleCount() < 2)
            throw std::invalid_argument("Spectral Lambert scattering requires a valid reflectance curve.");
    }

    double Evaluate(
        const HitRecord&,
        const Vector3f&,
        const Vector3f&,
        float wavelengthNm) const override
    {
        // Like the fixed-rho Lambert model, angular hemisphere checks are left
        // to EstimateSingleReturn. Do NOT multiply by another cosine here.
        // Invalid/out-of-band queries retain the curve's explicit exceptions.
        return ReflectanceAtWavelength(wavelengthNm) / std::numbers::pi_v<double>;
    }

    double ReflectanceAtWavelength(double wavelengthNm) const
    {
        return curve.Evaluate(wavelengthNm);
    }

    const SpectralReflectanceCurve& ReflectanceCurve() const noexcept
    {
        return curve;
    }

private:
    SpectralReflectanceCurve curve;
};
