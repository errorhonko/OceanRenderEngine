#pragma once

#include "reflection/lidar/LidarScatteringModel.h"
#include "core/spectrum/SpectralReflectanceCurve.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <utility>
#include <variant>

// Formula-based adapter, not a copied HELIOS++ source file. Reference:
// 3dgeo-heidelberg/helios, commit 2b3585aa766213ae9a46c4790ad4f3594ffa213d,
// src/scene/Material.cpp and src/maths/EnergyMaths.cpp (LGPL-3.0 project).
enum class HeliosEmpiricalKind { Phong, Lambert, DirectionIndependent };

class HeliosEmpiricalAngularResponse
{
public:
    HeliosEmpiricalAngularResponse(std::array<double, 3> kd,
        std::array<double, 3> ks, double exponent = 10)
        : exponent(exponent)
    {
        if (!std::isfinite(exponent) || exponent < 0)
            throw std::invalid_argument("HELIOS exponent must be finite and nonnegative.");
        double d = 0, s = 0;
        for (int c = 0; c < 3; ++c) {
            if (!std::isfinite(kd[c]) || !std::isfinite(ks[c]) ||
                kd[c] < 0 || kd[c] > 1 || ks[c] < 0 || ks[c] > 1)
                throw std::invalid_argument("HELIOS Kd/Ks components must be in [0,1].");
            d += kd[c]; s += ks[c];
        }
        if (d == 0 && s > 0)
            throw std::invalid_argument("HELIOS does not support Ks-only materials.");
        kind = d == 0 ? HeliosEmpiricalKind::DirectionIndependent :
            (s == 0 ? HeliosEmpiricalKind::Lambert : HeliosEmpiricalKind::Phong);
        // Initialize EVERY material here, including zero-sum and non-final MTL
        // entries. Never depend on end-of-file or successful spectral lookup.
        specularity = d + s > 0 ? s / (d + s) : 0;
    }

    double Evaluate(double cosIncident) const
    {
        if (!std::isfinite(cosIncident) || cosIncident < -1 || cosIncident > 1)
            throw std::invalid_argument("HELIOS incidence cosine is invalid.");
        if (cosIncident <= 0) return 0; // Front-surface reflection only.
        if (kind == HeliosEmpiricalKind::DirectionIndependent) return 1;
        if (kind == HeliosEmpiricalKind::Lambert) return cosIncident;
        const double cosDoubleAngle = std::abs(std::fma(2 * cosIncident, cosIncident, -1));
        const double lobe = exponent == 0 ? 1 :
            std::pow(std::min(1.0, cosDoubleAngle), exponent);
        // Stable algebraic form of computeBDRF; no divide by cos(theta).
        // The absolute cosine's second lobe is deliberately retained.
        return (1 - specularity) * cosIncident + specularity * lobe;
    }

    HeliosEmpiricalKind Kind() const noexcept { return kind; }
    double Specularity() const noexcept { return specularity; }
    double Exponent() const noexcept { return exponent; }

private:
    HeliosEmpiricalKind kind;
    double specularity = 0;
    double exponent;
};

// Monostatic, unpolarized empirical backscatter, NOT a general BRDF. We adapt
// its angular shape to our energy units: projected response = rho * gain / pi.
// Thus its Lambert branch matches the existing Lambert energy chain. This does
// NOT reproduce HELIOS's raw percentage/intensity scale or its whole simulator.
class HeliosEmpiricalLidarScattering final : public LidarScatteringModel
{
public:
    HeliosEmpiricalLidarScattering(double reflectance,
        HeliosEmpiricalAngularResponse angular)
        : reflectance(reflectance), angular(angular)
    {
        if (!std::isfinite(reflectance) || reflectance < 0 || reflectance > 1)
            throw std::invalid_argument("HELIOS normalized reflectance must be in [0,1].");
    }

    HeliosEmpiricalLidarScattering(SpectralReflectanceCurve curve,
        HeliosEmpiricalAngularResponse angular)
        : reflectance(std::move(curve)), angular(angular)
    {
        if (std::get<SpectralReflectanceCurve>(reflectance).SampleCount() < 2)
            throw std::invalid_argument("HELIOS spectral response requires a valid curve.");
    }

    double ReflectanceAtWavelength(double wavelengthNm) const
    {
        if (!std::isfinite(wavelengthNm) || wavelengthNm <= 0)
            throw std::invalid_argument("HELIOS wavelength must be finite positive nm.");
        if (const auto scalar = std::get_if<double>(&reflectance)) return *scalar;
        return std::get<SpectralReflectanceCurve>(reflectance).Evaluate(wavelengthNm);
    }
    const HeliosEmpiricalAngularResponse& AngularResponse() const noexcept { return angular; }

    double Evaluate(const HitRecord&, const Vector3f&, const Vector3f&, float) const override
    {
        throw std::logic_error("HELIOS empirical response is not a BRDF; use EvaluateProjectedResponse.");
    }

    double EvaluateProjectedResponse(const HitRecord& hit, const Vector3f& wi,
        const Vector3f& wo, float wavelengthNm) const override
    {
        const auto n = Unit(hit.geometricNormal);
        const auto i = Unit(wi), o = Unit(wo);
        double differenceSquared = 0, cosI = 0, cosO = 0;
        for (int c = 0; c < 3; ++c) {
            differenceSquared += (i[c] - o[c]) * (i[c] - o[c]);
            cosI += n[c] * i[c]; cosO += n[c] * o[c];
        }
        // Only normalization/float roundoff tolerance, not a bistatic model.
        if (differenceSquared > 1e-10)
            throw std::invalid_argument("HELIOS empirical baseline requires monostatic directions.");
        const double rho = ReflectanceAtWavelength(wavelengthNm);
        if (cosI <= 0 || cosO <= 0) return 0;
        return rho * angular.Evaluate(std::clamp(cosI, 0.0, 1.0)) /
            std::numbers::pi_v<double>;
    }

private:
    static std::array<double, 3> Unit(const Vector3f& v)
    {
        const double length = std::hypot(double(v.x), double(v.y), double(v.z));
        if (!std::isfinite(length) || length <= 0)
            throw std::invalid_argument("HELIOS direction/geometry normal is invalid.");
        return {v.x / length, v.y / length, v.z / length};
    }
    std::variant<double, SpectralReflectanceCurve> reflectance;
    HeliosEmpiricalAngularResponse angular;
};
