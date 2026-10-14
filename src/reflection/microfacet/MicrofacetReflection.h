#pragma once

#include "reflection/fresnel/FresnelUtils.h"
#include "core/math/Vector3f.h"

#include <algorithm>
#include <cmath>
#include <concepts>
#include <stdexcept>
#include <type_traits>

namespace MicrofacetReflectionDetail
{
inline Vector3f NormalizeDirection(const Vector3f& w)
{
    const double length = std::hypot(double(w.x), double(w.y), double(w.z));
    if (!std::isfinite(length) || length == 0.0)
        throw std::invalid_argument("Microfacet direction must be finite and nonzero.");

    return Vector3f(
        float(double(w.x) / length),
        float(double(w.y) / length),
        float(double(w.z) / length));
}
}

// Finite-roughness, one-sided dielectric reflection in a local +Z frame.
// Both directions point away from the surface and are normalized internally.
// Distribution must supply a compatible, finite NDF D(m) and Smith G(wo,wi).
// etaIncident/etaTransmitted are indices at the wavelength being evaluated.
// Returns scalar BRDF in sr^-1; no external cosine, visibility, PDF or energy.
// Not a delta mirror, transmission term, spectral container or BSDF sampler.
template <typename Distribution>
requires requires(const Distribution& d, const Vector3f& w)
{
    { d.D(w) } -> std::convertible_to<double>;
    { d.G(w, w) } -> std::convertible_to<double>;
}
inline double EvaluateMicrofacetDielectricReflection(
    const Distribution& distribution,
    const Vector3f& woLocal,
    const Vector3f& wiLocal,
    double etaIncident,
    double etaTransmitted)
{
    if (!std::isfinite(etaIncident) || !std::isfinite(etaTransmitted) ||
        etaIncident <= 0.0 || etaTransmitted <= 0.0)
        throw std::invalid_argument("Dielectric indices must be finite and positive.");

    const Vector3f wo = MicrofacetReflectionDetail::NormalizeDirection(woLocal);
    const Vector3f wi = MicrofacetReflectionDetail::NormalizeDirection(wiLocal);

    if (wo.z <= 0.0f || wi.z <= 0.0f)
        return 0.0;

    if (etaIncident == etaTransmitted)
        return 0.0;

    const Vector3f wm = MicrofacetReflectionDetail::NormalizeDirection(wo + wi);
    const double cosMicrofacet = std::clamp(
        double(wo.x) * wm.x + double(wo.y) * wm.y + double(wo.z) * wm.z,
        0.0, 1.0);
    const double fresnel = FrDielectric(
        cosMicrofacet, etaIncident, etaTransmitted);
    const double normalDistribution = static_cast<double>(distribution.D(wm));
    const double maskingShadowing = static_cast<double>(distribution.G(wo, wi));

    if (!std::isfinite(fresnel) || fresnel < 0.0 || fresnel > 1.0 ||
        !std::isfinite(normalDistribution) || normalDistribution < 0.0 ||
        !std::isfinite(maskingShadowing) || maskingShadowing < 0.0 ||
        maskingShadowing > 1.0)
        throw std::domain_error("Microfacet model returned invalid F, D or G.");

    const double denominator = 4.0 * double(wi.z) * double(wo.z);
    const double brdf = fresnel * normalDistribution * maskingShadowing / denominator;
    if (!std::isfinite(brdf))
        throw std::overflow_error("Microfacet reflection BRDF overflow.");

    // A BRDF value can exceed 1; energy conservation concerns its integral.
    return brdf;
}

// One PBRT conductor BRDF kernel for both Spectrum and scalar wavelength adapters.
// Fresnel evaluator supplies validated F; D/G/half-vector/projection are computed
// once, even for multiple channels. Local +Z, finite roughness, pure f_r [sr^-1].
template <typename Distribution, typename FresnelEvaluator>
requires requires(const Distribution& d, const Vector3f& w)
{
    { d.D(w) } -> std::convertible_to<double>;
    { d.G(w, w) } -> std::convertible_to<double>;
}
inline auto EvaluateMicrofacetConductorReflection(const Distribution& distribution,
    const Vector3f& woLocal, const Vector3f& wiLocal, FresnelEvaluator fresnelAtCosine)
    -> decltype(fresnelAtCosine(0.0))
{
    using Value = decltype(fresnelAtCosine(0.0));
    const auto wo = MicrofacetReflectionDetail::NormalizeDirection(woLocal);
    const auto wi = MicrofacetReflectionDetail::NormalizeDirection(wiLocal);
    if (wo.z <= 0 || wi.z <= 0) return Value(0);
    const auto wm = MicrofacetReflectionDetail::NormalizeDirection(wo + wi);
    const double cosMicrofacet = std::clamp(
        double(wo.x)*wm.x + double(wo.y)*wm.y + double(wo.z)*wm.z, 0.0, 1.0);
    const Value fresnel = fresnelAtCosine(cosMicrofacet);
    if constexpr (std::is_floating_point_v<Value>)
        if (fresnel == 0) return Value(0);
    const double d = static_cast<double>(distribution.D(wm));
    const double g = static_cast<double>(distribution.G(wo, wi));
    if (!std::isfinite(d) || d < 0 || !std::isfinite(g) || g < 0 || g > 1)
        throw std::domain_error("Conductor microfacet model returned invalid D or G.");
    const double scale = d * g / (4.0 * double(wi.z) * double(wo.z));
    if (!std::isfinite(scale)) throw std::overflow_error("Conductor microfacet projection overflow.");
    const Value brdf = fresnel * scale;
    if constexpr (std::is_floating_point_v<Value>)
    {
        if (!std::isfinite(brdf)) throw std::overflow_error("Conductor microfacet BRDF overflow.");
    }
    else
    {
        // Spectrum stores float coefficients; also detect narrowing overflow.
        for (const auto coefficient : brdf.c)
            if (!std::isfinite(coefficient)) throw std::overflow_error("Conductor spectrum BRDF overflow.");
    }
    return brdf; // A finite BRDF may exceed 1; its energy integral must not.
}

// Fixed-wavelength adapter. RGB callers supply FrComplex through the overload above;
// no conversion between RGB channels and physical laser wavelengths occurs.
template <typename Distribution>
requires requires(const Distribution& d, const Vector3f& w)
{
    { d.D(w) } -> std::convertible_to<double>;
    { d.G(w, w) } -> std::convertible_to<double>;
}
inline double EvaluateMicrofacetConductorReflection(const Distribution& distribution,
    const Vector3f& woLocal, const Vector3f& wiLocal,
    double etaIncident, double etaConductor, double kappaConductor)
{
    if (!std::isfinite(etaIncident) || etaIncident <= 0 ||
        !std::isfinite(etaConductor) || etaConductor < 0 ||
        !std::isfinite(kappaConductor) || kappaConductor < 0 ||
        (etaConductor == 0 && kappaConductor == 0))
        throw std::invalid_argument("Conductor microfacet indices are invalid.");
    return EvaluateMicrofacetConductorReflection(distribution, woLocal, wiLocal,
        [=](double cosine) { return FrConductor(cosine, etaIncident, etaConductor, kappaConductor); });
}
