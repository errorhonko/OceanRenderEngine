#pragma once

#include "core/spectrum/SpectralOpticalConstants.h"
#include "reflection/lidar/LidarScatteringModel.h"
#include "reflection/microfacet/MicrofacetReflection.h"
#include "reflection/microfacet/TrowbridgeReitzDistribution.h"
#include <utility>
#include <variant>

// PBRT finite-roughness GGX conductor reflection: either one explicit wavelength
// or an owned spectral n/k table. Both modes use the SAME Fresnel/BRDF kernel.
// No RGB conversion, inferred optical constants, transmission or delta.
class RoughConductorLidarScattering final : public LidarScatteringModel
{
public:
    RoughConductorLidarScattering(double etaIncident, double etaConductor,
        double kappaConductor, float alphaX, float alphaY, float wavelengthNm)
        : etaIncident(etaIncident), alphaX(alphaX), alphaY(alphaY),
          opticalConstants(FixedConstants{etaConductor, kappaConductor, wavelengthNm}),
          distribution(alphaX, alphaY)
    {
        ValidateCommon();
        (void)FrConductor(1.0, etaIncident, etaConductor, kappaConductor);
        if (!std::isfinite(wavelengthNm) || wavelengthNm <= 0)
            throw std::invalid_argument("Conductor reference wavelength must be finite positive nm.");
    }

    // Copy lvalues / move rvalues; no dangling table reference or per-hit file I/O.
    RoughConductorLidarScattering(double etaIncident, SpectralOpticalConstants spectrum,
        float alphaX, float alphaY)
        : etaIncident(etaIncident), alphaX(alphaX), alphaY(alphaY),
          opticalConstants(std::move(spectrum)), distribution(alphaX, alphaY)
    {
        ValidateCommon();
        if (OpticalConstantsSpectrum()->SampleCount() < 2)
            throw std::invalid_argument("Spectral conductor requires a valid optical-constant table.");
    }

    static RoughConductorLidarScattering FromRoughness(double etaIncident,
        double etaConductor, double kappaConductor, float roughnessX, float roughnessY,
        float wavelengthNm)
    {
        ValidateRoughness(roughnessX, roughnessY);
        return {etaIncident, etaConductor, kappaConductor,
            TrowbridgeReitzDistribution::RoughnessToAlpha(roughnessX),
            TrowbridgeReitzDistribution::RoughnessToAlpha(roughnessY), wavelengthNm};
    }

    static RoughConductorLidarScattering FromRoughness(double etaIncident,
        SpectralOpticalConstants spectrum, float roughnessX, float roughnessY)
    {
        ValidateRoughness(roughnessX, roughnessY);
        return {etaIncident, std::move(spectrum),
            TrowbridgeReitzDistribution::RoughnessToAlpha(roughnessX),
            TrowbridgeReitzDistribution::RoughnessToAlpha(roughnessY)};
    }

    double Evaluate(const HitRecord& hit, const Vector3f& incidentDirection,
        const Vector3f& outgoingDirection, float queryWavelengthNm) const override
    {
        // Resolve once, before hemisphere exits; invalid/out-of-band input never
        // silently becomes a black BRDF. Do not add cosine/PDF/energy factors here.
        const auto nk = OpticalConstantsAtWavelength(queryWavelengthNm);
        const auto wi = MicrofacetReflectionDetail::NormalizeDirection(incidentDirection);
        const auto wo = MicrofacetReflectionDetail::NormalizeDirection(outgoingDirection);
        const auto ng = MicrofacetReflectionDetail::NormalizeDirection(hit.geometricNormal);
        if (wi.dot(ng) <= 0 || wo.dot(ng) <= 0) return 0;
        auto ns = hit.normal;
        if (!Finite(ns) || ns.near_zero()) ns = ng;
        ns = MicrofacetReflectionDetail::NormalizeDirection(ns);
        if (ns.dot(ng) < 0) ns = -ns;
        const auto frame = SurfaceFrame(hit.dpdu, ns);
        return EvaluateMicrofacetConductorReflection(distribution,
            frame.ToLocal(wo), frame.ToLocal(wi), etaIncident, nk.n, nk.k);
    }

    OpticalConstants OpticalConstantsAtWavelength(double queryWavelengthNm) const
    {
        if (!std::isfinite(queryWavelengthNm) || queryWavelengthNm <= 0)
            throw std::invalid_argument("Conductor query wavelength must be finite positive nm.");
        if (const auto* spectrum = OpticalConstantsSpectrum())
            return spectrum->Evaluate(queryWavelengthNm);
        const auto& fixed = FixedParameters();
        if (queryWavelengthNm != fixed.wavelengthNm)
            throw std::out_of_range("Conductor n/k are configured only for the reference wavelength.");
        return {fixed.n, fixed.k};
    }

    bool IsSpectral() const noexcept { return OpticalConstantsSpectrum() != nullptr; }
    const SpectralOpticalConstants* OpticalConstantsSpectrum() const noexcept
    {
        return std::get_if<SpectralOpticalConstants>(&opticalConstants);
    }
    double EtaIncident() const noexcept { return etaIncident; }
    // Legacy fixed-mode diagnostics: spectral values require an explicit wavelength.
    double EtaConductor() const { return FixedParameters().n; }
    double KappaConductor() const { return FixedParameters().k; }
    float AlphaX() const noexcept { return alphaX; }
    float AlphaY() const noexcept { return alphaY; }
    float WavelengthNm() const { return FixedParameters().wavelengthNm; }
private:
    struct FixedConstants { double n, k; float wavelengthNm; };
    const FixedConstants& FixedParameters() const
    {
        if (const auto* fixed = std::get_if<FixedConstants>(&opticalConstants)) return *fixed;
        throw std::logic_error("Spectral conductor has no single n/k or reference wavelength; query a wavelength explicitly.");
    }
    void ValidateCommon() const
    {
        if (!std::isfinite(etaIncident) || etaIncident <= 0)
            throw std::invalid_argument("Conductor incident-medium index must be finite positive.");
        if (!std::isfinite(alphaX) || !std::isfinite(alphaY) || alphaX < 1e-3f || alphaY < 1e-3f)
            throw std::invalid_argument("Conductor alpha must be at least 1e-3; delta mirrors need separate handling.");
    }
    static void ValidateRoughness(float roughnessX, float roughnessY)
    {
        if (!std::isfinite(roughnessX) || !std::isfinite(roughnessY) ||
            roughnessX < 1e-6f || roughnessY < 1e-6f || roughnessX > 1 || roughnessY > 1)
            throw std::invalid_argument("Conductor roughness must be in [1e-6,1].");
    }
    static bool Finite(const Vector3f& v)
    { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
    static Frame SurfaceFrame(Vector3f tangent, const Vector3f& normal)
    {
        if (!Finite(tangent) || tangent.near_zero()) return Frame::FromZ(normal);
        tangent = tangent - normal * tangent.dot(normal);
        if (!Finite(tangent) || tangent.near_zero()) return Frame::FromZ(normal);
        return Frame::FromXZ(MicrofacetReflectionDetail::NormalizeDirection(tangent), normal);
    }
    double etaIncident;
    float alphaX, alphaY;
    std::variant<FixedConstants, SpectralOpticalConstants> opticalConstants;
    TrowbridgeReitzDistribution distribution;
};
