#pragma once

#include "reflection/lidar/LidarScatteringModel.h"
#include "reflection/microfacet/MicrofacetReflection.h"
#include "reflection/microfacet/TrowbridgeReitzDistribution.h"

// PBRT finite-roughness GGX conductor reflection, at one explicitly configured
// wavelength. No RGB conversion, inferred optical constants, transmission or delta.
class RoughConductorLidarScattering final : public LidarScatteringModel
{
public:
    RoughConductorLidarScattering(double etaIncident, double etaConductor,
        double kappaConductor, float alphaX, float alphaY, float wavelengthNm)
        : etaIncident(etaIncident), etaConductor(etaConductor), kappaConductor(kappaConductor),
          alphaX(alphaX), alphaY(alphaY), wavelengthNm(wavelengthNm), distribution(alphaX, alphaY)
    {
        (void)FrConductor(1.0, etaIncident, etaConductor, kappaConductor);
        if (!std::isfinite(wavelengthNm) || wavelengthNm <= 0)
            throw std::invalid_argument("Conductor reference wavelength must be finite positive nm.");
        if (!std::isfinite(alphaX) || !std::isfinite(alphaY) || alphaX < 1e-3f || alphaY < 1e-3f)
            throw std::invalid_argument("Conductor alpha must be at least 1e-3; delta mirrors need separate handling.");
    }

    static RoughConductorLidarScattering FromRoughness(double etaIncident,
        double etaConductor, double kappaConductor, float roughnessX, float roughnessY,
        float wavelengthNm)
    {
        if (!std::isfinite(roughnessX) || !std::isfinite(roughnessY) ||
            roughnessX < 1e-6f || roughnessY < 1e-6f || roughnessX > 1 || roughnessY > 1)
            throw std::invalid_argument("Conductor roughness must be in [1e-6,1].");
        return {etaIncident, etaConductor, kappaConductor,
            TrowbridgeReitzDistribution::RoughnessToAlpha(roughnessX),
            TrowbridgeReitzDistribution::RoughnessToAlpha(roughnessY), wavelengthNm};
    }

    double Evaluate(const HitRecord& hit, const Vector3f& incidentDirection,
        const Vector3f& outgoingDirection, float queryWavelengthNm) const override
    {
        if (!std::isfinite(queryWavelengthNm) || queryWavelengthNm <= 0)
            throw std::invalid_argument("Conductor query wavelength must be finite positive nm.");
        if (queryWavelengthNm != wavelengthNm)
            throw std::out_of_range("Conductor n/k are configured only for the reference wavelength.");
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
            frame.ToLocal(wo), frame.ToLocal(wi), etaIncident, etaConductor, kappaConductor);
    }

    double EtaIncident() const noexcept { return etaIncident; }
    double EtaConductor() const noexcept { return etaConductor; }
    double KappaConductor() const noexcept { return kappaConductor; }
    float AlphaX() const noexcept { return alphaX; }
    float AlphaY() const noexcept { return alphaY; }
    float WavelengthNm() const noexcept { return wavelengthNm; }
private:
    static bool Finite(const Vector3f& v)
    { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
    static Frame SurfaceFrame(Vector3f tangent, const Vector3f& normal)
    {
        if (!Finite(tangent) || tangent.near_zero()) return Frame::FromZ(normal);
        tangent = tangent - normal * tangent.dot(normal);
        if (!Finite(tangent) || tangent.near_zero()) return Frame::FromZ(normal);
        return Frame::FromXZ(MicrofacetReflectionDetail::NormalizeDirection(tangent), normal);
    }
    double etaIncident, etaConductor, kappaConductor;
    float alphaX, alphaY, wavelengthNm;
    TrowbridgeReitzDistribution distribution;
};
