#pragma once

#include "reflection/lidar/LidarScatteringModel.h"
#include "reflection/layered/CoatedDiffuseReflection.h"
#include "core/spectrum/SpectralReflectanceCurve.h"
#include <variant>

class CoatedDiffuseLidarScattering final : public LidarScatteringModel
{
public:
    CoatedDiffuseLidarScattering(double etaIncident, double etaCoating, float alphaX, float alphaY,
        double baseReflectance, double opticalThickness = 0.0, LayeredReflectionOptions options = {})
        : reflection(etaIncident, etaCoating, alphaX, alphaY, opticalThickness, options), base(baseReflectance)
    { (void)LayeredReflectionDetail::DiffuseInterface(baseReflectance); }

    CoatedDiffuseLidarScattering(double etaIncident, double etaCoating, float alphaX, float alphaY,
        SpectralReflectanceCurve baseReflectance, double opticalThickness = 0.0, LayeredReflectionOptions options = {})
        : reflection(etaIncident, etaCoating, alphaX, alphaY, opticalThickness, options), base(std::move(baseReflectance))
    {
        if (std::get<SpectralReflectanceCurve>(base).SampleCount() < 2)
            throw std::invalid_argument("Coated diffuse requires a valid BASE reflectance curve.");
    }
    double BaseReflectanceAtWavelength(double wavelengthNm) const
    {
        if (!std::isfinite(wavelengthNm) || wavelengthNm <= 0)
            throw std::invalid_argument("Coated diffuse wavelength must be finite positive nm.");
        if (const auto* curve = std::get_if<SpectralReflectanceCurve>(&base)) return curve->Evaluate(wavelengthNm);
        return std::get<double>(base); // Explicit achromatic base assumption.
    }
    double Evaluate(const HitRecord& hit, const Vector3f& incidentDirection,
        const Vector3f& outgoingDirection, float wavelengthNm) const override
    {
        const double rho = BaseReflectanceAtWavelength(wavelengthNm);
        const auto wi = MicrofacetReflectionDetail::NormalizeDirection(incidentDirection);
        const auto wo = MicrofacetReflectionDetail::NormalizeDirection(outgoingDirection);
        const auto ng = MicrofacetReflectionDetail::NormalizeDirection(hit.geometricNormal);
        if (wi.dot(ng) <= 0 || wo.dot(ng) <= 0) return 0;
        auto ns = hit.normal;
        if (!Finite(ns) || ns.near_zero()) ns = ng;
        ns = MicrofacetReflectionDetail::NormalizeDirection(ns);
        if (ns.dot(ng) < 0) ns = -ns;
        const auto frame = SurfaceFrame(hit.dpdu, ns);
        return reflection.Evaluate(frame.ToLocal(wo), frame.ToLocal(wi), rho);
    }
    const LayeredReflectionOptions& EvaluationOptions() const noexcept { return reflection.Options(); }
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
    CoatedDiffuseReflection reflection;
    std::variant<double, SpectralReflectanceCurve> base;
};
