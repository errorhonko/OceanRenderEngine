#pragma once

#include "reflection/layered/LayeredReflectionSampling.h"
#include "reflection/bxdfs/DielectricBxDF.h"
#include "reflection/bxdfs/DiffuseBxDF.h"
#include <optional>

namespace LayeredReflectionDetail
{
    inline void RequireConditionalEvent(BxDFReflectionType flags)
    {
        if (flags != BxDFReflectionType::Reflection && flags != BxDFReflectionType::Transmission)
            throw std::invalid_argument("Layered interfaces require exactly Reflection or Transmission, not a mixed event.");
    }
    inline void RequireUnitDirection(const Vector3f& w)
    {
        const double lengthSquared = double(w.x)*w.x + double(w.y)*w.y + double(w.z)*w.z;
        if (!std::isfinite(lengthSquared) || std::abs(lengthSquared - 1.0) > 1e-3)
            throw std::invalid_argument("Layered interface directions must be finite normalized local directions.");
    }
    inline double NeutralCoefficient(const Spectrum& value)
    {
        // Achromatic interface/basis coefficients only, NOT RGB -> NIR inference.
        for (float coefficient : value.c)
            if (coefficient != value.c[0])
                throw std::logic_error("Scalar layered adapters require an achromatic interface or unit basis.");
        return Checked(value.c[0]);
    }
    class DielectricInterface
    {
    public:
        DielectricInterface(float relativeEta, const TrowbridgeReitzDistribution& distribution)
            : bxdf(relativeEta, distribution), delta(relativeEta == 1.0f || distribution.EffectivelySmooth()), eta(relativeEta) {}
        double f(const Vector3f& wo, const Vector3f& wi, TransportMode mode) const
        { return NeutralCoefficient(bxdf.f(wo, wi, mode)); }
        bool IsDelta() const noexcept { return delta; }
        double RelativeEta() const noexcept { return eta; }
        // All-event entry/exit sampling is separate from the conditional API.
        std::optional<LayeredInterfaceSample> SampleAll(const Vector3f& wo, float uc, Point2f u,
            TransportMode mode) const
        {
            RequireUnitDirection(wo);
            if (wo.z==0) return {};
            const auto s=bxdf.Sample_f(wo,uc,u,mode,BxDFReflectionType::All);
            if (!s) return {};
            return LayeredInterfaceSample{NeutralCoefficient(s->f),s->wi,s->pdf};
        }
        double PdfAll(const Vector3f& wo,const Vector3f& wi,TransportMode mode) const
        {
            RequireUnitDirection(wo); RequireUnitDirection(wi);
            if (wo.z==0 || wi.z==0) return 0;
            return Checked(bxdf.Pdf(wo,wi,mode,BxDFReflectionType::All));
        }
        // Density with respect to dOmega(wi), conditional on the requested
        // event, NOT on success. Its valid-direction mass may be less than 1.
        // Delta events have zero continuous density; sample.pdf is their mass.
        double Pdf(const Vector3f& wo, const Vector3f& wi,
            TransportMode mode, BxDFReflectionType flags) const
        {
            RequireConditionalEvent(flags);
            RequireUnitDirection(wo); RequireUnitDirection(wi);
            if (wo.z == 0 || wi.z == 0) return 0;
            return Checked(bxdf.Pdf(wo, wi, mode, flags));
        }
        std::optional<LayeredInterfaceSample> Sample_f(const Vector3f& wo, Point2f u,
            TransportMode mode, BxDFReflectionType flags) const
        {
            RequireConditionalEvent(flags); RequireUnitDirection(wo);
            if (wo.z == 0) return {};
            // Forced flags reuse the existing conditional VNDF branches, not its
            // legacy 2D reflection/transmission mixture. No new Fresnel formulas.
            const auto s = bxdf.Sample_f(wo, u, mode, flags);
            if (!s) return {};
            return LayeredInterfaceSample{NeutralCoefficient(s->f), s->wi, s->pdf};
        }
    private:
        DielectricBxDF bxdf;
        bool delta;
        float eta;
    };
    class DiffuseInterface
    {
    public:
        explicit DiffuseInterface(double reflectance) : rho(reflectance), unit(Spectrum(1.0f))
        {
            if (!std::isfinite(rho) || rho < 0 || rho > 1)
                throw std::invalid_argument("Coated diffuse base reflectance must be in [0,1].");
        }
        double f(const Vector3f& wo, const Vector3f& wi, TransportMode mode) const
        { return rho * NeutralCoefficient(unit.f(wo, wi, mode)); }
        bool IsDelta() const noexcept { return false; }
        double Reflectance() const noexcept { return rho; }
        // Reflectance changes f, not the cosine-direction sampling density.
        double Pdf(const Vector3f& wo, const Vector3f& wi,
            TransportMode mode, BxDFReflectionType flags) const
        {
            RequireConditionalEvent(flags);
            RequireUnitDirection(wo); RequireUnitDirection(wi);
            if (wo.z == 0 || wi.z == 0) return 0;
            return Checked(unit.Pdf(wo, wi, mode, flags));
        }
        std::optional<LayeredInterfaceSample> Sample_f(const Vector3f& wo, Point2f u,
            TransportMode mode, BxDFReflectionType flags) const
        {
            RequireConditionalEvent(flags); RequireUnitDirection(wo);
            if (wo.z == 0) return {};
            const auto s = unit.Sample_f(wo, u, mode, flags);
            if (!s) return {};
            return LayeredInterfaceSample{rho * NeutralCoefficient(s->f), s->wi, s->pdf};
        }
    private:
        double rho; // Wavelength-resolved scalar, kept in double, never RGB data.
        DiffuseBxDF unit;
    };
}

// Local scalar BRDF. Reuses existing dielectric and diffuse sampling/math.
// Finite roughness only; no Delta reflection, volume scattering or internal TOF.
class CoatedDiffuseReflection
{
public:
    CoatedDiffuseReflection(double etaIncident, double etaCoating, float alphaX, float alphaY,
        double opticalThickness = 0.0, LayeredReflectionOptions options = {})
        : top(RelativeEta(etaIncident, etaCoating), Distribution(alphaX, alphaY)),
          opticalThickness(opticalThickness), options(options)
    {
        this->options.Validate();
        if (!std::isfinite(opticalThickness) || opticalThickness < 0)
            throw std::invalid_argument("Layer optical thickness must be finite nonnegative.");
    }
    LayeredReflectionEstimate Estimate(Vector3f wo, Vector3f wi, double baseReflectance) const
    {
        const LayeredReflectionDetail::DiffuseInterface bottom(baseReflectance);
        return EvaluateScalarLayeredReflectionMonteCarlo(top, bottom, wo, wi, opticalThickness, options);
    }
    double Evaluate(Vector3f wo, Vector3f wi, double baseReflectance) const
    { return Estimate(wo, wi, baseReflectance).Brdf(); }
    std::optional<LayeredPathSample> Sample_f(Vector3f wo,float uc,Point2f u,double baseReflectance,
        TransportMode mode = TransportMode::Radiance,
        BxDFReflectionType flags = BxDFReflectionType::All) const
    {
        const LayeredReflectionDetail::DiffuseInterface bottom(baseReflectance);
        return SampleScalarLayeredReflection(top,bottom,wo,uc,u,opticalThickness,options,mode,flags);
    }
    // Approximate external MIS density; never divide Evaluate() by this value
    // as a substitute for the sampled path's proportional f/pdf pair.
    double Pdf(Vector3f wo,Vector3f wi,double baseReflectance,
        TransportMode mode = TransportMode::Radiance,
        BxDFReflectionType flags = BxDFReflectionType::All) const
    {
        const LayeredReflectionDetail::DiffuseInterface bottom(baseReflectance);
        return EstimateScalarLayeredPdfForMIS(top,bottom,wo,wi,options,mode,flags);
    }
    const LayeredReflectionOptions& Options() const noexcept { return options; }
private:
    static float RelativeEta(double outside, double inside)
    {
        if (!std::isfinite(outside) || outside <= 0 || !std::isfinite(inside) || inside <= 0)
            throw std::invalid_argument("Coated diffuse indices must be finite positive.");
        const float ratio = float(inside / outside);
        if (!std::isfinite(ratio) || ratio <= 0)
            throw std::overflow_error("Coated diffuse relative index is outside the interface numerical range.");
        return ratio;
    }
    static TrowbridgeReitzDistribution Distribution(float x, float y)
    {
        if (!std::isfinite(x) || !std::isfinite(y) || x < 1e-3f || y < 1e-3f)
            throw std::invalid_argument("Coated diffuse alpha must be finite and at least 1e-3; Delta requires separate handling.");
        return {x, y};
    }
    LayeredReflectionDetail::DielectricInterface top;
    double opticalThickness;
    LayeredReflectionOptions options;
};
