#pragma once
#include "reflection/layered/CoatedDiffuseReflection.h"

// Explicit ACHROMATIC visible-light adapter. A NIR reflectance curve is NOT RGB.
// Both f and the random walk reuse the scalar model; no duplicate Fresnel/GGX.
class CoatedDiffuseBxDF final : public BxDF
{
public:
    CoatedDiffuseBxDF(double etaIncident,double etaCoating,float alphaX,float alphaY,
        double baseReflectance,double opticalThickness=0,LayeredReflectionOptions options={})
        : BxDF(static_cast<BxDFType>(BSDF_REFLECTION|BSDF_DIFFUSE|BSDF_GLOSSY)),
          model(etaIncident,etaCoating,alphaX,alphaY,opticalThickness,options),base(baseReflectance)
    { (void)LayeredReflectionDetail::DiffuseInterface(base); }
    bool NeedsComponentSample() const override { return true; } // Hidden internal R/T!
    Spectrum f(const Vector3f& wo,const Vector3f& wi,TransportMode) const override
    { return Coefficient(model.Evaluate(wo,wi,base)); }
    std::optional<BSDFSample> Sample_f(const Vector3f&,const Point2f&,
        TransportMode = TransportMode::Radiance,
        BxDFReflectionType flags = BxDFReflectionType::All) const override
    {
        if (!LayeredReflectionDetail::AllowsLayeredReflection(flags)) return {};
        throw std::invalid_argument("Coated diffuse random walk requires independent uc and u.");
    }
    std::optional<BSDFSample> Sample_f(const Vector3f& wo,float uc,const Point2f& u,
        TransportMode mode = TransportMode::Radiance,
        BxDFReflectionType flags = BxDFReflectionType::All) const override
    {
        const auto s=model.Sample_f(wo,uc,u,base,mode,flags);
        if (!s) return {};
        // Common scaling is allowed only because this is a proportional pair.
        const double scale=std::max(s->f,s->pdf);
        const float value=float(s->f/scale),p=float(s->pdf/scale);
        // A vanishing absorbed contribution can underflow RGB float storage;
        // terminate it just as the existing Spectrum-zero path does.
        if (value==0 && p>0) return {};
        if (!(value>0) || !(p>0) || !std::isfinite(value) || !std::isfinite(p))
            throw std::overflow_error("Layered path weight exceeds float adapter range.");
        return BSDFSample(Spectrum(value),s->wi,p,s->flags,1.0f,true);
    }
    float Pdf(const Vector3f& wo,const Vector3f& wi,TransportMode mode,
        BxDFReflectionType flags=BxDFReflectionType::All) const override
    { return ScalarFloat(model.Pdf(wo,wi,base,mode,flags)); }
    Spectrum rho(const Vector3f& wo,int n,const Point2f* samples) const override
    {
        if (n<=0 || !samples) return Spectrum(0.f);
        // rho's legacy API has only 2D samples: direction-space cosine
        // integration of f, NOT the layered Sample_f() random-walk algorithm.
        double sum=0;
        for (int i=0;i<n;++i)
        {
            LayeredReflectionDetail::RequireLayeredSample(samples[i].x);
            LayeredReflectionDetail::RequireLayeredSample(samples[i].y);
            const auto wi=BRDFUtils::SampleCosineHemisphere(samples[i]);
            sum+=std::numbers::pi_v<double>*model.Evaluate(wo,wi,base);
        }
        return Coefficient(sum/n);
    }
    const LayeredReflectionOptions& Options() const noexcept {return model.Options();}
private:
    static float ScalarFloat(double x)
    {
        LayeredReflectionDetail::Checked(x);
        if (x>std::numeric_limits<float>::max())
            throw std::overflow_error("Coated diffuse coefficient exceeds float range.");
        return float(x);
    }
    static Spectrum Coefficient(double x) {return Spectrum(ScalarFloat(x));}
    CoatedDiffuseReflection model;
    double base;
};
