#pragma once
#include "reflection/bxdfs/BxDF.h"
#include "reflection/microfacet/TrowbridgeReitzDistribution.h"

class DielectricBxDF : public BxDF
{
    using Direction = DielectricDetail::Direction;
    struct Evaluation { double f = 0, pdf = 0; };
public:
    DielectricBxDF(float eta, const TrowbridgeReitzDistribution& distrib)
        : BxDF(static_cast<BxDFType>(((eta == 1) ? 0 : BSDF_REFLECTION) |
            BSDF_TRANSMISSION | (distrib.EffectivelySmooth() ? BSDF_SPECULAR : BSDF_GLOSSY))),
          eta(eta), mfDistrib(distrib) {}

    std::optional<BSDFSample> Sample_f(const Vector3f& wo, const Point2f& u,
        TransportMode mode = TransportMode::Radiance,
        BxDFReflectionType sampleFlags = BxDFReflectionType::All) const override
    {
        // Two dimensions suffice for a forced event or a smooth interface.
        // Never silently derive an allegedly independent event from the VNDF u.
        if (sampleFlags == BxDFReflectionType::All && eta != 1 && !mfDistrib.EffectivelySmooth())
            throw std::invalid_argument("Rough dielectric All requires Sample_f(wo, uc, u, mode, flags).");
        return Sample_f(wo, u.x, u, mode, sampleFlags);
    }

    std::optional<BSDFSample> Sample_f(const Vector3f& wo, float uc, const Point2f& u,
        TransportMode mode = TransportMode::Radiance,
        BxDFReflectionType sampleFlags = BxDFReflectionType::All) const override
    {
        using namespace DielectricDetail;
        const Direction o = Promote(wo);
        if (o.z == 0) return {};
        const auto macro = EvaluateInterface(o.z, 1.0, double(eta));
        double pr = macro.R, pt = macro.T;
        Mask(sampleFlags, pr, pt);
        if (eta == 1 || mfDistrib.EffectivelySmooth())
        {
            if (pr + pt == 0) return {};
            if (uc < pr / (pr + pt))
            {
                const Direction i{-o.x, -o.y, o.z};
                return BSDFSample(Spectrum(float(macro.R / std::abs(i.z))), Store(i),
                    float(pr / (pr + pt)), BxDFType::SpecularReflection);
            }
            Direction i;
            if (!RefractUnit(o, {0, 0, 1}, macro, i) || i.z == 0) return {};
            double value = macro.T / std::abs(i.z);
            if (mode == TransportMode::Radiance) value /= macro.eta * macro.eta;
            return BSDFSample(Spectrum(float(value)), Store(i), float(pt / (pr + pt)),
                BxDFType::SpecularTransmission, float(macro.eta));
        }

        // First sample the visible normal; THEN select the actual facet event.
        // A macro-normal TIR must not eliminate tilted transmitting facets.
        const Direction m = Promote(mfDistrib.Sample_Wm(wo, u));
        const auto optics = EvaluateInterface(Dot(o, m), 1.0, double(eta));
        pr = optics.R; pt = optics.T;
        Mask(sampleFlags, pr, pt);
        if (pr + pt == 0) return {};
        const bool reflect = uc < pr / (pr + pt);
        Direction i;
        if (reflect) i = Unit(Combine(o, -1, m, 2 * Dot(o, m)));
        else if (!RefractUnit(o, m, optics, i)) return {};
        if (i.z == 0 || (o.z * i.z > 0) != reflect) return {};
        const auto value = Evaluate(o, i, m, reflect, mode, sampleFlags);
        if (!(value.pdf > 0)) return {};
        return BSDFSample(Spectrum(float(value.f)), Store(i), float(value.pdf),
            reflect ? BxDFType::GlossyReflection : BxDFType::GlossyTransmission,
            reflect ? 1.0f : float(optics.eta));
    }

    Spectrum f(const Vector3f& wo, const Vector3f& wi, TransportMode mode) const override
    {
        if (eta == 1 || mfDistrib.EffectivelySmooth()) return Spectrum(0.0f);
        const auto o = DielectricDetail::Promote(wo), i = DielectricDetail::Promote(wi);
        Direction m;
        if (!Reconstruct(o, i, m)) return Spectrum(0.0f);
        return Spectrum(float(Evaluate(o, i, m, o.z * i.z > 0, mode, BxDFReflectionType::All).f));
    }

    float Pdf(const Vector3f& wo, const Vector3f& wi, TransportMode mode,
        BxDFReflectionType sampleFlags = BxDFReflectionType::All) const override
    {
        if (eta == 1 || mfDistrib.EffectivelySmooth()) return 0;
        const auto o = DielectricDetail::Promote(wo), i = DielectricDetail::Promote(wi);
        Direction m;
        if (!Reconstruct(o, i, m)) return 0;
        return float(Evaluate(o, i, m, o.z * i.z > 0, mode, sampleFlags).pdf);
    }

    Spectrum rho(const Vector3f& wo, int nSamples, const Point2f* samples) const override
    {
        if (nSamples <= 0 || !samples) return Spectrum(0.0f);
        Spectrum result(0.0f);
        for (int j = 0; j < nSamples; ++j)
        {
            // This legacy rho API supplies only 2D samples. For rough media,
            // estimate both conditional integrals instead of inventing a third RNG dimension.
            if (eta != 1 && !mfDistrib.EffectivelySmooth())
            {
                for (auto event : {BxDFReflectionType::Reflection, BxDFReflectionType::Transmission})
                {
                    const auto bs = Sample_f(wo, 0.5f, samples[j], TransportMode::Radiance, event);
                    if (bs && bs->pdf > 0 && std::isfinite(bs->pdf))
                        result = result + bs->f * (BRDFUtils::AbsCosTheta(bs->wi) / bs->pdf);
                }
            }
            else
            {
                const auto bs = Sample_f(wo, samples[j], TransportMode::Radiance, BxDFReflectionType::All);
                if (bs && bs->pdf > 0 && std::isfinite(bs->pdf))
                    result = result + bs->f * (BRDFUtils::AbsCosTheta(bs->wi) / bs->pdf);
            }
        }
        return result / float(nSamples);
    }
    void Regularize() { mfDistrib.Regularize(); }

private:
    static void Mask(BxDFReflectionType flags, double& pr, double& pt)
    {
        if (!(flags & BxDFReflectionType::Reflection)) pr = 0;
        if (!(flags & BxDFReflectionType::Transmission)) pt = 0;
    }
    double EffectiveEta(Direction o) const { return o.z > 0 ? double(eta) : 1.0 / double(eta); }
    bool Reconstruct(Direction o, Direction i, Direction& m) const
    {
        using namespace DielectricDetail;
        if (i.z == 0 || o.z == 0) return false;
        const double etap = o.z * i.z > 0 ? 1.0 : EffectiveEta(o);
        m = Unit(Combine(i, etap, o, 1));
        if (Dot(m, m) == 0) return false;
        if (m.z < 0) m = {-m.x, -m.y, -m.z};
        return Dot(m, i) * i.z >= 0 && Dot(m, o) * o.z >= 0;
    }
    Evaluation Evaluate(Direction o, Direction i, Direction m, bool reflect,
        TransportMode mode, BxDFReflectionType flags) const
    {
        using namespace DielectricDetail;
        const double a = Dot(i, m), b = Dot(o, m);
        const auto optics = EvaluateInterface(b, 1.0, double(eta));
        double pr = optics.R, pt = optics.T;
        Mask(flags, pr, pt);
        // Converting at the NDF boundary does not round Snell, dot products,
        // the reciprocal index, half-vector reconstruction or Jacobian.
        const Vector3f of = Store(o), inf = Store(i), mf = Store(m);
        const double D = mfDistrib.D(mf), G = mfDistrib.G(of, inf);
        const double pm = mfDistrib.PDF(of, mf);
        const double probability = pr + pt > 0 ? (reflect ? pr : pt) / (pr + pt) : 0;
        if (reflect)
        {
            if (b == 0) return {};
            return {D * G * optics.R / std::abs(4 * i.z * o.z), pm / (4 * std::abs(b)) * probability};
        }
        const double etap = EffectiveEta(o);
        const double h = std::fma(b, 1 / etap, a), denom = h * h;
        if (denom == 0 || optics.tir) return {};
        double value = optics.T * D * G * std::abs(a * b / (i.z * o.z * denom));
        if (mode == TransportMode::Radiance) value /= etap * etap;
        return {value, pm * std::abs(a) / denom * probability};
    }
    float eta;
    TrowbridgeReitzDistribution mfDistrib;
};
