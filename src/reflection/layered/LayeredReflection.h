#pragma once

#include "core/math/HashUtils.h"
#include "reflection/bxdfs/BxDF.h"
#include "reflection/microfacet/MicrofacetReflection.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

enum class LayeredEvaluationMethod { MonteCarlo, AnalyticApproximation };

struct LayeredReflectionOptions
{
    LayeredEvaluationMethod method = LayeredEvaluationMethod::MonteCarlo;
    std::size_t samplesPerEvaluation = 64; // NOT emitter rays per pulse.
    std::size_t maxInternalReflections = 32; // Upper-interface returns to the base.
    std::uint64_t seed = 42;
    bool russianRoulette = true;
    bool useMIS = false; // Opt-in: MIS is not lower variance for every coating.
    std::size_t pdfSamples = 32; // Appended: independent MIS estimate, not f() samples.

    void Validate() const
    {
        switch (method)
        {
        case LayeredEvaluationMethod::MonteCarlo: break;
        case LayeredEvaluationMethod::AnalyticApproximation:
            throw std::logic_error("Analytic layered reflection is not implemented; no automatic fallback.");
        default: throw std::invalid_argument("Unknown layered evaluation method.");
        }
        if (samplesPerEvaluation == 0 || samplesPerEvaluation > 1048576)
            throw std::invalid_argument("Layered samples per evaluation must be in [1,1048576].");
        if (pdfSamples == 0 || pdfSamples > 1048576)
            throw std::invalid_argument("Layered PDF samples must be in [1,1048576].");
        if (maxInternalReflections > 4096)
            throw std::invalid_argument("Layered internal reflection limit must not exceed 4096.");
    }
};

struct LayeredInterfaceSample
{
    double f = 0.0;
    Vector3f wi;
    double pdf = 0.0; // Conditional solid-angle density; probability mass for Delta test interfaces.
};

struct LayeredReflectionEstimate
{
    double directReflection = 0.0;
    double internalReflection = 0.0;
    std::size_t pathsAtReflectionLimit = 0; // Diagnostic, not a lost-energy bound.
    double Brdf() const noexcept { return directReflection + internalReflection; }
};

namespace LayeredReflectionDetail
{
    class RandomStream
    {
    public:
        explicit RandomStream(std::uint64_t seed) : state(seed) {}
        float Next()
        {
            state = HashUtils::MixBits(state + 0x9e3779b97f4a7c15ULL);
            // 23-bit open interval: avoid exact 0/1 at VNDF/hemisphere endpoints.
            return (float(state >> 41) + 0.5f) * (1.0f / 8388608.0f);
        }
        Point2f Next2D() { return {Next(), Next()}; }
    private:
        std::uint64_t state;
    };
    inline double Checked(double value)
    {
        if (!std::isfinite(value) || value < 0)
            throw std::domain_error("Layered reflection produced a negative or nonfinite coefficient.");
        return value;
    }
    inline bool Usable(const LayeredInterfaceSample& sample)
    {
        Checked(sample.f); Checked(sample.pdf);
        if (!std::isfinite(sample.wi.x) || !std::isfinite(sample.wi.y) || !std::isfinite(sample.wi.z))
            throw std::domain_error("Layered interface sampled a nonfinite direction.");
        return sample.f > 0 && sample.pdf > 0 && sample.wi.z != 0;
    }
    inline double Weight(const LayeredInterfaceSample& sample)
    { return Checked(sample.f * std::abs(double(sample.wi.z)) / sample.pdf); }
    inline double Tr(double opticalThickness, const Vector3f& w)
    { return opticalThickness == 0 ? 1.0 : std::exp(-opticalThickness / std::abs(double(w.z))); }
    inline double ConnectionWeight(double sampledPdf, double otherPdf)
    {
        Checked(sampledPdf); Checked(otherPdf);
        const double scale = std::max(sampledPdf, otherPdf);
        if (scale == 0) return 0;
        const double a = sampledPdf / scale, b = otherPdf / scale;
        return a * a / (a * a + b * b); // One sample per technique, power beta=2.
    }
}

// PBRT-inspired scalar two-interface estimator, deliberately narrower than its
// LayeredBxDF: opaque reflecting base, no volume scattering, local +Z exterior.
// Interfaces supply f(), Sample_f(), Pdf() and IsDelta(), using forced events.
// MIS compares both densities in dOmega(v), v pointing UP from the base:
// qExit(v)=top.Pdf(wi,-v,Importance,Transmission), qBase(v)=bottom.Pdf(-w,v,...).
// Do NOT compare qBase to top.Pdf(-v,wi,...): that is a different direction measure.
// Sum all internal-reflection orders up to the explicit cap; finite cap is biased
// relative to an unrestricted layer. RR is probability-compensated separately.
template<class TopInterface, class BottomInterface>
LayeredReflectionEstimate EvaluateScalarLayeredReflectionMonteCarlo(
    const TopInterface& top, const BottomInterface& bottom,
    Vector3f wo, Vector3f wi, double opticalThickness,
    const LayeredReflectionOptions& options = {})
{
    using namespace LayeredReflectionDetail;
    options.Validate();
    if (!std::isfinite(opticalThickness) || opticalThickness < 0)
        throw std::invalid_argument("Layer optical thickness must be finite nonnegative (not meters).");
    wo = MicrofacetReflectionDetail::NormalizeDirection(wo);
    wi = MicrofacetReflectionDetail::NormalizeDirection(wi);
    if (wo.z <= 0 || wi.z <= 0) return {};
    // Reflection in the same exterior medium is reciprocal. Canonical ordering
    // gives identical finite-sample estimates for swapped queries; not evidence
    // by itself that the underlying interface implementation is correct.
    if (std::array{wi.x, wi.y, wi.z} < std::array{wo.x, wo.y, wo.z}) std::swap(wo, wi);
    std::uint64_t querySeed = options.seed;
    for (float component : {wo.x,wo.y,wo.z,wi.x,wi.y,wi.z})
        querySeed = HashUtils::HashCombine(querySeed, HashUtils::FloatToBits(component));
    if (top.IsDelta() && bottom.IsDelta())
        throw std::invalid_argument("Two Delta interfaces need a separate singular layered distribution.");
    LayeredReflectionEstimate result;
    result.directReflection = Checked(top.f(wo, wi, TransportMode::Radiance));
    double sum = 0;
    for (std::size_t s = 0; s < options.samplesPerEvaluation; ++s)
    {
        RandomStream rng(HashUtils::HashCombine(querySeed, s));
        const auto entry = top.Sample_f(wo, rng.Next2D(), TransportMode::Radiance, BxDFReflectionType::Transmission);
        const auto exit = top.Sample_f(wi, rng.Next2D(), TransportMode::Importance, BxDFReflectionType::Transmission);
        // Never retry rejected samples; they contribute zero to the original N.
        if (!entry || !Usable(*entry) || entry->wi.z >= 0) continue;
        const bool validExit = exit && Usable(*exit) && exit->wi.z < 0;
        if (!options.useMIS && !validExit) continue;
        double beta = Weight(*entry) * Tr(opticalThickness, entry->wi);
        const double exitWeight = validExit ? Weight(*exit) * Tr(opticalThickness, exit->wi) : 0;
        Vector3f w = entry->wi; // Arrives at the base from above.
        for (std::size_t r = 0; r <= options.maxInternalReflections; ++r)
        {
            if (beta == 0) break;
            // Proposal E: presampled reverse transmission from the requested wi.
            // Its Delta mass is never compared to a continuous base density.
            if (exitWeight > 0 && (!options.useMIS || !bottom.IsDelta()))
            {
                const double connection = Checked(bottom.f(-w, -exit->wi, TransportMode::Radiance));
                double weight = 1;
                if (options.useMIS && !top.IsDelta())
                    weight = ConnectionWeight(exit->pdf, bottom.Pdf(-w, -exit->wi,
                        TransportMode::Radiance, BxDFReflectionType::Reflection));
                sum = Checked(sum + Checked(beta * connection * exitWeight * weight));
            }
            // Proposal B is also needed at the FINAL allowed base visit (TRT
            // when r=0), even though no further internal reflection is followed.
            const bool baseConnection = options.useMIS && !top.IsDelta();
            std::optional<LayeredInterfaceSample> base;
            if (baseConnection || r < options.maxInternalReflections)
                base = bottom.Sample_f(-w, rng.Next2D(), TransportMode::Radiance, BxDFReflectionType::Reflection);
            const bool validBase = base && Usable(*base) && base->wi.z > 0;
            const double baseBeta = validBase ? Checked(beta * Weight(*base) * Tr(opticalThickness, base->wi)) : 0;
            if (baseConnection && baseBeta > 0)
            {
                // Adjoint evaluation matches the reverse-exit proposal above.
                const double fExit = Checked(top.f(wi, -base->wi, TransportMode::Importance));
                double weight = 1;
                if (!bottom.IsDelta())
                    weight = ConnectionWeight(base->pdf, top.Pdf(wi, -base->wi,
                        TransportMode::Importance, BxDFReflectionType::Transmission));
                sum = Checked(sum + Checked(baseBeta * fExit * weight));
            }
            if (r == options.maxInternalReflections)
            {
                ++result.pathsAtReflectionLimit;
                break;
            }
            if (!validBase) break;
            beta = baseBeta;
            // Always sample the internal reflection branch; its actual Fresnel
            // loss stays in f/pdf. Escaping paths are accounted for by connection.
            const auto reflection = top.Sample_f(-base->wi, rng.Next2D(), TransportMode::Radiance, BxDFReflectionType::Reflection);
            if (!reflection || !Usable(*reflection) || reflection->wi.z >= 0) break;
            beta = Checked(beta * Weight(*reflection) * Tr(opticalThickness, reflection->wi));
            w = reflection->wi;
            if (options.russianRoulette && r >= 3 && beta > 0 && beta < 0.25)
            {
                const double survival = beta;
                if (rng.Next() >= survival) break;
                beta /= survival;
            }
        }
    }
    result.internalReflection = sum / double(options.samplesPerEvaluation);
    Checked(result.Brdf());
    return result;
}
