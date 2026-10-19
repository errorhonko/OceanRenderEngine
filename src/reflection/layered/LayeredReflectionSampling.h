#pragma once
#include "reflection/layered/LayeredReflection.h"
#include <limits>
#include <numbers>

// A sampled INTERNAL path, not f(wo,wi) and the marginal direction density.
// Only f*abs(wi.z)/pdf is the transport weight. The pair may be rescaled together.
struct LayeredPathSample
{
    double f = 0;
    Vector3f wi;
    double pdf = 0;
    BxDFType flags = BxDFType::Unset;
    bool pdfIsProportional = true;
    std::size_t internalReflections = 0, rouletteSurvivals = 0, pairRescalings = 0;
};

namespace LayeredReflectionDetail {
inline bool AllowsLayeredReflection(BxDFReflectionType flags)
{
    if (flags == BxDFReflectionType::All || flags == BxDFReflectionType::Reflection) return true;
    if (flags == BxDFReflectionType::Unset || flags == BxDFReflectionType::Transmission) return false;
    throw std::invalid_argument("Unknown layered reflection flags.");
}
inline void RequireLayeredSample(float u)
{
    if (!std::isfinite(u) || u < 0 || u >= 1)
        throw std::invalid_argument("Layered sample coordinates must be finite in [0,1).");
}
inline std::uint64_t DirectionSeed(std::uint64_t seed, Vector3f w)
{
    for (float x : {w.x,w.y,w.z}) seed = HashUtils::HashCombine(seed, HashUtils::FloatToBits(x));
    return seed;
}
struct PathWeightPair
{
    double numerator = 1, denominator = 1;
    std::size_t rescalings = 0;
    bool Multiply(double f, double p)
    {
        Checked(f); Checked(p);
        if (f == 0 || p == 0) return false;
        numerator *= f; denominator *= p;
        Checked(numerator); Checked(denominator);
        if (numerator == 0 || denominator == 0) return false;
        const double scale = std::max(numerator, denominator);
        if (scale > 1e40 || std::min(numerator,denominator) < 1e-40)
        {
            numerator /= scale; denominator /= scale; ++rescalings;
        }
        return numerator > 0 && denominator > 0;
    }
    double Throughput() const { return Checked(numerator / denominator); }
};
inline LayeredPathSample FinishLayeredPath(const PathWeightPair& pair, Vector3f wi, bool delta,
    std::size_t reflections, std::size_t survivors)
{
    return {pair.numerator, wi, pair.denominator,
        delta ? BxDFType::SpecularReflection : BxDFType::GlossyReflection,
        true, reflections, survivors, pair.rescalings};
}
}

// PBRT-style random walk, specialized to a non-transmitting base, no volume.
// M is the number of permitted upper-interface reflections BACK TO THE BASE.
// M=0 still includes direct R and TRT, exactly as the existing f estimator.
// External All / Reflection both permit all these internal R/T decisions.
template<class TopInterface, class BottomInterface>
std::optional<LayeredPathSample> SampleScalarLayeredReflection(
    const TopInterface& top, const BottomInterface& bottom, Vector3f wo,
    float uc, Point2f u, double opticalThickness, const LayeredReflectionOptions& options = {},
    TransportMode mode = TransportMode::Radiance,
    BxDFReflectionType flags = BxDFReflectionType::All)
{
    using namespace LayeredReflectionDetail;
    options.Validate();
    if (!std::isfinite(opticalThickness) || opticalThickness < 0)
        throw std::invalid_argument("Layer optical thickness must be finite nonnegative.");
    RequireLayeredSample(uc); RequireLayeredSample(u.x); RequireLayeredSample(u.y);
    wo = MicrofacetReflectionDetail::NormalizeDirection(wo);
    if (!AllowsLayeredReflection(flags) || wo.z <= 0) return {};
    const auto first = top.SampleAll(wo, uc, u, mode);
    if (!first || !Usable(*first)) return {};
    PathWeightPair pair;
    if (first->wi.z > 0)
    {
        if (!pair.Multiply(first->f, first->pdf)) return {};
        return FinishLayeredPath(pair, first->wi, top.IsDelta(), 0, 0);
    }
    if (!pair.Multiply(first->f * std::abs(double(first->wi.z)), first->pdf) ||
        !pair.Multiply(Tr(opticalThickness,first->wi),1)) return {};
    std::uint64_t seed = DirectionSeed(options.seed ^ 0xb6a45e273c1d9f08ULL,wo);
    for (float x : {uc,u.x,u.y}) seed = HashUtils::HashCombine(seed,HashUtils::FloatToBits(x));
    RandomStream rng(seed); // Separate domain from fixed-direction f and PDF estimates.
    Vector3f w = first->wi;
    bool specular = top.IsDelta();
    std::size_t survivors = 0;
    for (std::size_t r=0; r<=options.maxInternalReflections; ++r)
    {
        const auto base = bottom.Sample_f(-w,rng.Next2D(),mode,BxDFReflectionType::Reflection);
        if (!base || !Usable(*base) || base->wi.z <= 0) return {};
        if (!pair.Multiply(base->f * std::abs(double(base->wi.z)),base->pdf) ||
            !pair.Multiply(Tr(opticalThickness,base->wi),1)) return {};
        specular = specular && bottom.IsDelta();
        const float event = rng.Next();
        const Point2f direction = rng.Next2D(); // Explicit order, no argument-order dependency.
        const auto upper = top.SampleAll(-base->wi,event,direction,mode);
        if (!upper || !Usable(*upper)) return {};
        if (upper->wi.z > 0)
        {
            // Final outgoing cosine belongs to the caller, NOT this numerator.
            if (!pair.Multiply(upper->f,upper->pdf)) return {};
            return FinishLayeredPath(pair,upper->wi,specular && top.IsDelta(),r,survivors);
        }
        if (r == options.maxInternalReflections) return {}; // Cap, not roulette.
        if (!pair.Multiply(upper->f * std::abs(double(upper->wi.z)),upper->pdf) ||
            !pair.Multiply(Tr(opticalThickness,upper->wi),1)) return {};
        w = upper->wi;
        const double beta = pair.Throughput(); // Includes all preceding cosines.
        if (options.russianRoulette && r>=3 && beta<.25)
        {
            const double survival = beta;
            if (survival <= 0 || rng.Next() >= survival) return {};
            // PBRT's denominator bookkeeping; do NOT also divide numerator by survival.
            if (!pair.Multiply(1,survival)) return {};
            ++survivors;
        }
    }
    return {};
}

// Independent deterministic-for-query MIS proxy, NOT an exact normalized
// marginal of SampleScalarLayeredReflection. Like PBRT: direct R + stochastic
// first TRT approximation + a positive uniform tail for omitted orders.
// Unlike blind copying: every inner MIS comparison uses the SAME INTERNAL v;
// the exit ALL PDF below remains the integrand's density in external wi.
template<class TopInterface, class BottomInterface>
double EstimateScalarLayeredPdfForMIS(const TopInterface& top, const BottomInterface& bottom,
    Vector3f wo, Vector3f wi, const LayeredReflectionOptions& options = {},
    TransportMode mode = TransportMode::Radiance,
    BxDFReflectionType flags = BxDFReflectionType::All)
{
    using namespace LayeredReflectionDetail;
    options.Validate();
    wo = MicrofacetReflectionDetail::NormalizeDirection(wo);
    wi = MicrofacetReflectionDetail::NormalizeDirection(wi);
    if (!AllowsLayeredReflection(flags) || wo.z<=0 || wi.z<=0) return 0;
    if (bottom.IsDelta())
        throw std::invalid_argument("Layered MIS PDF currently requires a continuous opaque base.");
    const double direct = Checked(top.PdfAll(wo,wi,mode));
    double sum = 0;
    const auto reverseMode = mode==TransportMode::Radiance ? TransportMode::Importance : TransportMode::Radiance;
    std::uint64_t seed = DirectionSeed(options.seed ^ 0x4c38d129a7f06be5ULL,wo);
    seed = DirectionSeed(seed,wi); // Do not reuse f()'s canonical-query stream.
    RandomStream rng(seed);
    if (bottom.Reflectance()>0)
    for (std::size_t s=0; s<options.pdfSamples; ++s)
    {
        const auto entry=top.Sample_f(wo,rng.Next2D(),mode,BxDFReflectionType::Transmission);
        const auto exit=top.Sample_f(wi,rng.Next2D(),reverseMode,BxDFReflectionType::Transmission);
        if (!entry || !Usable(*entry) || entry->wi.z>=0) continue;
        const bool validExit=exit && Usable(*exit) && exit->wi.z<0;
        if (top.IsDelta())
        {
            if (!validExit) continue;
            // Deterministic Snell map: use inverse solid-angle Jacobian,
            // not the inside density as if it were per outside solid angle.
            const double eta=top.RelativeEta();
            const auto entryOptics=DielectricDetail::EvaluateInterface(
                DielectricDetail::Promote(wo).z,1,eta);
            const auto exitOptics=DielectricDetail::EvaluateInterface(
                -std::abs(DielectricDetail::Promote(exit->wi).z),1,eta);
            const double jacobian=std::abs(DielectricDetail::Promote(wi).z) /
                (eta*eta*std::abs(DielectricDetail::Promote(exit->wi).z));
            sum += entryOptics.T * bottom.Pdf(-entry->wi,-exit->wi,mode,BxDFReflectionType::Reflection) *
                exitOptics.T * jacobian;
            continue;
        }
        const double entryProbability=Checked(top.PdfAll(wo,entry->wi,mode)/entry->pdf);
        if (validExit)
        {
            const Vector3f v=-exit->wi;
            const double pb=Checked(bottom.Pdf(-entry->wi,v,mode,BxDFReflectionType::Reflection));
            const double pe=Checked(top.PdfAll(-v,wi,mode));
            sum += entryProbability * pb * pe / exit->pdf * ConnectionWeight(exit->pdf,pb);
        }
        // The independent base proposal must survive a rejected exit sample.
        const auto base=bottom.Sample_f(-entry->wi,rng.Next2D(),mode,BxDFReflectionType::Reflection);
        if (base && Usable(*base) && base->wi.z>0)
        {
            const double qe=Checked(top.Pdf(wi,-base->wi,reverseMode,BxDFReflectionType::Transmission));
            const double pe=Checked(top.PdfAll(-base->wi,wi,mode));
            sum += entryProbability * pe * ConnectionWeight(base->pdf,qe);
        }
    }
    const double approximate = direct + sum/double(options.pdfSamples);
    // Reflection-only exterior: uniform HEMISPHERE, rather than PBRT's sphere.
    return Checked(.9*approximate + .1/(2*std::numbers::pi_v<double>));
}
