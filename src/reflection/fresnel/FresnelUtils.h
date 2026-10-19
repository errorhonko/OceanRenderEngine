#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <complex>
#include <limits>
#include <stdexcept>
#include <utility>

template <std::floating_point Float>
inline Float FrConductor(Float cosThetaI, Float etaIncident,
    Float etaConductor, Float kappaConductor)
{
    // PBRT v4 complex Fresnel, unpolarized. n/k are absolute optical constants;
    // divide by the real incident-medium index to obtain relative complex eta.
    if (!std::isfinite(cosThetaI) || !std::isfinite(etaIncident) || etaIncident <= 0 ||
        !std::isfinite(etaConductor) || etaConductor < 0 ||
        !std::isfinite(kappaConductor) || kappaConductor < 0 ||
        (etaConductor == 0 && kappaConductor == 0))
        throw std::invalid_argument("Conductor Fresnel requires finite cosine, positive incident index and nonnegative nonzero n/k.");
    cosThetaI = std::clamp(cosThetaI, Float(0), Float(1));
    if (etaConductor == etaIncident && kappaConductor == 0) return Float(0);
    if (cosThetaI == 0) return Float(1); // Avoid the endpoint 0/0 in amplitudes.
    const std::complex<Float> eta(etaConductor / etaIncident, kappaConductor / etaIncident);
    const auto etaSquared = eta * eta;
    if (!std::isfinite(eta.real()) || !std::isfinite(eta.imag()) ||
        !std::isfinite(etaSquared.real()) || !std::isfinite(etaSquared.imag()) ||
        etaSquared == std::complex<Float>(0, 0))
        throw std::overflow_error("Conductor relative index exceeds the numerical range.");
    const Float sin2ThetaI = std::max(Float(0), Float(1) - cosThetaI * cosThetaI);
    const auto cosThetaT = std::sqrt(std::complex<Float>(1, 0) - sin2ThetaI / etaSquared);
    const auto rParallel = (eta * cosThetaI - cosThetaT) / (eta * cosThetaI + cosThetaT);
    const auto rPerpendicular = (cosThetaI - eta * cosThetaT) / (cosThetaI + eta * cosThetaT);
    const Float reflectance = (std::norm(rParallel) + std::norm(rPerpendicular)) / Float(2);
    if (!std::isfinite(reflectance) || reflectance < 0 ||
        reflectance > Float(1) + Float(32) * std::numeric_limits<Float>::epsilon())
        throw std::domain_error("Conductor Fresnel produced an invalid reflectance.");
    return std::clamp(reflectance, Float(0), Float(1)); // Only roundoff at the bound.
}

namespace DielectricDetail {
// Shared lossless, unpolarized Fresnel/Snell arithmetic, local double only.
struct Interface {
    double R, T, eta, cosI, cosT;
    bool tir, flipNormal;
};
inline Interface EvaluateInterface(double cosine, double ni, double nt)
{
    cosine = std::clamp(cosine, -1.0, 1.0);
    const bool flip = cosine < 0;
    if (flip) std::swap(ni, nt);
    const double ci = std::abs(cosine), eta = nt / ni;
    if (ni == nt) return {0, 1, 1, ci, ci, false, flip};
    const double ratio = ni / nt;
    const double sin2I = std::max(0.0, std::fma(-ci, ci, 1.0));
    const double q = std::fma(-(ratio * ratio), sin2I, 1.0);
    if (q <= 0) return {1, 0, eta, ci, 0, true, flip};
    const double ct = std::sqrt(q);
    const double dp = nt * ci + ni * ct, ds = ni * ci + nt * ct;
    const double rp = (nt * ci - ni * ct) / dp;
    const double rs = (ni * ci - nt * ct) / ds;
    const double R = (rp * rp + rs * rs) * .5;
    // Power T = 1-R algebraically, without cancellation near total reflection.
    const double T = 2 * ni * nt * ci * ct * (1 / (dp * dp) + 1 / (ds * ds));
    return {std::clamp(R, 0.0, 1.0), std::clamp(T, 0.0, 1.0), eta, ci, ct, false, flip};
}
}
template <std::floating_point Float>
inline Float FrDielectric(
    Float cosThetaI,
    Float etaI,
    Float etaT)
{
    return static_cast<Float>(DielectricDetail::EvaluateInterface(
        double(cosThetaI), double(etaI), double(etaT)).R);
}
