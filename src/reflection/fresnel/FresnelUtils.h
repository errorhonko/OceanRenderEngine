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

template <std::floating_point Float>
inline Float FrDielectric(
    Float cosThetaI,
    Float etaI,
    Float etaT)
{
    cosThetaI = std::clamp(
        cosThetaI,
        Float(-1),
        Float(1));

    const bool entering =
        cosThetaI > Float(0);

    if (!entering)
    {
        std::swap(etaI, etaT);
        cosThetaI = std::abs(cosThetaI);
    }

    const Float sinThetaI =
        std::sqrt(std::max(
            Float(0),
            Float(1) -
            cosThetaI * cosThetaI));

    const Float sinThetaT =
        etaI / etaT * sinThetaI;

    // 从高折射率介质进入低折射率介质时可能全反射。
    if (sinThetaT >= Float(1))
        return Float(1);

    const Float cosThetaT =
        std::sqrt(std::max(
            Float(0),
            Float(1) -
            sinThetaT * sinThetaT));

    const Float rParallel =
        (etaT * cosThetaI -
            etaI * cosThetaT) /
        (etaT * cosThetaI +
            etaI * cosThetaT);

    const Float rPerpendicular =
        (etaI * cosThetaI -
            etaT * cosThetaT) /
        (etaI * cosThetaI +
            etaT * cosThetaT);

    return
        (rParallel * rParallel +
            rPerpendicular * rPerpendicular) /
        Float(2);
}
