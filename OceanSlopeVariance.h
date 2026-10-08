#pragma once

#include "OceanHeightField.h"

#include <cmath>
#include <functional>
#include <numbers>
#include <stdexcept>

// Ensemble slope statistics in the horizontal world X/Z axes.
// These are dimensionless and are not local tangent-frame roughnesses.
struct OceanSlopeVariance
{
    double varianceX = 0.0;
    double varianceZ = 0.0;
    double covarianceXZ = 0.0;

    double RmsX() const { return std::sqrt(varianceX); }
    double RmsZ() const { return std::sqrt(varianceZ); }
    double TotalVariance() const { return varianceX + varianceZ; }
};

// Input is the two-sided Cartesian height PSD Psi(kx, kz), in m^4.
// Wave numbers are in rad/m. No FFT normalization or random seed is needed.
using OceanCartesianSpectrum = std::function<double(double, double)>;

inline OceanSlopeVariance IntegrateSlopeVariance(
    const OceanCartesianSpectrum& spectrum,
    double minWaveNumber,
    double maxWaveNumber,
    int radialSamples = 512,
    int angularSamples = 128)
{
    if (!spectrum ||
        !std::isfinite(minWaveNumber) ||
        !std::isfinite(maxWaveNumber) ||
        minWaveNumber <= 0.0 || maxWaveNumber < minWaveNumber ||
        radialSamples <= 0 || angularSamples < 8)
    {
        throw std::invalid_argument(
            "Slope integration requires a spectrum, 0 < kMin <= kMax, "
            "positive radial samples and at least 8 angular samples.");
    }

    OceanSlopeVariance result;
    if (minWaveNumber == maxWaveNumber)
        return result;

    // Polar coordinates with u = log(k):
    // kx^2 Psi dkx dkz = cos(theta)^2 k^4 Psi du dtheta.
    const double logMin = std::log(minWaveNumber);
    const double du =
        (std::log(maxWaveNumber) - logMin) / radialSamples;
    const double dTheta =
        2.0 * std::numbers::pi_v<double> / angularSamples;

    for (int r = 0; r < radialSamples; ++r)
    {
        const double k = std::exp(logMin + (r + 0.5) * du);
        for (int a = 0; a < angularSamples; ++a)
        {
            const double theta = (a + 0.5) * dTheta;
            const double c = std::cos(theta);
            const double s = std::sin(theta);
            const double psi = spectrum(k * c, k * s);
            if (!std::isfinite(psi) || psi < 0.0)
                throw std::domain_error("Height PSD must be finite and non-negative.");
            if (psi == 0.0)
                continue;

            const double weight = psi * k * k * k * k * du * dTheta;
            if (!std::isfinite(weight))
                throw std::overflow_error("Slope integral contribution overflow.");

            result.varianceX += c * c * weight;
            result.varianceZ += s * s * weight;
            result.covarianceXZ += c * s * weight;
        }
    }

    if (!std::isfinite(result.varianceX) ||
        !std::isfinite(result.varianceZ) ||
        !std::isfinite(result.covarianceXZ))
    {
        throw std::overflow_error("Slope integral overflow.");
    }
    return result;
}

// Use the exact same cutoff as the resolved height/slope field.
// Modes beyond maxWaveNumber are excluded, not silently counted as zero.
inline OceanSlopeVariance IntegrateUnresolvedSlopeVariance(
    const OceanCartesianSpectrum& spectrum,
    const OceanHeightField& heightField,
    double maxWaveNumber,
    int radialSamples = 512,
    int angularSamples = 128)
{
    return IntegrateSlopeVariance(
        spectrum, heightField.CutoffWaveNumber(), maxWaveNumber,
        radialSamples, angularSamples);
}
