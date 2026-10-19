#pragma once
#include "core/spectrum/SpectrumInterpolation.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

struct OpticalConstants
{
    double n = 0.0;
    double k = 0.0;
};

struct OpticalConstantsSample
{
    double wavelengthNm = 0.0;
    double n = 0.0;
    double k = 0.0;
};

struct SpectralOpticalConstantsMetadata
{
    std::string name;
    std::string source;
    std::string sampleConditions; // E.g. sample preparation and measurement temperature.
};

// Paired, piecewise-linear optical constants; independent of BRDF and RGB state.
// Wavelength is vacuum wavelength in nm. n/k are dimensionless ABSOLUTE values,
// with the passive-material convention eta = n + i*k, NOT reflectance or percent.
// Neither n nor k is restricted to [0,1]. The incident-medium index stays outside
// this container. No sorting, extrapolation, clamping or implicit unit conversion.
// Linear interpolation is a tabulated-data baseline, not a dispersion model.
class SpectralOpticalConstants
{
public:
    explicit SpectralOpticalConstants(std::vector<OpticalConstantsSample> samples,
        SpectralOpticalConstantsMetadata metadata = {})
        : samples(std::move(samples)), metadata(std::move(metadata))
    {
        Validate();
    }

    OpticalConstants Evaluate(double wavelengthNm) const
    {
        RequireSamples();
        if (!std::isfinite(wavelengthNm) || wavelengthNm <= 0.0)
            throw std::invalid_argument("Optical-constant query wavelength must be finite positive nm.");
        if (!Contains(wavelengthNm))
            throw std::out_of_range("Optical-constant query lies outside the tabulated wavelength range.");

        const auto interval = SpectrumInterpolation::FindInterval(std::span(samples), wavelengthNm,
            [](const OpticalConstantsSample& sample) { return sample.wavelengthNm; });
        const auto& lower = samples[interval.lower];
        const auto& upper = samples[interval.upper];
        const OpticalConstants result{
            interval.Evaluate(lower.n, upper.n),
            interval.Evaluate(lower.k, upper.k)};
        // Underflow can turn tiny, individually valid endpoint pairs into (0,0).
        // Do not silently pass a numerically invalid index into Fresnel evaluation.
        if (!ValidConstants(result.n, result.k))
            throw std::underflow_error("Optical-constant interpolation produced an invalid n/k pair.");
        return result;
    }

    bool Contains(double wavelengthNm) const noexcept
    {
        return samples.size() >= 2 && std::isfinite(wavelengthNm) && wavelengthNm > 0.0 &&
            wavelengthNm >= samples.front().wavelengthNm &&
            wavelengthNm <= samples.back().wavelengthNm;
    }

    std::size_t SampleCount() const noexcept { return samples.size(); }
    double MinimumWavelengthNm() const { RequireSamples(); return samples.front().wavelengthNm; }
    double MaximumWavelengthNm() const { RequireSamples(); return samples.back().wavelengthNm; }
    const std::vector<OpticalConstantsSample>& Samples() const noexcept { return samples; }
    const SpectralOpticalConstantsMetadata& Metadata() const noexcept { return metadata; }

private:
    std::vector<OpticalConstantsSample> samples;
    SpectralOpticalConstantsMetadata metadata;

    static bool ValidConstants(double n, double k) noexcept
    {
        // Matches the existing conductor Fresnel input domain, including k=0.
        return std::isfinite(n) && n >= 0.0 && std::isfinite(k) && k >= 0.0 &&
            (n != 0.0 || k != 0.0);
    }

    void Validate() const
    {
        if (samples.size() < 2)
            throw std::invalid_argument("Optical-constant table requires at least two samples.");
        for (std::size_t i = 0; i < samples.size(); ++i)
        {
            const auto& sample = samples[i];
            if (!std::isfinite(sample.wavelengthNm) || sample.wavelengthNm <= 0.0)
                throw std::invalid_argument("Optical-constant sample wavelength must be finite positive nm.");
            if (!ValidConstants(sample.n, sample.k))
                throw std::invalid_argument("Optical constants require finite nonnegative n/k, not both zero.");
            if (i && sample.wavelengthNm <= samples[i - 1].wavelengthNm)
                throw std::invalid_argument("Optical-constant wavelengths must be strictly increasing without duplicates.");
        }
    }

    void RequireSamples() const
    {
        if (samples.size() < 2)
            throw std::logic_error("Optical-constant table has no valid samples (moved-from object).");
    }
};
