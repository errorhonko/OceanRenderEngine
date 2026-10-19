#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <span>
#include <stdexcept>

// Numeric operations only: no reflectance, n/k, RGB, unit or file-format policy.
namespace SpectrumInterpolation
{
    template<std::floating_point Float>
    Float Weight(Float x, Float x0, Float x1)
    {
        if (!std::isfinite(x) || !std::isfinite(x0) || !std::isfinite(x1) || x0 >= x1)
            throw std::invalid_argument("Interpolation requires finite coordinates and an increasing interval.");
        if (x < x0 || x > x1)
            throw std::out_of_range("Interpolation query lies outside the interval.");
        if (x == x0) return Float(0);
        if (x == x1) return Float(1);
        const Float width = x1 - x0;
        if (!std::isfinite(width))
            throw std::overflow_error("Interpolation interval width exceeds the numerical range.");
        return (x - x0) / width;
    }

    struct Interval
    {
        std::size_t lower = 0;
        std::size_t upper = 0;
        double weight = 0.0;

        template<std::floating_point Float>
        Float Evaluate(Float lowerValue, Float upperValue) const
        {
            return std::lerp(lowerValue, upperValue, static_cast<Float>(weight));
        }
    };

    // PRECONDITION: samples were validated as finite, strictly increasing by
    // coordinate. Business containers validate once at construction, not on each
    // O(log N) query. This function guards size/query/support before dereferencing.
    // Projection keeps a wavelength table independent of its stored value type.
    template<class Sample, std::size_t Extent, class Projection>
    Interval FindInterval(std::span<const Sample, Extent> samples, double x,
        Projection coordinate)
    {
        if (samples.size() < 2)
            throw std::invalid_argument("Interpolation table requires at least two samples.");
        if (!std::isfinite(x))
            throw std::invalid_argument("Interpolation query must be finite.");
        if (x < coordinate(samples.front()) || x > coordinate(samples.back()))
            throw std::out_of_range("Interpolation query lies outside table support.");
        const auto upper = std::lower_bound(samples.begin(), samples.end(), x,
            [&](const Sample& sample, double value) { return coordinate(sample) < value; });
        const auto upperIndex = static_cast<std::size_t>(upper - samples.begin());
        if (coordinate(*upper) == x)
            return {upperIndex, upperIndex, 0.0}; // Preserve exact table knots.
        const auto lowerIndex = upperIndex - 1;
        return {lowerIndex, upperIndex,
            Weight(x, double(coordinate(samples[lowerIndex])), double(coordinate(*upper)))};
    }

    // Legacy single-segment API uses endpoint extension, NOT extrapolation.
    // This policy is never implicitly applied to measured spectral containers.
    template<std::floating_point Float>
    Float InterpolateClamped(Float x, Float x0, Float x1, Float y0, Float y1)
    {
        if (!std::isfinite(x) || !std::isfinite(x0) || !std::isfinite(x1) || x0 >= x1 ||
            !std::isfinite(y0) || !std::isfinite(y1))
            throw std::invalid_argument("Clamped interpolation requires finite data and an increasing interval.");
        const Float weight = Weight(std::clamp(x, x0, x1), x0, x1);
        return Interval{0, 1, double(weight)}.Evaluate(y0, y1);
    }
}
