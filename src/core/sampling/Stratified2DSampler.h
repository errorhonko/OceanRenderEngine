#pragma once

#include "core/sampling/IndependentSampler.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

// PBRT v4-inspired jittered 2D stratification:
// https://pbr-book.org/4ed/Sampling_and_Reconstruction/Stratified_Sampler
// A finite batch, NOT PBRT's sample-index / multi-dimension path sampler.
// Get2D() visits every equal-area cell once, in row-major order.
// For LiDAR, construct one batch per pulse, with samplesPerPulse == SamplesPerBatch().
// Use ALL cells for an equal-weight 1/N estimate; an ordered prefix is not a full-domain estimate.
// Beam PDFs, b/q weights and receiver cosine factors remain unchanged.
class Stratified2DSampler final : public Sampler
{
public:
    // Point2f uses float: prohibit grids finer than its spacing near 1.
    static constexpr std::size_t MaxStrataPerAxis = std::size_t(1) << 24;

    Stratified2DSampler(
        std::size_t xStrata,
        std::size_t yStrata,
        std::uint32_t seed = 0)
        : xStrata(xStrata),
          yStrata(yStrata),
          sampleCount(ValidateBatchSize(xStrata, yStrata)),
          random(seed)
    {
    }

    // Independent 1D draws; these do not advance the 2D cell index.
    // They do advance the shared RNG, so reproducibility assumes the same call sequence.
    float Get1D() override
    {
        return std::min(random.Get1D(), std::nextafter(1.0f, 0.0f));
    }

    Point2f Get2D() override
    {
        if (Exhausted())
            throw std::out_of_range("Stratified 2D sample batch exhausted.");

        const std::size_t index = nextSample++;
        const std::size_t x = index % xStrata;
        const std::size_t y = index / xStrata;
        const Point2f jitter = random.Get2D();

        return Point2f(
            SampleCellCoordinate(x, xStrata, jitter.x),
            SampleCellCoordinate(y, yStrata, jitter.y));
    }

    std::size_t XStrata() const { return xStrata; }
    std::size_t YStrata() const { return yStrata; }
    std::size_t SamplesPerBatch() const { return sampleCount; }
    std::size_t SamplesGenerated() const { return nextSample; }
    std::size_t RemainingSamples() const { return sampleCount - nextSample; }
    bool Exhausted() const { return nextSample == sampleCount; }

private:
    static std::size_t ValidateBatchSize(std::size_t x, std::size_t y)
    {
        if (x == 0 || y == 0)
            throw std::invalid_argument("Stratified axis counts must be positive.");
        if (x > std::numeric_limits<std::size_t>::max() / y)
            throw std::length_error("Stratified 2D sample count overflow.");
        if (x > MaxStrataPerAxis || y > MaxStrataPerAxis)
            throw std::invalid_argument("Stratified grid exceeds Point2f precision.");
        return x * y;
    }

    static float SampleCellCoordinate(
        std::size_t cell, std::size_t count, float jitter)
    {
        const double lower = double(cell) / double(count);
        const double upper = double(cell + 1) / double(count);
        const double value = (double(cell) + double(jitter)) / double(count);

        // Keep the ACTUAL float result in [lower, upper), not just [0, 1).
        // Rounding near an internal boundary must not move it to another cell.
        float first = float(lower);
        if (double(first) < lower)
            first = std::nextafter(first, 1.0f);
        float last = float(upper);
        if (double(last) >= upper)
            last = std::nextafter(last, 0.0f);
        return std::clamp(float(value), first, last);
    }

    std::size_t xStrata;
    std::size_t yStrata;
    std::size_t sampleCount;
    std::size_t nextSample = 0;
    IndependentSampler random;
};
