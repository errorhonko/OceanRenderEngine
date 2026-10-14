#pragma once

#include "lidar/signal_processing/LidarDigitalRangeEstimator.h" // Existing LidarReturnWindow time-coordinate contract.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

struct LidarDigitalPeak
{
    std::size_t peakBinIndex = 0;
    std::size_t plateauFirstBin = 0; // Inclusive.
    std::size_t plateauLastBin = 0;  // Inclusive.
    std::uint32_t peakCode = 0;
    double peakAboveBaselineCodes = 0.0;
    bool plateauOutOfRange = false; // OR over ALL plateau bins, not only the representative.
};

struct LidarDigitalPeakDetectionConfig
{
    // Algorithmic suppression interval in seconds, NOT guaranteed instrument resolution.
    // A pair exactly this far apart is retained. Zero disables distance suppression.
    double minimumPeakSeparationSeconds = 0.0;
};

// Positive-polarity ADC waveform peak detection only: no geometry, ranging or calibration.
// Baseline and threshold are external estimates, in ADC codes (fractional values allowed).
// Flat maxima form one plateau; its midpoint bin represents it, rounding toward earlier time.
// Incomplete maxima touching the valid search interval edges are deliberately excluded.
// Greedy amplitude-ranked suppression is a baseline, not overlapping-waveform decomposition.
class LidarDigitalPeakDetector
{
public:
    explicit LidarDigitalPeakDetector(const LidarDigitalPeakDetectionConfig& config = {})
        : config(config)
    {
        if (!std::isfinite(config.minimumPeakSeparationSeconds) ||
            config.minimumPeakSeparationSeconds < 0.0)
            throw std::invalid_argument("Peak separation must be finite and nonnegative.");
    }

    std::vector<LidarDigitalPeak> Detect(
        const LidarDigitalWaveform& waveform,
        double emissionTimeSeconds,
        const LidarReturnWindow& searchWindow,
        double baselineCode,
        double thresholdCodes) const
    {
        if (!std::isfinite(emissionTimeSeconds) || emissionTimeSeconds < 0.0)
            throw std::invalid_argument("Emission time must be finite and nonnegative.");
        if (!std::isfinite(searchWindow.startTimeSeconds) ||
            !std::isfinite(searchWindow.endTimeSeconds) ||
            searchWindow.endTimeSeconds <= searchWindow.startTimeSeconds)
            throw std::invalid_argument("Peak search window must be finite with positive duration.");
        if (!std::isfinite(baselineCode) || baselineCode < 0.0 ||
            baselineCode > double(waveform.Adc().MaximumCode()))
            throw std::invalid_argument("Peak baseline must lie within ADC code range.");
        if (!std::isfinite(thresholdCodes) || thresholdCodes < 0.0)
            throw std::invalid_argument("Peak threshold must be finite and nonnegative.");

        const auto& samples = waveform.Samples();
        std::size_t begin = samples.size(), end = samples.size();
        double previousTime = 0.0;
        for (std::size_t i = 0; i < samples.size(); ++i)
        {
            const double time = waveform.BinCenterTimeSeconds(i);
            if (!std::isfinite(time) || (i > 0 && time <= previousTime))
                throw std::domain_error("Digital peak time bins must be finite and distinguishable.");
            previousTime = time;
            // Search window is half-open and includes a center equal to emission time.
            if (time >= emissionTimeSeconds && time >= searchWindow.startTimeSeconds &&
                time < searchWindow.endTimeSeconds)
            {
                if (begin == samples.size()) begin = i;
                end = i + 1;
            }
        }
        if (begin == samples.size() || end - begin < 3 ||
            thresholdCodes >= double(waveform.Adc().MaximumCode()) - baselineCode)
            return {};

        const auto amplitude = [&](std::size_t i) {
            return std::max(double(samples[i].code) - baselineCode, 0.0);
        };
        std::vector<LidarDigitalPeak> candidates;
        for (std::size_t left = begin; left < end;)
        {
            std::size_t right = left;
            bool clipped = samples[left].belowRange || samples[left].aboveRange;
            while (right + 1 < end && samples[right + 1].code == samples[left].code)
            {
                ++right;
                clipped = clipped || samples[right].belowRange || samples[right].aboveRange;
            }
            const double height = amplitude(left);
            if (left > begin && right + 1 < end && height > thresholdCodes &&
                height > amplitude(left - 1) && height > amplitude(right + 1))
            {
                candidates.push_back({left + (right - left) / 2, left, right,
                    samples[left].code, height, clipped});
            }
            left = right + 1;
        }

        std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
            if (a.peakAboveBaselineCodes != b.peakAboveBaselineCodes)
                return a.peakAboveBaselineCodes > b.peakAboveBaselineCodes;
            return a.peakBinIndex < b.peakBinIndex;
        });
        std::vector<LidarDigitalPeak> peaks;
        for (const auto& candidate : candidates)
        {
            bool tooClose = false;
            for (const auto& kept : peaks)
            {
                const std::size_t gap = candidate.peakBinIndex > kept.peakBinIndex ?
                    candidate.peakBinIndex - kept.peakBinIndex : kept.peakBinIndex - candidate.peakBinIndex;
                // Relative index difference avoids subtraction of large absolute clocks.
                const double separation = double(gap) * waveform.Config().binWidthSeconds;
                if (separation < config.minimumPeakSeparationSeconds)
                {
                    tooClose = true;
                    break;
                }
            }
            if (!tooClose) peaks.push_back(candidate);
        }
        std::sort(peaks.begin(), peaks.end(), [](const auto& a, const auto& b) {
            return a.peakBinIndex < b.peakBinIndex;
        });
        return peaks;
    }

    const LidarDigitalPeakDetectionConfig& Config() const { return config; }

private:
    LidarDigitalPeakDetectionConfig config;
};
