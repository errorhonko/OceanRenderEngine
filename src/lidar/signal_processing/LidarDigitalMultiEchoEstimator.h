#pragma once

#include "lidar/signal_processing/LidarDigitalPeakDetector.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

struct LidarDigitalMultiEchoConfig
{
    LidarDigitalPeakDetectionConfig peakDetection;
    // Bin counts, not instrument resolution. Tune/calibrate for the receiver response.
    LidarPeakNeighborhood neighborhood{24, 47};
};

struct LidarDigitalEchoEstimate
{
    LidarDigitalPeak peak;
    std::size_t firstBinIndex = 0; // Inclusive centroid window, contains entire peak plateau.
    std::size_t lastBinIndex = 0;  // Inclusive; no bin shared by adjacent echoes.
    LidarDigitalRangeEstimate measured; // Raw centroid time/range; no system-delay correction.
    bool localWindowHasOutOfRangeSamples = false;
};

// Independent signal-processing module: detection -> disjoint local windows -> raw ranging.
// A valley partition is an engineering baseline, NOT deconvolution or attribution of true objects.
// Positive above-baseline samples (including weak tails) participate in each centroid.
// Apply LidarRangeCalibration externally, using a calibration appropriate for this estimator/window.
class LidarDigitalMultiEchoEstimator
{
public:
    explicit LidarDigitalMultiEchoEstimator(const LidarDigitalMultiEchoConfig& config = {})
        : config(config), detector(config.peakDetection)
    {}

    std::vector<LidarDigitalEchoEstimate> Estimate(
        const LidarDigitalWaveform& waveform,
        double emissionTimeSeconds,
        const LidarReturnWindow& searchWindow,
        double baselineCode,
        double thresholdCodes) const
    {
        // Reuse detector validation and peak identity; never run a global maximum search per echo.
        const auto peaks = detector.Detect(waveform, emissionTimeSeconds, searchWindow, baselineCode, thresholdCodes);
        if (peaks.empty()) return {};
        const auto& samples = waveform.Samples();
        std::size_t begin = samples.size(), end = samples.size();
        bool searchClipped = false;
        for (std::size_t i = 0; i < samples.size(); ++i)
        {
            const double t = waveform.BinCenterTimeSeconds(i);
            if (t >= emissionTimeSeconds && t >= searchWindow.startTimeSeconds && t < searchWindow.endTimeSeconds)
            {
                if (begin == samples.size()) begin = i;
                end = i + 1;
                searchClipped = searchClipped || samples[i].belowRange || samples[i].aboveRange;
            }
        }

        std::vector<std::size_t> cuts;
        for (std::size_t j = 1; j < peaks.size(); ++j)
        {
            const std::size_t left = peaks[j - 1].plateauLastBin + 1;
            const std::size_t right = peaks[j].plateauFirstBin - 1;
            std::size_t valley = left;
            // Preserve a half-bin midpoint; rounding it first can favor the wrong equal minimum.
            const auto distance = [left, right](std::size_t i) {
                return std::abs(double(i - left) - 0.5 * double(right - left));
            };
            for (std::size_t i = left + 1; i <= right; ++i)
            {
                if (samples[i].code < samples[valley].code ||
                    (samples[i].code == samples[valley].code && distance(i) < distance(valley)))
                    valley = i;
            }
            // Valley belongs to earlier echo; the later one starts at valley + 1.
            cuts.push_back(valley);
        }

        std::vector<LidarDigitalEchoEstimate> result;
        result.reserve(peaks.size());
        for (std::size_t j = 0; j < peaks.size(); ++j)
        {
            const auto& peak = peaks[j];
            const std::size_t index = peak.peakBinIndex;
            std::size_t first = index - std::min(config.neighborhood.binsBefore, index);
            std::size_t last = index + std::min(config.neighborhood.binsAfter, samples.size() - 1 - index);
            // Even a zero-radius neighborhood must not discard part of an ADC plateau.
            first = std::min(first, peak.plateauFirstBin);
            last = std::max(last, peak.plateauLastBin);
            first = std::max(first, j == 0 ? begin : cuts[j - 1] + 1);
            last = std::min(last, j + 1 == peaks.size() ? end - 1 : cuts[j]);

            double totalWeight = 0.0, meanOffsetBins = 0.0;
            bool localClipped = false;
            for (std::size_t i = first; i <= last; ++i)
            {
                localClipped = localClipped || samples[i].belowRange || samples[i].aboveRange;
                const double amplitude = std::max(double(samples[i].code) - baselineCode, 0.0);
                if (amplitude == 0.0) continue;
                // Normalization cancels in centroid; avoids large unnormalized sums.
                const double weight = amplitude / peak.peakAboveBaselineCodes;
                const double newWeight = totalWeight + weight;
                if (!std::isfinite(weight) || !std::isfinite(newWeight))
                    throw std::overflow_error("Multi-echo centroid weights overflow.");
                meanOffsetBins += (double(i - first) - meanOffsetBins) * (weight / newWeight);
                totalWeight = newWeight;
            }
            const double tof = (waveform.BinCenterTimeSeconds(first) - emissionTimeSeconds) +
                meanOffsetBins * waveform.Config().binWidthSeconds;
            const double arrival = emissionTimeSeconds + tof;
            const double range = (0.5 * 299792458.0) * tof;
            if (!std::isfinite(tof) || !std::isfinite(arrival) || !std::isfinite(range))
                throw std::overflow_error("Multi-echo centroid time or range overflow.");
            const auto& representative = samples[index];
            result.push_back({peak, first, last,
                {index, arrival, tof, range, peak.peakCode, peak.peakAboveBaselineCodes,
                    representative.belowRange || representative.aboveRange, searchClipped}, localClipped});
        }
        return result;
    }

    const LidarDigitalMultiEchoConfig& Config() const { return config; }

private:
    LidarDigitalMultiEchoConfig config;
    LidarDigitalPeakDetector detector;
};
