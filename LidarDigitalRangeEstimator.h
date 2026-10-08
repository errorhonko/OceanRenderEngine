#pragma once

#include "LidarDigitalWaveform.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <algorithm>
struct LidarReturnWindow
{
    // 与波形相同的时间坐标，单位 s。
    // 按时间箱中心判断是否属于 [start, end)。
    double startTimeSeconds = 0.0;
    double endTimeSeconds = 0.0;
};

struct LidarDigitalRangeEstimate
{
    std::size_t peakBinIndex = 0;

    double arrivalTimeSeconds = 0.0;
    double timeOfFlightSeconds = 0.0;
    double rangeMeters = 0.0;

    std::uint32_t peakCode = 0;
    // 扣除基线后的幅值，单位为码，允许小数。
    double peakAboveBaselineCodes = 0.0;

    // 峰值箱是否严格超出 ADC 量程。
    bool peakOutOfRange = false;

    // 本次有效搜索区域内是否出现过越界（发射后且位于窗口内）。
    bool searchHasOutOfRangeSamples = false;
};

struct LidarPeakNeighborhood
{
    // 均不包含峰值箱。
    std::size_t binsBefore = 0;
    std::size_t binsAfter = 0;
};

class LidarDigitalRangeEstimator
{
public:
    explicit LidarDigitalRangeEstimator(
        double baselineCode = 0.0,
        double minimumPeakAboveBaselineCodes = 0.0)
        : baselineCode(baselineCode),
        minimumPeakAboveBaselineCodes(
            minimumPeakAboveBaselineCodes)
    {
        if (!std::isfinite(baselineCode) || baselineCode < 0.0)
        {
            throw std::invalid_argument(
                "Baseline code must be finite and nonnegative.");
        }

        if (!std::isfinite(minimumPeakAboveBaselineCodes) ||
            minimumPeakAboveBaselineCodes < 0.0)
        {
            throw std::invalid_argument(
                "Peak amplitude threshold must be finite and nonnegative.");
        }
    }

    // 正向脉冲、单站、单回波峰值基线。
    std::optional<LidarDigitalRangeEstimate> Estimate(
        const LidarDigitalWaveform& waveform,
        double emissionTimeSeconds,
        std::optional<LidarReturnWindow> window =
            std::nullopt) const
    {
        if (!std::isfinite(emissionTimeSeconds) ||
            emissionTimeSeconds < 0.0)
        {
            throw std::invalid_argument(
                "Emission time must be finite and nonnegative.");
        }

        if (window)
        {
            if (!std::isfinite(window->startTimeSeconds) ||
                !std::isfinite(window->endTimeSeconds) ||
                window->endTimeSeconds <=
                window->startTimeSeconds)
            {
                throw std::invalid_argument(
                    "Return window must be finite and have positive duration.");
            }
        }

        const std::uint32_t maximumCode =
            waveform.Adc().MaximumCode();

        if (baselineCode > maximumCode)
        {
            throw std::invalid_argument(
                "Baseline code exceeds ADC code range.");
        }

        if (minimumPeakAboveBaselineCodes >
            maximumCode - baselineCode)
        {
            throw std::invalid_argument(
                "Peak threshold exceeds available code range.");
        }

        const auto& samples = waveform.Samples();

        std::optional<std::size_t> peakIndex;
        double peakAmplitudeCodes = 0.0;
        bool searchHasOutOfRangeSamples = false;

        for (std::size_t i = 0; i < samples.size(); ++i)
        {
            const double binTime =
                waveform.BinCenterTimeSeconds(i);

            if (!std::isfinite(binTime))
            {
                throw std::domain_error(
                    "Digital waveform time must be finite.");
            }

            if (binTime < emissionTimeSeconds)
                continue;

            if (window &&
                (binTime < window->startTimeSeconds ||
                 binTime >= window->endTimeSeconds))
            {
                continue;
            }

            const auto& sample = samples[i];

            searchHasOutOfRangeSamples =
                searchHasOutOfRangeSamples ||
                sample.belowRange ||
                sample.aboveRange;

            // 原始数字码保持整数，扣基线时转换为 double。
            const double amplitudeCodes =
                std::max(static_cast<double>(sample.code) - baselineCode, 0.0);

            // 等高峰保留较早的时间箱。
            if (amplitudeCodes > peakAmplitudeCodes)
            {
                peakAmplitudeCodes = amplitudeCodes;
                peakIndex = i;
            }
        }

        if (!peakIndex ||
            peakAmplitudeCodes <=
            minimumPeakAboveBaselineCodes)
        {
            return std::nullopt;
        }

        const double arrivalTime =
            waveform.BinCenterTimeSeconds(*peakIndex);

        const double timeOfFlight =
            arrivalTime - emissionTimeSeconds;

        constexpr double speedOfLight = 299792458.0;

        const double range =
            (0.5 * speedOfLight) * timeOfFlight;

        if (!std::isfinite(timeOfFlight) ||
            !std::isfinite(range))
        {
            throw std::overflow_error(
                "Digital range estimate overflow.");
        }

        const auto& peak = samples[*peakIndex];

        return LidarDigitalRangeEstimate{
            *peakIndex,
            arrivalTime,
            timeOfFlight,
            range,
            peak.code,
            peakAmplitudeCodes,
            peak.belowRange || peak.aboveRange,
            searchHasOutOfRangeSamples
        };
    }

    // 已知基线、无噪声、单回波的数字幅值重心。
    // 使用有效搜索区域内的所有正向幅值，不自动分离多个回波。
    std::optional<LidarDigitalRangeEstimate> EstimateCentroid(
        const LidarDigitalWaveform& waveform,
        double emissionTimeSeconds,
        std::optional<LidarReturnWindow> window =
            std::nullopt) const
    {
        // 复用峰值法的参数检查、检出门限和越界标志。
        auto result = Estimate(
            waveform,
            emissionTimeSeconds,
            window);

        if (!result)
            return std::nullopt;

        const auto& samples = waveform.Samples();

        double totalWeight = 0.0;
        double meanTimeOfFlight = 0.0;

        for (std::size_t i = 0; i < samples.size(); ++i)
        {
            const double binTime =
                waveform.BinCenterTimeSeconds(i);

            if (binTime < emissionTimeSeconds)
                continue;

            if (window &&
                (binTime < window->startTimeSeconds ||
                 binTime >= window->endTimeSeconds))
            {
                continue;
            }

            const double code = static_cast<double>(samples[i].code);

            // 仅正向幅值参与加权，保留小数基线的影响。
            if (code <= baselineCode)
                continue;

            const double amplitudeCodes =
                code - baselineCode;

            const double timeOfFlight =
                binTime - emissionTimeSeconds;

            if (!std::isfinite(timeOfFlight))
            {
                throw std::overflow_error(
                    "Digital centroid time overflow.");
            }

            // 用峰值幅值归一化，不改变重心。
            const double weight =
                amplitudeCodes / result->peakAboveBaselineCodes;

            const double newTotalWeight =
                totalWeight + weight;

            if (!std::isfinite(newTotalWeight))
            {
                throw std::overflow_error(
                    "Digital centroid weight overflow.");
            }

            // 增量加权平均，避免直接累加 weight * time。
            meanTimeOfFlight +=
                (timeOfFlight - meanTimeOfFlight) *
                (weight / newTotalWeight);

            totalWeight = newTotalWeight;
        }

        constexpr double speedOfLight = 299792458.0;

        const double arrivalTime =
            emissionTimeSeconds + meanTimeOfFlight;

        const double range =
            (0.5 * speedOfLight) * meanTimeOfFlight;

        if (!std::isfinite(meanTimeOfFlight) ||
            !std::isfinite(arrivalTime) ||
            !std::isfinite(range))
        {
            throw std::overflow_error(
                "Digital centroid range overflow.");
        }

        // 只替换时间和距离，保留峰值及越界诊断。
        result->arrivalTimeSeconds = arrivalTime;
        result->timeOfFlightSeconds = meanTimeOfFlight;
        result->rangeMeters = range;

        return result;
    }


        std::optional<LidarDigitalRangeEstimate>
            EstimatePeakNeighborhoodCentroid(
                const LidarDigitalWaveform& waveform,
                double emissionTimeSeconds,
                const LidarPeakNeighborhood& neighborhood,
                std::optional<LidarReturnWindow> searchWindow =
                std::nullopt) const
        {
            // 先在原搜索范围检出峰值。
            // 同时复用参数检查和检测门限。
            auto peak = Estimate(
                waveform,
                emissionTimeSeconds,
                searchWindow);

            if (!peak)
                return std::nullopt;

            const std::size_t count = waveform.Samples().size();
            const std::size_t peakIndex = peak->peakBinIndex;

            // 先限制距离再做加减，避免无符号下溢和溢出。
            const std::size_t firstIndex =
                peakIndex -
                std::min(neighborhood.binsBefore, peakIndex);

            const std::size_t lastIndex =
                peakIndex +
                std::min(
                    neighborhood.binsAfter,
                    count - 1 - peakIndex);

            const auto& config = waveform.Config();

            // 用箱边界构造区间：
            // 包含 firstIndex 到 lastIndex 的完整时间箱。
            LidarReturnWindow centroidWindow{
                config.startTimeSeconds +
                    static_cast<double>(firstIndex) *
                    config.binWidthSeconds,

                config.startTimeSeconds +
                    static_cast<double>(lastIndex + 1) *
                    config.binWidthSeconds
            };

            if (!std::isfinite(centroidWindow.startTimeSeconds) ||
                !std::isfinite(centroidWindow.endTimeSeconds) ||
                centroidWindow.endTimeSeconds <=
                centroidWindow.startTimeSeconds)
            {
                throw std::domain_error(
                    "Peak neighborhood time window is not representable.");
            }

            // 局部窗口不能越过调用方指定的搜索范围。
            if (searchWindow)
            {
                centroidWindow.startTimeSeconds =
                    std::max(
                        centroidWindow.startTimeSeconds,
                        searchWindow->startTimeSeconds);

                centroidWindow.endTimeSeconds =
                    std::min(
                        centroidWindow.endTimeSeconds,
                        searchWindow->endTimeSeconds);
            }

            auto result = EstimateCentroid(
                waveform,
                emissionTimeSeconds,
                centroidWindow);

            if (!result)
                return std::nullopt;

            // 保留整个检出搜索区域的越界诊断。
            // 避免局部窗口把搜索时发现的削顶隐藏掉。
            result->searchHasOutOfRangeSamples =
                peak->searchHasOutOfRangeSamples;

            return result;
        }

private:
    double baselineCode;
    double minimumPeakAboveBaselineCodes;
};
