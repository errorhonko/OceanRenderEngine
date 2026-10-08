#pragma once


#include "LidarWaveform.h"

#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>

struct LidarRangeEstimate
{
    std::size_t peakBinIndex = 0;

    // 波形时间坐标中的估计到达时刻。
    double arrivalTimeSeconds = 0.0;

    // 相对于发射时刻的估计传播时间。
    double timeOfFlightSeconds = 0.0;

    // 单站系统的估计距离。
    double rangeMeters = 0.0;

    // 峰值箱内能量，不是峰值功率。
    double peakBinEnergyJ = 0.0;
};

class LidarRangeEstimator
{
public:
    explicit LidarRangeEstimator(
        double minimumPeakBinEnergyJ = 0.0)
        : minimumPeakBinEnergyJ(minimumPeakBinEnergyJ)
    {
        if (!std::isfinite(minimumPeakBinEnergyJ) ||
            minimumPeakBinEnergyJ < 0.0)
        {
            throw std::invalid_argument(
                "Peak energy threshold must be finite and nonnegative.");
        }
    }

    // 基线限制：
    // 单站、单峰、直接探测的理想能量波形。
    // 不包含系统延迟校正、噪声模型或亚时间箱插值。
    std::optional<LidarRangeEstimate> Estimate(
        const LidarWaveform& waveform,
        double emissionTimeSeconds) const
    {
        if (!std::isfinite(emissionTimeSeconds) ||
            emissionTimeSeconds < 0.0)
        {
            throw std::invalid_argument(
                "Emission time must be finite and nonnegative.");
        }

        const auto& bins = waveform.EnergyBinsJ();

        std::optional<std::size_t> peakIndex;
        double peakEnergyJ = 0.0;

        for (std::size_t i = 0; i < bins.size(); ++i)
        {
            const double energyJ = bins[i];

            if (!std::isfinite(energyJ) || energyJ < 0.0)
            {
                throw std::domain_error(
                    "Waveform energy must be finite and nonnegative.");
            }

            const double binTime =
                waveform.BinCenterTimeSeconds(i);

            if (!std::isfinite(binTime))
            {
                throw std::domain_error(
                    "Waveform bin time must be finite.");
            }

            // 只搜索箱中心不早于本次发射的区域。
            if (binTime < emissionTimeSeconds)
                continue;

            // 严格大于：等高峰时保留较早的箱。
            if (energyJ > peakEnergyJ)
            {
                peakEnergyJ = energyJ;
                peakIndex = i;
            }
        }

        // 全零波形，或峰值未超过检测门限。
        if (!peakIndex ||
            peakEnergyJ <= minimumPeakBinEnergyJ)
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
                "Range estimate overflow.");
        }

        return LidarRangeEstimate{
            *peakIndex,
            arrivalTime,
            timeOfFlight,
            range,
            peakEnergyJ
        };
    }

    // 无背景、单个回波分量的能量重心基线。
// 只使用箱中心不早于发射时刻的部分。
    std::optional<LidarRangeEstimate> EstimateCentroid(
        const LidarWaveform& waveform,
        double emissionTimeSeconds) const
    {
        // 复用峰值法的输入验证和检出门限。
        auto result = Estimate(
            waveform,
            emissionTimeSeconds);

        if (!result)
            return std::nullopt;

        const auto& bins = waveform.EnergyBinsJ();

        double totalWeight = 0.0;
        double meanTimeOfFlight = 0.0;

        for (std::size_t i = 0; i < bins.size(); ++i)
        {
            const double binTime =
                waveform.BinCenterTimeSeconds(i);

            if (binTime < emissionTimeSeconds ||
                bins[i] == 0.0)
            {
                continue;
            }

            const double timeOfFlight =
                binTime - emissionTimeSeconds;

            if (!std::isfinite(timeOfFlight))
            {
                throw std::overflow_error(
                    "Centroid time overflow.");
            }

            // 全部除以同一个峰值，不改变加权平均，
            // 并避免直接累加很大或很小的能量。
            const double weight =
                bins[i] / result->peakBinEnergyJ;

            if (weight == 0.0)
                continue;
            const double newTotalWeight =
                totalWeight + weight;

            if (!std::isfinite(newTotalWeight))
            {
                throw std::overflow_error(
                    "Centroid weight overflow.");
            }

            // 增量加权平均，等价于 sum(weight*time)/sum(weight)。
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

        if (!std::isfinite(arrivalTime) ||
            !std::isfinite(range))
        {
            throw std::overflow_error(
                "Centroid range overflow.");
        }

        result->arrivalTimeSeconds = arrivalTime;
        result->timeOfFlightSeconds = meanTimeOfFlight;
        result->rangeMeters = range;

        return result;
    }

private:
    double minimumPeakBinEnergyJ = 0.0;
};