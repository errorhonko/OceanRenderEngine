#pragma once

#include "LidarIntegrator.h"
#include "LidarPulseProfile.h"
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

struct LidarWaveformConfig
{
    // 波形记录窗口的起始时刻。
    double startTimeSeconds = 0.0;

    // 每个时间箱的宽度。
    double binWidthSeconds = 1.0e-9;

    std::size_t binCount = 0;
};

class LidarWaveform
{
public:

    explicit LidarWaveform(
        const LidarWaveformConfig& config)
        : config(config),
        energyBinsJ(config.binCount, 0.0)
    {
        if (!std::isfinite(config.startTimeSeconds))
        {
            throw std::invalid_argument(
                "Waveform start time must be finite.");
        }

        if (!std::isfinite(config.binWidthSeconds) ||
            config.binWidthSeconds <= 0.0)
        {
            throw std::invalid_argument(
                "Waveform bin width must be positive.");
        }

        if (config.binCount == 0)
        {
            throw std::invalid_argument(
                "Waveform bin count must be positive.");
        }

        const double duration =
            config.binWidthSeconds *
            static_cast<double>(config.binCount);

        if (!std::isfinite(duration))
        {
            throw std::invalid_argument(
                "Waveform duration is invalid.");
        }
    }

    double AccumulatePulse(
        const LidarPulseResult& pulse)
    {
        LidarWaveform stagedWaveform(config);

        double recordedEnergyJ = 0.0;

        for (const auto& sample : pulse.returns)
        {
            const double sampleEnergyJ =
                stagedWaveform.AccumulateReturn(sample);

            const double newRecordedEnergyJ =
                recordedEnergyJ +
                sampleEnergyJ;

            if (!std::isfinite(newRecordedEnergyJ))
            {
                throw std::overflow_error(
                    "Pulse waveform energy overflow.");
            }

            recordedEnergyJ =
                newRecordedEnergyJ;
        }

        for (std::size_t i = 0;
            i < energyBinsJ.size();
            ++i)
        {
            const double mergedEnergyJ =
                energyBinsJ[i] +
                stagedWaveform.energyBinsJ[i];

            if (!std::isfinite(mergedEnergyJ))
            {
                throw std::overflow_error(
                    "Waveform merge energy overflow.");
            }
        }

        for (std::size_t i = 0;
            i < energyBinsJ.size();
            ++i)
        {
            energyBinsJ[i] +=
                stagedWaveform.energyBinsJ[i];
        }

        return recordedEnergyJ;
    }

    double AccumulatePulse(
        const LidarPulseResult& pulse,
        const LidarPulseProfile& profile)
    {
        LidarWaveform stagedWaveform(config);

        double recordedEnergyJ = 0.0;

        for (const auto& sample : pulse.returns)
        {
            const double sampleEnergyJ =
                stagedWaveform.AccumulateReturn(
                    sample,
                    profile);

            const double newRecordedEnergyJ =
                recordedEnergyJ +
                sampleEnergyJ;

            if (!std::isfinite(newRecordedEnergyJ))
            {
                throw std::overflow_error(
                    "Pulse waveform energy overflow.");
            }

            recordedEnergyJ =
                newRecordedEnergyJ;
        }

        for (std::size_t i = 0;
            i < energyBinsJ.size();
            ++i)
        {
            const double mergedEnergyJ =
                energyBinsJ[i] +
                stagedWaveform.energyBinsJ[i];

            if (!std::isfinite(mergedEnergyJ))
            {
                throw std::overflow_error(
                    "Waveform merge energy overflow.");
            }
        }

        for (std::size_t i = 0;
            i < energyBinsJ.size();
            ++i)
        {
            energyBinsJ[i] +=
                stagedWaveform.energyBinsJ[i];
        }

        return recordedEnergyJ;
    }

    double AccumulateReturn(
        const LidarReturnSample& sample)
    {
        if (!std::isfinite(sample.arrivalTimeSeconds) ||
            !std::isfinite(sample.receivedEnergyJ) ||
            sample.receivedEnergyJ < 0.0)
        {
            throw std::invalid_argument(
                "Waveform return sample is invalid.");
        }

        const double relativeTime =
            sample.arrivalTimeSeconds -
            config.startTimeSeconds;

        const double duration =
            config.binWidthSeconds *
            static_cast<double>(config.binCount);

        // 记录窗口采用 [start, start + duration)。
        if (relativeTime < 0.0 ||
            relativeTime >= duration)
        {
            return 0.0;
        }

        const std::size_t binIndex =
            static_cast<std::size_t>(
                relativeTime /
                config.binWidthSeconds);

        const double newEnergy =
            energyBinsJ[binIndex] +
            sample.receivedEnergyJ;

        if (!std::isfinite(newEnergy))
        {
            throw std::overflow_error(
                "Waveform energy overflow.");
        }

        energyBinsJ[binIndex] = newEnergy;

        return sample.receivedEnergyJ;
    }

    void Clear()
    {
        for (double& energy : energyBinsJ)
            energy = 0.0;
    }

    const std::vector<double>& EnergyBinsJ() const
    {
        return energyBinsJ;
    }

    double BinCenterTimeSeconds(
        std::size_t index) const
    {
        if (index >= energyBinsJ.size())
            throw std::out_of_range(
                "Waveform bin index is out of range.");

        return config.startTimeSeconds +
            (static_cast<double>(index) + 0.5) *
            config.binWidthSeconds;
    }

    double TotalEnergyJ() const
    {
        double total = 0.0;

        for (double energy : energyBinsJ)
            total += energy;

        return total;
    }

    double AccumulateReturn(
        const LidarReturnSample& sample,
        const LidarPulseProfile& profile)
    {
        if (!std::isfinite(sample.arrivalTimeSeconds) ||
            !std::isfinite(sample.receivedEnergyJ) ||
            sample.receivedEnergyJ < 0.0)
        {
            throw std::invalid_argument(
                "Waveform return sample is invalid.");
        }

        if (sample.receivedEnergyJ == 0.0)
            return 0.0;

        // 先计算全部贡献，通过所有验证后再写入。
        // 这样中途抛异常时，原波形不会只更新一半。
        std::vector<double> contributions(
            energyBinsJ.size(),
            0.0);

        double totalFraction = 0.0;
        double accumulatedEnergyJ = 0.0;

        // 在回波相对时间中构造箱边界，避免先加大时刻再相减。
        // 相邻箱复用同一个表达式生成公共边界，避免舍入造成重叠。
        const double recordStartRelativeToReturn =
            config.startTimeSeconds - sample.arrivalTimeSeconds;

        for (std::size_t i = 0;
            i < energyBinsJ.size();
            ++i)
        {
            // 转换成相对于回波中心的时间。
            const double relativeStart =
                recordStartRelativeToReturn +
                static_cast<double>(i) * config.binWidthSeconds;

            const double relativeEnd =
                recordStartRelativeToReturn +
                static_cast<double>(i + 1) * config.binWidthSeconds;

            const double fraction =
                profile.FractionBetween(
                    relativeStart,
                    relativeEnd);

            // LidarPulseProfile 的接口约定。
            if (!std::isfinite(fraction) ||
                fraction < 0.0 ||
                fraction > 1.0)
            {
                throw std::domain_error(
                    "Pulse profile returned an invalid fraction.");
            }

            const double newTotalFraction =
                totalFraction + fraction;

            if (!std::isfinite(newTotalFraction) ||
                newTotalFraction > 1.0 + 1.0e-12)
            {
                throw std::domain_error(
                    "Pulse profile is not normalized.");
            }

            totalFraction = newTotalFraction;

            const double contributionJ =
                sample.receivedEnergyJ *
                fraction;

            const double newBinEnergyJ =
                energyBinsJ[i] +
                contributionJ;

            if (!std::isfinite(contributionJ) ||
                !std::isfinite(newBinEnergyJ))
            {
                throw std::overflow_error(
                    "Waveform energy overflow.");
            }

            contributions[i] = contributionJ;

            const double newAccumulatedEnergyJ =
                accumulatedEnergyJ +
                contributionJ;

            if (!std::isfinite(newAccumulatedEnergyJ))
            {
                throw std::overflow_error(
                    "Accumulated waveform energy overflow.");
            }

            accumulatedEnergyJ =
                newAccumulatedEnergyJ;
        }

        // 所有计算和验证成功之后才真正修改波形。
        for (std::size_t i = 0;
            i < energyBinsJ.size();
            ++i)
        {
            energyBinsJ[i] += contributions[i];
        }

        return accumulatedEnergyJ;
    }

    const LidarWaveformConfig& Config() const
    {
        return config;
    }

private:
    LidarWaveformConfig config;
    std::vector<double> energyBinsJ;
};
