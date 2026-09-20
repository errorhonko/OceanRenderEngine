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

    void Accumulate(
        const LidarPulseResult& pulse)
    {
        for (const auto& sample : pulse.returns)
            Accumulate(sample);
    }

    bool Accumulate(
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
            return false;
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

        return true;
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

    double Accumulate(
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

        for (std::size_t i = 0;
            i < energyBinsJ.size();
            ++i)
        {
            const double binStartTime =
                config.startTimeSeconds +
                static_cast<double>(i) *
                config.binWidthSeconds;

            const double binEndTime =
                binStartTime +
                config.binWidthSeconds;

            // 转换成相对于回波中心的时间。
            const double relativeStart =
                binStartTime -
                sample.arrivalTimeSeconds;

            const double relativeEnd =
                binEndTime -
                sample.arrivalTimeSeconds;

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

private:
    LidarWaveformConfig config;
    std::vector<double> energyBinsJ;
};
