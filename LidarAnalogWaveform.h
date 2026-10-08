#pragma once

#include "LidarWaveform.h"
#include "LinearPhotodetector.h"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

// 保存时间箱内的平均电流，单位 A。
// 不是瞬时电流采样，也不是电压或 ADC 数值。
class LidarAnalogWaveform
{
public:
    LidarAnalogWaveform(
        const LidarWaveformConfig& config,
        std::vector<double> averageCurrentBinsA)
        : config(config),
        averageCurrentBinsA(
            std::move(averageCurrentBinsA))
    {
        if (!std::isfinite(config.startTimeSeconds) ||
            !std::isfinite(config.binWidthSeconds) ||
            config.binWidthSeconds <= 0.0 ||
            config.binCount == 0 ||
            this->averageCurrentBinsA.size() != config.binCount)
        {
            throw std::invalid_argument(
                "Invalid analog waveform configuration.");
        }

        const double duration =
            config.binWidthSeconds *
            static_cast<double>(config.binCount);

        if (!std::isfinite(duration) ||
            !std::isfinite(config.startTimeSeconds + duration))
        {
            throw std::invalid_argument(
                "Analog waveform time window overflow.");
        }

        for (double currentA : this->averageCurrentBinsA)
        {
            // 容器允许有符号电信号，为后续背景扣除等处理预留。
            if (!std::isfinite(currentA))
            {
                throw std::invalid_argument(
                    "Analog current must be finite.");
            }
        }
    }

    static LidarAnalogWaveform FromOpticalWaveform(
        const LidarWaveform& opticalWaveform,
        const LinearPhotodetector& detector)
    {
        const auto& config = opticalWaveform.Config();
        const auto& energies = opticalWaveform.EnergyBinsJ();

        std::vector<double> currents;
        currents.reserve(energies.size());

        for (double energyJ : energies)
        {
            currents.push_back(
                detector.EnergyToAverageCurrentA(
                    energyJ,
                    config.binWidthSeconds));
        }

        return LidarAnalogWaveform(
            config,
            std::move(currents));
    }

    const std::vector<double>& AverageCurrentBinsA() const
    {
        return averageCurrentBinsA;
    }

    const LidarWaveformConfig& Config() const
    {
        return config;
    }

    double BinCenterTimeSeconds(std::size_t index) const
    {
        if (index >= averageCurrentBinsA.size())
        {
            throw std::out_of_range(
                "Analog waveform bin index out of range.");
        }

        return config.startTimeSeconds +
            (static_cast<double>(index) + 0.5) *
            config.binWidthSeconds;
    }

    double TotalChargeC() const
    {
        double totalChargeC = 0.0;

        for (double currentA : averageCurrentBinsA)
        {
            const double chargeC =
                currentA * config.binWidthSeconds;

            const double nextTotal =
                totalChargeC + chargeC;

            if (!std::isfinite(chargeC) ||
                !std::isfinite(nextTotal))
            {
                throw std::overflow_error(
                    "Analog waveform charge overflow.");
            }

            totalChargeC = nextTotal;
        }

        return totalChargeC;
    }

private:
    LidarWaveformConfig config;
    std::vector<double> averageCurrentBinsA;
};