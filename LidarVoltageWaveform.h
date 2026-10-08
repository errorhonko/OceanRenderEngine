#pragma once

#include "LidarAnalogWaveform.h"
#include "IdealTransimpedanceAmplifier.h"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

class LidarVoltageWaveform
{
public:
    LidarVoltageWaveform(
        const LidarWaveformConfig& config,
        std::vector<double> averageVoltageBinsV)
        : config(config),
        averageVoltageBinsV(
            std::move(averageVoltageBinsV))
    {
        if (!std::isfinite(config.startTimeSeconds) ||
            !std::isfinite(config.binWidthSeconds) ||
            config.binWidthSeconds <= 0.0 ||
            config.binCount == 0 ||
            this->averageVoltageBinsV.size() != config.binCount)
        {
            throw std::invalid_argument(
                "Invalid voltage waveform configuration.");
        }

        const double duration =
            config.binWidthSeconds *
            static_cast<double>(config.binCount);

        if (!std::isfinite(duration) ||
            !std::isfinite(config.startTimeSeconds + duration))
        {
            throw std::invalid_argument(
                "Voltage waveform time window overflow.");
        }

        for (double voltageV : this->averageVoltageBinsV)
        {
            // 允许有符号电压，包括反相输出。
            if (!std::isfinite(voltageV))
            {
                throw std::invalid_argument(
                    "Voltage must be finite.");
            }
        }
    }

    static LidarVoltageWaveform FromCurrentWaveform(
        const LidarAnalogWaveform& currentWaveform,
        const IdealTransimpedanceAmplifier& amplifier)
    {
        const auto& currents =
            currentWaveform.AverageCurrentBinsA();

        std::vector<double> voltages;
        voltages.reserve(currents.size());

        for (double currentA : currents)
        {
            voltages.push_back(
                amplifier.CurrentToVoltageV(currentA));
        }

        return LidarVoltageWaveform(
            currentWaveform.Config(),
            std::move(voltages));
    }

    const std::vector<double>& AverageVoltageBinsV() const
    {
        return averageVoltageBinsV;
    }

    const LidarWaveformConfig& Config() const
    {
        return config;
    }

    double BinCenterTimeSeconds(std::size_t index) const
    {
        if (index >= averageVoltageBinsV.size())
        {
            throw std::out_of_range(
                "Voltage waveform bin index out of range.");
        }

        return config.startTimeSeconds +
            (static_cast<double>(index) + 0.5) *
            config.binWidthSeconds;
    }

private:
    LidarWaveformConfig config;
    std::vector<double> averageVoltageBinsV;
};