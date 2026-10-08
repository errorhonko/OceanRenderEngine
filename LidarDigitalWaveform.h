#pragma once

#include "IdealUniformAdc.h"
#include "LidarVoltageWaveform.h"

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

class LidarDigitalWaveform
{
public:
    static LidarDigitalWaveform FromVoltageWaveform(
        const LidarVoltageWaveform& voltageWaveform,
        const IdealUniformAdc& adc)
    {
        const auto& voltages =
            voltageWaveform.AverageVoltageBinsV();

        std::vector<LidarAdcSample> samples;
        samples.reserve(voltages.size());

        for (double voltageV : voltages)
        {
            samples.push_back(adc.Quantize(voltageV));
        }

        return LidarDigitalWaveform(
            voltageWaveform.Config(),
            adc,
            std::move(samples));
    }

    const std::vector<LidarAdcSample>& Samples() const
    {
        return samples;
    }

    const LidarWaveformConfig& Config() const
    {
        return config;
    }

    const IdealUniformAdc& Adc() const
    {
        return adc;
    }

    double BinCenterTimeSeconds(std::size_t index) const
    {
        CheckIndex(index);

        return config.startTimeSeconds +
            (static_cast<double>(index) + 0.5) *
            config.binWidthSeconds;
    }

    // 数字码对应量化区间的中心，不是原始电压。
    double ReconstructedVoltageV(std::size_t index) const
    {
        CheckIndex(index);

        return adc.CodeCenterVoltageV(
            samples[index].code);
    }

    bool HasOutOfRangeSamples() const
    {
        for (const auto& sample : samples)
        {
            if (sample.belowRange || sample.aboveRange)
                return true;
        }

        return false;
    }

private:
    // 只能由 FromVoltageWaveform 创建：
    // 时间配置来自已验证的电压波形；
    // 数字码和标志来自已验证的量化器。
    LidarDigitalWaveform(
        const LidarWaveformConfig& config,
        const IdealUniformAdc& adc,
        std::vector<LidarAdcSample> samples)
        : config(config),
        adc(adc),
        samples(std::move(samples))
    {
    }

    void CheckIndex(std::size_t index) const
    {
        if (index >= samples.size())
        {
            throw std::out_of_range(
                "Digital waveform bin index out of range.");
        }
    }

private:
    LidarWaveformConfig config;
    IdealUniformAdc adc;
    std::vector<LidarAdcSample> samples;
};