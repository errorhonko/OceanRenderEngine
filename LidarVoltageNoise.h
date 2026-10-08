#pragma once
#pragma once

#include "LidarVoltageWaveform.h"

#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

class LidarVoltageNoise
{
public:
    explicit LidarVoltageNoise(double sigmaVoltageV)
        : sigmaVoltageV(sigmaVoltageV)
    {
        if (!std::isfinite(sigmaVoltageV) ||
            sigmaVoltageV < 0.0)
        {
            throw std::invalid_argument(
                "Voltage noise sigma must be finite and nonnegative.");
        }
    }

    // 相同输入和 seed，在相同随机库实现下复现同一结果。
    // 每次调用创建局部随机状态，不修改输入。
    LidarVoltageWaveform Apply(
        const LidarVoltageWaveform& input,
        std::uint64_t seed) const
    {
        // 零噪声直接复制，精确保留原始电压。
        if (sigmaVoltageV == 0.0)
            return input;

        std::mt19937_64 generator(seed);
        std::normal_distribution<double> standardNormal(
            0.0, 1.0);

        std::vector<double> voltages =
            input.AverageVoltageBinsV();

        for (double& voltageV : voltages)
        {
            const double noiseV =
                sigmaVoltageV * standardNormal(generator);

            const double noisyVoltageV =
                voltageV + noiseV;

            if (!std::isfinite(noiseV) ||
                !std::isfinite(noisyVoltageV))
            {
                throw std::overflow_error(
                    "Noisy voltage overflow.");
            }

            // 不截掉负电压，也不提前模拟 ADC 削顶。
            voltageV = noisyVoltageV;
        }

        return LidarVoltageWaveform(
            input.Config(),
            std::move(voltages));
    }

    double SigmaVoltageV() const
    {
        return sigmaVoltageV;
    }

private:
    double sigmaVoltageV;
};