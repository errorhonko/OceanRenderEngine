#pragma once

#include "LidarDigitalWaveform.h"

// 暂时复用这里定义的 LidarReturnWindow。
#include "LidarDigitalRangeEstimator.h"

#include <cmath>
#include <cstddef>
#include <stdexcept>

struct LidarDigitalBaselineEstimate
{
    // 单位：数字码。允许小数。
    double meanCode = 0.0;

    // 样本标准差，单位：数字码。
    double sigmaCodes = 0.0;

    std::size_t sampleCount = 0;
};

class LidarDigitalBaselineEstimator
{
public:
    static LidarDigitalBaselineEstimate Estimate(
        const LidarDigitalWaveform& waveform,
        const LidarReturnWindow& noiseWindow)
    {
        if (!std::isfinite(noiseWindow.startTimeSeconds) ||
            !std::isfinite(noiseWindow.endTimeSeconds) ||
            noiseWindow.endTimeSeconds <=
            noiseWindow.startTimeSeconds)
        {
            throw std::invalid_argument(
                "Noise window must be finite and have positive duration.");
        }

        const auto& samples = waveform.Samples();

        std::size_t count = 0;
        double mean = 0.0;
        double m2 = 0.0;

        for (std::size_t i = 0; i < samples.size(); ++i)
        {
            const double time =
                waveform.BinCenterTimeSeconds(i);

            if (!std::isfinite(time))
            {
                throw std::domain_error(
                    "Digital waveform time must be finite.");
            }

            // 按箱中心判断，左闭右开。
            if (time < noiseWindow.startTimeSeconds ||
                time >= noiseWindow.endTimeSeconds)
            {
                continue;
            }

            const auto& sample = samples[i];

            // 不跳过削顶箱：跳过会改变统计分布。
            // 第一版直接拒绝受削顶污染的窗口。
            if (sample.belowRange || sample.aboveRange)
            {
                throw std::domain_error(
                    "Noise window contains out-of-range ADC samples.");
            }

            const double code =
                static_cast<double>(sample.code);

            ++count;

            // Welford 在线均值和离差平方和。
            const double delta = code - mean;

            mean += delta / static_cast<double>(count);

            const double deltaAfterUpdate = code - mean;

            m2 += delta * deltaAfterUpdate;
        }

        if (count < 2)
        {
            throw std::invalid_argument(
                "Noise window must contain at least two bin centers.");
        }

        const double variance =
            m2 / static_cast<double>(count - 1);

        const double sigma = std::sqrt(variance);

        if (!std::isfinite(mean) ||
            !std::isfinite(sigma))
        {
            throw std::overflow_error(
                "Baseline statistics are not representable.");
        }

        return LidarDigitalBaselineEstimate{
            mean,
            sigma,
            count
        };
    }
};