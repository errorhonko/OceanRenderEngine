#pragma once

#include "LidarAnalogWaveform.h"

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

// 单位直流增益的一阶因果有效电流响应。
// 每箱输入按常量处理，输出仍是箱内平均电流。
// 不含放大器、噪声或 ADC。
class FirstOrderLidarResponse
{
public:
    explicit FirstOrderLidarResponse(
        double timeConstantSeconds)
        : timeConstantSeconds(timeConstantSeconds)
    {
        if (!std::isfinite(timeConstantSeconds) ||
            timeConstantSeconds < 0.0)
        {
            throw std::invalid_argument(
                "Response time constant must be finite "
                "and nonnegative.");
        }
    }

    LidarAnalogWaveform Apply(
        const LidarAnalogWaveform& input) const
    {
        // tau=0 定义为理想直通。
        if (timeConstantSeconds == 0.0)
            return input;

        const auto& config = input.Config();
        const auto& currents = input.AverageCurrentBinsA();

        const double ratio =
            config.binWidthSeconds / timeConstantSeconds;

        const double decay = std::exp(-ratio);

        // 避免 ratio 很小时，1-exp(-ratio) 的相减损失精度。
        const double stateInputWeight =
            -std::expm1(-ratio);

        double averageInputWeight;

        if (ratio < 1.0e-4)
        {
            // 1-(1-exp(-r))/r 的小 r 展开。
            // 直接算 1-b 会损失小量的精度。
            averageInputWeight =
                ratio * (
                    0.5 +
                    ratio * (
                        -1.0 / 6.0 +
                        ratio * (
                            1.0 / 24.0 -
                            ratio / 120.0)));
        }
        else
        {
            averageInputWeight =
                1.0 - stateInputWeight / ratio;
        }

        const double averageStateWeight =
            1.0 - averageInputWeight;

        std::vector<double> output;
        output.reserve(currents.size());

        // 本版假定记录窗口开始时，接收器处于零状态。
        double state = 0.0;

        for (double inputCurrent : currents)
        {
            const double averageOutput =
                averageStateWeight * state +
                averageInputWeight * inputCurrent;

            const double nextState =
                decay * state +
                stateInputWeight * inputCurrent;

            if (!std::isfinite(averageOutput) ||
                !std::isfinite(nextState))
            {
                throw std::overflow_error(
                    "Receiver response overflow.");
            }

            output.push_back(averageOutput);
            state = nextState;
        }

        return LidarAnalogWaveform(
            config,
            std::move(output));
    }

    double TimeConstantSeconds() const
    {
        return timeConstantSeconds;
    }

private:
    double timeConstantSeconds;
};