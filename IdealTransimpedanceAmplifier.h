#pragma once

#include <cmath>
#include <stdexcept>

// 理想跨阻读出：V = Zt * I。
// 仅做增益与极性转换，不含带宽、噪声、偏置及饱和。
class IdealTransimpedanceAmplifier
{
public:
    explicit IdealTransimpedanceAmplifier(
        double gainVoltsPerAmp)
        : gainVoltsPerAmp(gainVoltsPerAmp)
    {
        if (!std::isfinite(gainVoltsPerAmp) ||
            gainVoltsPerAmp == 0.0)
        {
            throw std::invalid_argument(
                "Transimpedance gain must be finite "
                "and nonzero.");
        }
    }

    double CurrentToVoltageV(double currentA) const
    {
        // 有符号电流允许通过。
        if (!std::isfinite(currentA))
        {
            throw std::invalid_argument(
                "Input current must be finite.");
        }

        const double voltageV =
            gainVoltsPerAmp * currentA;

        if (!std::isfinite(voltageV))
        {
            throw std::overflow_error(
                "Amplifier voltage overflow.");
        }

        return voltageV;
    }

    double GainVoltsPerAmp() const
    {
        return gainVoltsPerAmp;
    }

private:
    double gainVoltsPerAmp;
};