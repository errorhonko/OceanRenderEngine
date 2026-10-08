#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

struct LidarAdcSample
{
    std::uint32_t code = 0;

    // 仅在输入严格超出量程时置位。
    bool belowRange = false;
    bool aboveRange = false;
};

class IdealUniformAdc
{
public:
    IdealUniformAdc(
        int bitCount,
        double minimumVoltageV,
        double maximumVoltageV)
        : bitCount(bitCount),
        minimumVoltageV(minimumVoltageV),
        maximumVoltageV(maximumVoltageV)
    {
        // 当前实现限制为 1~24 位。
        if (bitCount < 1 || bitCount > 24)
        {
            throw std::invalid_argument(
                "ADC bit count must be between 1 and 24.");
        }

        if (!std::isfinite(minimumVoltageV) ||
            !std::isfinite(maximumVoltageV) ||
            maximumVoltageV <= minimumVoltageV)
        {
            throw std::invalid_argument(
                "Invalid ADC voltage range.");
        }

        voltageSpanV =
            maximumVoltageV - minimumVoltageV;

        levelCount =
            std::uint32_t{ 1 } << bitCount;

        lsbVoltageV =
            voltageSpanV / double(levelCount);

        if (!std::isfinite(voltageSpanV) ||
            !std::isfinite(lsbVoltageV) ||
            lsbVoltageV <= 0.0)
        {
            throw std::invalid_argument(
                "ADC voltage range is not representable.");
        }
    }

    LidarAdcSample Quantize(double voltageV) const
    {
        if (!std::isfinite(voltageV))
        {
            throw std::invalid_argument(
                "ADC input voltage must be finite.");
        }

        if (voltageV <= minimumVoltageV)
        {
            return {
                0,
                voltageV < minimumVoltageV,
                false
            };
        }

        if (voltageV >= maximumVoltageV)
        {
            return {
                MaximumCode(),
                false,
                voltageV > maximumVoltageV
            };
        }

        const double position =
            (voltageV - minimumVoltageV) /
            voltageSpanV;

        const double codeValue =
            std::min(
                std::floor(position * double(levelCount)),
                double(MaximumCode()));

        return {
            static_cast<std::uint32_t>(codeValue),
            false,
            false
        };
    }

    double CodeCenterVoltageV(std::uint32_t code) const
    {
        if (code > MaximumCode())
        {
            throw std::out_of_range(
                "ADC code is out of range.");
        }

        const double voltageV =
            minimumVoltageV +
            (double(code) + 0.5) * lsbVoltageV;

        if (!std::isfinite(voltageV))
        {
            throw std::overflow_error(
                "ADC reconstructed voltage overflow.");
        }

        return voltageV;
    }

    double LsbVoltageV() const
    {
        return lsbVoltageV;
    }

    int BitCount() const
    {
        return bitCount;
    }

    std::uint32_t MaximumCode() const
    {
        return levelCount - 1;
    }

private:
    int bitCount;
    double minimumVoltageV;
    double maximumVoltageV;
    double voltageSpanV;
    double lsbVoltageV;
    std::uint32_t levelCount;
};