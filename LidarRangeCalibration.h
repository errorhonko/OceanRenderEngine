#pragma once

#include "LidarDigitalRangeEstimator.h"

#include <cmath>
#include <optional>
#include <stdexcept>

struct LidarCalibratedRangeEstimate
{
    // 保留原始时间、距离、峰值和越界诊断。
    LidarDigitalRangeEstimate measured;

    double correctedTimeOfFlightSeconds = 0.0;
    double correctedRangeMeters = 0.0;
};

class LidarRangeCalibration
{
public:
    explicit LidarRangeCalibration(
        double systemDelaySeconds = 0.0)
        : systemDelaySeconds(systemDelaySeconds)
    {
        if (!std::isfinite(systemDelaySeconds) ||
            systemDelaySeconds < 0.0)
        {
            throw std::invalid_argument(
                "System delay must be finite and nonnegative.");
        }
    }
    // 单站系统、单个已知距离目标的固定延迟标定。
// reference 必须是未经校正的测距结果。
    static LidarRangeCalibration FromReferenceMeasurement(
        const LidarDigitalRangeEstimate& reference,
        double referenceRangeMeters)
    {
        if (!std::isfinite(referenceRangeMeters) ||
            referenceRangeMeters < 0.0)
        {
            throw std::invalid_argument(
                "Reference range must be finite and nonnegative.");
        }

        if (!std::isfinite(reference.arrivalTimeSeconds) ||
            !std::isfinite(reference.timeOfFlightSeconds) ||
            reference.timeOfFlightSeconds < 0.0 ||
            !std::isfinite(reference.rangeMeters) ||
            reference.rangeMeters < 0.0)
        {
            throw std::invalid_argument(
                "Invalid reference measurement.");
        }

        // 基线标定暂不接受有效搜索区域内发生越界的记录。
        if (reference.peakOutOfRange ||
            reference.searchHasOutOfRangeSamples)
        {
            throw std::invalid_argument(
                "Clipped reference waveform cannot be used for calibration.");
        }

        constexpr double speedOfLight = 299792458.0;

        // 等价于 2*R/c，避免先乘 2 导致中间值溢出。
        const double expectedTimeOfFlight =
            referenceRangeMeters / (0.5 * speedOfLight);

        const double delay =
            reference.timeOfFlightSeconds -
            expectedTimeOfFlight;

        // 当前类只支持非负固定延迟，不是通用有符号偏差校正。
        if (delay < 0.0)
        {
            throw std::domain_error(
                "Reference measurement implies a negative delay.");
        }

        return LidarRangeCalibration(delay);
    }
    // 输入必须是未校正的测距结果。
    std::optional<LidarCalibratedRangeEstimate> Apply(
        const LidarDigitalRangeEstimate& measured) const
    {
        if (!std::isfinite(measured.arrivalTimeSeconds) ||
            !std::isfinite(measured.timeOfFlightSeconds) ||
            measured.timeOfFlightSeconds < 0.0 ||
            !std::isfinite(measured.rangeMeters) ||
            measured.rangeMeters < 0.0)
        {
            throw std::invalid_argument(
                "Invalid measured range estimate.");
        }

        const double correctedTimeOfFlight =
            measured.timeOfFlightSeconds -
            systemDelaySeconds;

        // 校正后出现负传播时间，不能作为有效距离。
        // 不直接截成零，否则会掩盖标定或检出问题。
        if (correctedTimeOfFlight < 0.0)
            return std::nullopt;

        constexpr double speedOfLight = 299792458.0;

        const double correctedRange =
            (0.5 * speedOfLight) *
            correctedTimeOfFlight;

        if (!std::isfinite(correctedTimeOfFlight) ||
            !std::isfinite(correctedRange))
        {
            throw std::overflow_error(
                "Calibrated range overflow.");
        }

        return LidarCalibratedRangeEstimate{
            measured,
            correctedTimeOfFlight,
            correctedRange
        };
    }

    double SystemDelaySeconds() const
    {
        return systemDelaySeconds;
    }

private:
    double systemDelaySeconds;
};