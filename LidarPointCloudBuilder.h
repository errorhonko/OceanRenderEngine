#pragma once

#include "LidarPointSample.h"
#include "LidarPulseMeasurement.h"
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

// 单站、发射接收共址：p = origin + range * centerDirection。
// 共址关系、世界坐标系及元数据/波形对应关系由上层保证。
// 本类不追踪几何、不校正平台运动、不分离多回波，也不重新标定。
class LidarPointCloudBuilder
{
public:
    // 必须显式选择距离来源；缺少校正结果时不偷偷退回原始距离。
    explicit LidarPointCloudBuilder(
        LidarPointRangeSource rangeSource,
        bool rejectClippedReturns = true)
        : rangeSource(rangeSource), rejectClippedReturns(rejectClippedReturns)
    {
        if (rangeSource != LidarPointRangeSource::Raw &&
            rangeSource != LidarPointRangeSource::Corrected)
            throw std::invalid_argument("Unknown point range source.");
    }

    std::optional<LidarPointSample> BuildPoint(
        const LidarPulseMeasurement& measurement) const
    {
        const auto& reception = measurement.reception;
        if (!reception.measured)
            return std::nullopt;

        const LidarDigitalRangeEstimate* raw = &*reception.measured;
        double range = raw->rangeMeters;
        if (rangeSource == LidarPointRangeSource::Corrected)
        {
            if (!reception.corrected)
                return std::nullopt;
            raw = &reception.corrected->measured;
            range = reception.corrected->correctedRangeMeters;
            if (!std::isfinite(reception.corrected->correctedTimeOfFlightSeconds) ||
                reception.corrected->correctedTimeOfFlightSeconds < 0.0)
                throw std::invalid_argument("Invalid corrected propagation time.");
        }
        if (!std::isfinite(range) || range < 0.0 ||
            !std::isfinite(raw->rangeMeters) || raw->rangeMeters < 0.0 ||
            !std::isfinite(raw->arrivalTimeSeconds) || raw->arrivalTimeSeconds < 0.0 ||
            !std::isfinite(raw->timeOfFlightSeconds) || raw->timeOfFlightSeconds < 0.0 ||
            !std::isfinite(raw->peakAboveBaselineCodes) || raw->peakAboveBaselineCodes < 0.0)
            throw std::invalid_argument("Invalid point measurement.");

        // 搜索区削顶会影响定位；其他区域越界只作为诊断保留。
        if (rejectClippedReturns &&
            (raw->peakOutOfRange || raw->searchHasOutOfRangeSamples))
            return std::nullopt;

        const auto& origin = measurement.pulse.OriginWorld();
        const auto& direction = measurement.pulse.CenterDirectionWorld();
        const double x = double(origin.x) + range * double(direction.x);
        const double y = double(origin.y) + range * double(direction.y);
        const double z = double(origin.z) + range * double(direction.z);
        const double floatLimit = std::numeric_limits<float>::max();
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            std::abs(x) > floatLimit || std::abs(y) > floatLimit || std::abs(z) > floatLimit)
            throw std::overflow_error("Point position cannot be represented by Vector3f.");

        LidarPointSample point;
        point.positionWorld = Vector3f(float(x), float(y), float(z));
        point.centerDirectionWorld = direction;
        point.pulseId = measurement.pulse.PulseId();
        point.emissionTimeSeconds = measurement.pulse.EmissionTimeSeconds();
        point.arrivalTimeSeconds = raw->arrivalTimeSeconds;
        point.rangeMeters = range;
        point.rangeSource = rangeSource;
        point.peakCode = raw->peakCode;
        point.peakAboveBaselineCodes = raw->peakAboveBaselineCodes;
        point.peakOutOfRange = raw->peakOutOfRange;
        point.searchHasOutOfRangeSamples = raw->searchHasOutOfRangeSamples;
        point.hasAdcOutOfRangeSamples = reception.hasAdcOutOfRangeSamples;
        return point;
    }

    LidarPointRangeSource RangeSource() const { return rangeSource; }
    bool RejectClippedReturns() const { return rejectClippedReturns; }

private:
    LidarPointRangeSource rangeSource;
    bool rejectClippedReturns;
};
