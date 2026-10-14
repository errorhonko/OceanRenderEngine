#pragma once

#include "lidar/output/LidarPointSample.h"
#include "lidar/simulation/LidarPulseMeasurement.h"
#include "lidar/signal_processing/LidarReturnSelection.h"
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

// 单站、发射接收共址：p = origin + range * centerDirection。
// 共址关系、世界坐标系及元数据/波形对应关系由上层保证。
// 本类不追踪几何、不校正平台运动、不分离多回波，也不重新标定。
class LidarPointCloudBuilder
{
public:
    // 必须显式选择距离来源；缺少校正结果时不偷偷退回原始距离。
    explicit LidarPointCloudBuilder(
        LidarPointRangeSource rangeSource,
        bool rejectClippedReturns = true,
        LidarReturnSelection returnSelection = LidarReturnSelection::All)
        : rangeSource(rangeSource), rejectClippedReturns(rejectClippedReturns), returnSelection(returnSelection)
    {
        if (rangeSource != LidarPointRangeSource::Raw &&
            rangeSource != LidarPointRangeSource::Corrected)
            throw std::invalid_argument("Unknown point range source.");
        switch (returnSelection)
        {
        case LidarReturnSelection::All:
        case LidarReturnSelection::First:
        case LidarReturnSelection::Strongest:
        case LidarReturnSelection::Last:
            break;
        default:
            throw std::invalid_argument("Unknown LiDAR return selection.");
        }
    }

    std::optional<LidarPointSample> BuildPoint(
        const LidarPulseMeasurement& measurement) const
    {
        return BuildPointImpl(measurement, rejectClippedReturns);
    }

    // 多回波按各自平台/局部测距窗口拒绝削顶，搜索区/整张记录越界只保留诊断。
    // 不让另一个回波的削顶牵连当前干净回波；缺少校正时也不回退原始距离。
    std::vector<LidarPointSample> BuildPoints(const LidarMultiEchoPulseMeasurement& measurement) const
    {
        const auto& reception = measurement.reception;
        if (reception.echoes.size() > std::numeric_limits<std::uint32_t>::max())
            throw std::overflow_error("Detected echo count exceeds point metadata range.");
        const auto count = static_cast<std::uint32_t>(reception.echoes.size());
        std::vector<LidarPointSample> points;
        for (std::size_t i = 0; i < reception.echoes.size(); ++i)
        {
            const auto& echo = reception.echoes[i];
            if (rejectClippedReturns && (echo.estimate.peak.plateauOutOfRange ||
                echo.estimate.localWindowHasOutOfRangeSamples || echo.estimate.measured.peakOutOfRange))
                continue;
            LidarReceiverPipelineResult single;
            single.measured = echo.estimate.measured;
            single.corrected = echo.corrected;
            single.hasAdcOutOfRangeSamples = reception.hasAdcOutOfRangeSamples;
            auto point = BuildPointImpl({measurement.pulse, single}, false);
            if (!point) continue;
            point->returnNumber = static_cast<std::uint32_t>(i + 1);
            point->returnCount = count;
            point->plateauOutOfRange = echo.estimate.peak.plateauOutOfRange;
            point->localWindowHasOutOfRangeSamples = echo.estimate.localWindowHasOutOfRangeSamples;
            points.push_back(*point);
        }
        // Detector/pipeline supply time-ordered echoes. Select only AFTER quality filtering.
        // The original returnNumber/returnCount and reception diagnostics remain unchanged.
        if (points.empty() || returnSelection == LidarReturnSelection::All)
            return points;
        std::size_t selected = 0;
        if (returnSelection == LidarReturnSelection::Last)
            selected = points.size() - 1;
        else if (returnSelection == LidarReturnSelection::Strongest)
        {
            for (std::size_t i = 1; i < points.size(); ++i)
                if (points[i].peakAboveBaselineCodes > points[selected].peakAboveBaselineCodes)
                    selected = i; // Strict comparison keeps earlier return on a tie.
        }
        return {points[selected]};
    }

private:
    std::optional<LidarPointSample> BuildPointImpl(
        const LidarPulseMeasurement& measurement, bool rejectClipped) const
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
        if (rejectClipped &&
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

public:
    LidarPointRangeSource RangeSource() const { return rangeSource; }
    bool RejectClippedReturns() const { return rejectClippedReturns; }
    LidarReturnSelection ReturnSelection() const { return returnSelection; }

private:
    LidarPointRangeSource rangeSource;
    bool rejectClippedReturns;
    LidarReturnSelection returnSelection;
};
