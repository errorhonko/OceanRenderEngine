#pragma once

#include "Vector3f.h"
#include <cstdint>

enum class LidarPointRangeSource
{
    Raw,
    Corrected
};

// 单次脉冲对应的一个估计点，非蒙特卡洛射线的真实命中点。
struct LidarPointSample
{
    Vector3f positionWorld;
    Vector3f centerDirectionWorld;
    std::uint64_t pulseId = 0;
    double emissionTimeSeconds = 0.0;
    double arrivalTimeSeconds = 0.0; // 原始测距算法的到达时刻
    double rangeMeters = 0.0;
    LidarPointRangeSource rangeSource = LidarPointRangeSource::Raw;

    std::uint32_t peakCode = 0;
    double peakAboveBaselineCodes = 0.0; // 码幅值，不是反射率或 J
    bool peakOutOfRange = false;
    bool searchHasOutOfRangeSamples = false;
    bool hasAdcOutOfRangeSamples = false; // 整张记录的诊断
};
