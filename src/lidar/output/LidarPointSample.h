#pragma once

#include "core/math/Vector3f.h"
#include <cstdint>
#include <optional>

enum class LidarPointRangeSource
{
    Raw,
    Corrected
};

// 单次脉冲对应的一个估计点，非蒙特卡洛射线的真实命中点。
struct LidarPointSample
{
    Vector3f positionWorld;
    // 发射时刻的传感器坐标，尚未去畸变；仅扫描驱动保证填充。
    std::optional<Vector3f> positionSensor;
    Vector3f centerDirectionWorld;
    std::uint64_t pulseId = 0;
    std::uint32_t channelId = 0;
    double emissionTimeSeconds = 0.0;
    double arrivalTimeSeconds = 0.0; // 原始测距算法的到达时刻
    double rangeMeters = 0.0;
    LidarPointRangeSource rangeSource = LidarPointRangeSource::Raw;

    std::uint32_t peakCode = 0;
    double peakAboveBaselineCodes = 0.0; // 码幅值，不是反射率或 J
    bool peakOutOfRange = false;
    bool searchHasOutOfRangeSamples = false;
    bool hasAdcOutOfRangeSamples = false; // 整张记录的诊断
    // 检出回波的原始时间顺序（1 起始）与检出总数；过滤后不重编号。
    // returnCount 不是过滤后的输出点数，不直接当作 LAS 的 number_of_returns。
    std::uint32_t returnNumber = 1;
    std::uint32_t returnCount = 1;
    bool plateauOutOfRange = false;
    bool localWindowHasOutOfRangeSamples = false;
};
