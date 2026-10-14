#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

class LidarPulseProfile
{
public:
    virtual ~LidarPulseProfile() = default;

    // 返回脉冲总能量中，落在指定相对时间区间内的比例。
    //
    // relativeStartSeconds 和 relativeEndSeconds
    // 都相对于脉冲参考时刻：
    //   Gaussian 中通常令 t = 0 为脉冲中心。
    //
    // 返回值理论范围为 [0, 1]。
    virtual double FractionBetween(
        double relativeStartSeconds,
        double relativeEndSeconds) const = 0;
};
