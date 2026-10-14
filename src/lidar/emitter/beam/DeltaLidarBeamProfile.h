#pragma once

#include "lidar/emitter/beam/LidarBeamProfile.h"
#include <cmath>
#include <stdexcept>

// 理想零发散光束：方向概率质量集中在发射器局部 +Z。
// 不是普通立体角密度，不能把 pdf=0 当成没有发射能量。
class DeltaLidarBeamProfile final : public LidarBeamProfile
{
public:
    LidarBeamDirectionSample SampleDirection(const Point2f& u) const override
    {
        if (!std::isfinite(u.x) || !std::isfinite(u.y) ||
            u.x < 0 || u.x > 1 || u.y < 0 || u.y > 1)
            throw std::out_of_range("Delta beam samples must be finite and in [0, 1].");
        return { Vector3f(0, 0, 1), 0.0, 1.0, true };
    }
    double EnergyDensityPerSr(const Vector3f&) const override { return 0.0; }
    double PdfDirection(const Vector3f&) const override { return 0.0; }
    bool IsDeltaDirection() const override { return true; }
};
