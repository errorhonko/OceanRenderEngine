#pragma once

#include "LaserEmitter.h"
#include "Vector3f.h"

#include <cmath>
#include <cstdint>
#include <stdexcept>

// 一次实际脉冲的测量元数据，不是光斑内的一条蒙特卡洛射线。
// 位置与中心方向已在世界坐标系中；本类不执行平台姿态变换。
// pulseId 唯一性由外层扫描器负责，时间使用相对仿真时钟的秒数。
class LidarPulseMetadata
{
public:
    LidarPulseMetadata(
        std::uint64_t pulseId,
        double emissionTimeSeconds,
        const Vector3f& originWorld,
        const Vector3f& centerDirectionWorld)
        : pulseId(pulseId),
          emissionTimeSeconds(emissionTimeSeconds),
          originWorld(originWorld),
          centerDirectionWorld(centerDirectionWorld)
    {
        if (!std::isfinite(emissionTimeSeconds) || emissionTimeSeconds < 0.0)
            throw std::invalid_argument("Pulse emission time must be finite and nonnegative.");
        if (!IsFinite(originWorld) || !IsFinite(centerDirectionWorld))
            throw std::invalid_argument("Pulse position and direction must be finite.");

        // double 可安全容纳有限 float 分量的平方，也保留很小方向的长度。
        const double x = centerDirectionWorld.x;
        const double y = centerDirectionWorld.y;
        const double z = centerDirectionWorld.z;
        const double length = std::sqrt(x * x + y * y + z * z);
        if (length == 0.0)
            throw std::invalid_argument("Pulse center direction must be nonzero.");
        this->centerDirectionWorld = Vector3f(
            static_cast<float>(x / length),
            static_cast<float>(y / length),
            static_cast<float>(z / length));
    }

    static LidarPulseMetadata FromEmitter(
        std::uint64_t pulseId,
        double emissionTimeSeconds,
        const LaserEmitter& emitter)
    {
        // 使用发射器中心方向，不调用 SampleRay，也不消耗随机样本。
        return LidarPulseMetadata(pulseId, emissionTimeSeconds,
            emitter.Position(), emitter.Direction());
    }

    std::uint64_t PulseId() const { return pulseId; }
    double EmissionTimeSeconds() const { return emissionTimeSeconds; }
    const Vector3f& OriginWorld() const { return originWorld; }
    const Vector3f& CenterDirectionWorld() const { return centerDirectionWorld; }

private:
    static bool IsFinite(const Vector3f& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    std::uint64_t pulseId;
    double emissionTimeSeconds;
    Vector3f originWorld;
    Vector3f centerDirectionWorld;
};
