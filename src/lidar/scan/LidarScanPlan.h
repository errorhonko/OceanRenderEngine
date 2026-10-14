#pragma once
#include "lidar/simulation/LidarPulseMetadata.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

struct LidarRasterCell
{
    std::size_t azimuthIndex;
    std::size_t elevationIndex;
    double azimuthRadians;
    double elevationRadians;
};

// 一个通道的一次实际发射，不是光束内部的一条 MC 射线。
// 局部坐标沿用现有约定：X 右、Y 上、Z 前；位置 m，时间 s。
// world 变换在计划之外完成；矩形网格索引不是通用计划的必需属性。
struct LidarFiringEvent
{
    std::uint64_t pulseId = 0;
    std::uint32_t channelId = 0;
    double emissionTimeSeconds = 0.0;
    Vector3f originSensor{0,0,0};
    Vector3f directionSensor{0,0,1};
    std::optional<LidarRasterCell> raster;
};

// 一个有限扫描记录的发射计划。顺序由实现确定，Firing 不消耗随机数。
// 非空、ID 唯一、时间非递减；不同通道允许同一时刻发射。
// 不涉及场景求交、MC 能量权重、电子接收或点云构建。
class LidarScanPlan
{
public:
    virtual ~LidarScanPlan() = default;
    virtual std::size_t Size() const = 0;
    virtual LidarFiringEvent Firing(std::size_t index) const = 0;
};

struct LidarScanPoseConfig
{
    Vector3f originWorld{0,0,0};
    Vector3f forwardWorld{0,-1,0};
    Vector3f upHintWorld{0,0,1};
};

// 当前只做固定刚体位姿适配，不是平台运动或 ROS 坐标适配器。
class LidarScanPose
{
public:
    explicit LidarScanPose(const LidarScanPoseConfig& config)
        : origin(config.originWorld)
    {
        if (!Finite(origin)) throw std::invalid_argument("Scan origin must be finite.");
        forward = Normalize(config.forwardWorld);
        const auto cross = Cross(Normalize(config.upHintWorld), forward);
        if (Length(cross) <= 1e-6)
            throw std::invalid_argument("Scan up hint must not be parallel to forward.");
        right = Normalize(cross);
        up = Normalize(Cross(forward, right));
    }

    LidarPulseMetadata ToWorld(const LidarFiringEvent& event) const
    {
        // 复用元数据校验与稳健归一化，避免大/小方向导致 float 平方溢出。
        const LidarPulseMetadata local(event.pulseId, event.emissionTimeSeconds,
            event.originSensor, event.directionSensor);
        const auto position = Transform(local.OriginWorld(), true);
        const auto direction = Transform(local.CenterDirectionWorld(), false);
        return LidarPulseMetadata(event.pulseId, event.emissionTimeSeconds, position, direction);
    }

    Vector3f ToSensor(const Vector3f& pointWorld) const
    {
        if(!Finite(pointWorld))throw std::invalid_argument("World point must be finite.");
        const double x=double(pointWorld.x)-origin.x,y=double(pointWorld.y)-origin.y,z=double(pointWorld.z)-origin.z;
        const double a=x*right.x+y*right.y+z*right.z;
        const double b=x*up.x+y*up.y+z*up.z;
        const double c=x*forward.x+y*forward.y+z*forward.z;
        const double limit=std::numeric_limits<float>::max();
        if(!std::isfinite(a)||!std::isfinite(b)||!std::isfinite(c)||std::abs(a)>limit||std::abs(b)>limit||std::abs(c)>limit)
            throw std::overflow_error("Sensor point cannot be represented.");
        return {float(a),float(b),float(c)};
    }
private:
    static bool Finite(const Vector3f& v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }
    static double Length(const Vector3f& v) {
        return std::sqrt(double(v.x)*v.x + double(v.y)*v.y + double(v.z)*v.z);
    }
    static Vector3f Normalize(const Vector3f& v) {
        if (!Finite(v) || Length(v)==0)
            throw std::invalid_argument("Scan axes must be finite and nonzero.");
        const double length=Length(v);
        return {float(v.x/length),float(v.y/length),float(v.z/length)};
    }
    static Vector3f Cross(const Vector3f& a,const Vector3f& b) {
        return {float(double(a.y)*b.z-double(a.z)*b.y),
            float(double(a.z)*b.x-double(a.x)*b.z),float(double(a.x)*b.y-double(a.y)*b.x)};
    }
    Vector3f Transform(const Vector3f& v,bool position) const {
        const double x=double(v.x)*right.x+double(v.y)*up.x+double(v.z)*forward.x+(position?origin.x:0);
        const double y=double(v.x)*right.y+double(v.y)*up.y+double(v.z)*forward.y+(position?origin.y:0);
        const double z=double(v.x)*right.z+double(v.y)*up.z+double(v.z)*forward.z+(position?origin.z:0);
        const double limit=std::numeric_limits<float>::max();
        if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||
            std::abs(x)>limit||std::abs(y)>limit||std::abs(z)>limit)
            throw std::overflow_error("Scan world coordinates cannot be represented.");
        return {float(x),float(y),float(z)};
    }
    Vector3f origin,forward,right,up;
};

struct LidarPlannedPulse
{
    std::size_t scanIndex;
    LidarFiringEvent firing;
    LidarPulseMetadata pulse; // 已通过固定安装位姿转换到世界坐标
};
