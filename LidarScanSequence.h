#pragma once

#include "LidarPulseMetadata.h"
#include "Vector3f.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

struct LidarScanSequenceConfig
{
    Vector3f originWorld{0, 0, 0};
    // 默认朝 -Y 扫描海面，局部上方指向世界 +Z，局部右方为 +X。
    Vector3f forwardWorld{0, -1, 0};
    Vector3f upHintWorld{0, 0, 1};
    double azimuthMinRadians = -0.1;
    double azimuthMaxRadians = 0.1;
    double elevationMinRadians = -0.1;
    double elevationMaxRadians = 0.1;
    std::size_t azimuthCount = 1;
    std::size_t elevationCount = 1;
    bool serpentine = false;
    std::uint64_t firstPulseId = 0;
    double startTimeSeconds = 0.0;
    double pulseIntervalSeconds = 1.0e-4;
};

struct LidarScanSample
{
    std::size_t scanIndex;
    std::size_t azimuthIndex;
    std::size_t elevationIndex;
    double azimuthRadians;   // 扫描器局部角度，不是世界坐标方位角
    double elevationRadians;
    LidarPulseMetadata pulse;
};

// 固定位姿的矩形角度网格。每个样本代表一个实际脉冲，不是 MC 射线。
// 方位快轴、俯仰慢轴；取网格单元中心，不包含两侧边界。
// 只提供发射计划，不模拟镜面动力学、回扫耗时、运动补偿或距离模糊。
class LidarScanSequence
{
public:
    explicit LidarScanSequence(const LidarScanSequenceConfig& config)
        : config(config)
    {
        constexpr double pi = 3.14159265358979323846;
        if (!IsFinite(config.originWorld))
            throw std::invalid_argument("Scan origin must be finite.");
        if (config.azimuthCount == 0 || config.elevationCount == 0 ||
            config.azimuthCount > std::numeric_limits<std::size_t>::max() / config.elevationCount)
            throw std::invalid_argument("Scan dimensions must be positive and representable.");
        count = config.azimuthCount * config.elevationCount;
        if (count - 1 > std::numeric_limits<std::uint64_t>::max() - config.firstPulseId)
            throw std::invalid_argument("Scan pulse ids overflow.");

        ValidateAxis(config.azimuthMinRadians, config.azimuthMaxRadians, config.azimuthCount);
        ValidateAxis(config.elevationMinRadians, config.elevationMaxRadians, config.elevationCount);
        if (std::abs(config.azimuthMinRadians) > 2*pi || std::abs(config.azimuthMaxRadians) > 2*pi ||
            config.azimuthMaxRadians - config.azimuthMinRadians > 2*pi ||
            config.elevationMinRadians < -pi/2 || config.elevationMaxRadians > pi/2)
            throw std::invalid_argument("Scan angle range is invalid.");
        if (!std::isfinite(config.startTimeSeconds) || config.startTimeSeconds < 0.0 ||
            !std::isfinite(config.pulseIntervalSeconds) || config.pulseIntervalSeconds <= 0.0)
            throw std::invalid_argument("Scan timing must be finite with positive pulse interval.");
        const double lastTime = std::fma(double(count - 1), config.pulseIntervalSeconds, config.startTimeSeconds);
        if (!std::isfinite(lastTime))
            throw std::invalid_argument("Scan final emission time overflows.");
        if (count > 1)
        {
            const double spacing = std::nextafter(lastTime, std::numeric_limits<double>::infinity()) - lastTime;
            if (double(count - 1) == double(count - 2) || config.pulseIntervalSeconds < spacing)
                throw std::invalid_argument("Scan pulse times cannot be distinguished in double precision.");
        }
        forward = Normalize(config.forwardWorld);
        const Vector3f upHint = Normalize(config.upHintWorld);
        const Vector3f cross = Cross(upHint, forward);
        if (Length(cross) <= 1.0e-6)
            throw std::invalid_argument("Scan up hint must not be parallel to forward.");
        right = Normalize(cross);
        up = Normalize(Cross(forward, right));
    }

    std::size_t Size() const { return count; }
    const LidarScanSequenceConfig& Config() const { return config; }

    LidarScanSample Sample(std::size_t index) const
    {
        if (index >= count)
            throw std::out_of_range("Scan index is out of range.");
        const std::size_t row = index / config.azimuthCount;
        std::size_t column = index % config.azimuthCount;
        if (config.serpentine && row % 2 != 0)
            column = config.azimuthCount - 1 - column;
        const double azimuth = Angle(config.azimuthMinRadians, config.azimuthMaxRadians, column, config.azimuthCount);
        const double elevation = Angle(config.elevationMinRadians, config.elevationMaxRadians, row, config.elevationCount);

        // 局部坐标：X=右、Y=上、Z=前；零角度沿 forward。
        const double x = std::cos(elevation) * std::sin(azimuth);
        const double y = std::sin(elevation);
        const double z = std::cos(elevation) * std::cos(azimuth);
        const Vector3f direction(
            float(x*right.x + y*up.x + z*forward.x),
            float(x*right.y + y*up.y + z*forward.y),
            float(x*right.z + y*up.z + z*forward.z));
        const double time = std::fma(double(index), config.pulseIntervalSeconds, config.startTimeSeconds);
        return {index, column, row, azimuth, elevation,
            LidarPulseMetadata(config.firstPulseId + std::uint64_t(index), time, config.originWorld, direction)};
    }

private:
    static bool IsFinite(const Vector3f& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
    static double Length(const Vector3f& v) { return std::sqrt(double(v.x)*v.x + double(v.y)*v.y + double(v.z)*v.z); }
    static Vector3f Normalize(const Vector3f& v)
    {
        if (!IsFinite(v)) throw std::invalid_argument("Scan axes must be finite.");
        const double length = Length(v);
        if (length == 0.0) throw std::invalid_argument("Scan axes must be nonzero.");
        return {float(v.x/length), float(v.y/length), float(v.z/length)};
    }
    static Vector3f Cross(const Vector3f& a, const Vector3f& b)
    {
        return {float(double(a.y)*b.z-double(a.z)*b.y), float(double(a.z)*b.x-double(a.x)*b.z), float(double(a.x)*b.y-double(a.y)*b.x)};
    }
    static void ValidateAxis(double minimum, double maximum, std::size_t n)
    {
        if (!std::isfinite(minimum) || !std::isfinite(maximum) || maximum < minimum ||
            (n > 1 && maximum == minimum))
            throw std::invalid_argument("Scan bounds must be ordered and nondegenerate for multiple samples.");
    }
    static double Angle(double minimum, double maximum, std::size_t i, std::size_t n)
    {
        return std::lerp(minimum, maximum, (double(i) + 0.5) / double(n));
    }

    LidarScanSequenceConfig config;
    std::size_t count = 0;
    Vector3f forward;
    Vector3f right;
    Vector3f up;
};
