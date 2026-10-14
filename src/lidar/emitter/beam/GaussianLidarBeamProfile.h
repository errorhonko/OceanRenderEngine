#pragma once

#include "lidar/emitter/beam/LidarBeamProfile.h"

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

// 远场、圆对称、截断后归一化的高斯斜率光束。
// 不模拟有限束腰、聚焦、衍射或相位；主轴为发射器局部 +Z。
// pulseEnergyJ 由 LaserEmitter 管理，表示截断后光束的总能量。
class GaussianLidarBeamProfile final : public LidarBeamProfile
{
public:
    GaussianLidarBeamProfile(
        double e2HalfAngleRadians,
        double maxHalfAngleRadians)
        : e2HalfAngleRadians(e2HalfAngleRadians),
          maxHalfAngleRadians(maxHalfAngleRadians)
    {
        constexpr double halfPi = std::numbers::pi_v<double> / 2.0;
        if (!std::isfinite(e2HalfAngleRadians) ||
            !std::isfinite(maxHalfAngleRadians) ||
            e2HalfAngleRadians <= 0.0 ||
            maxHalfAngleRadians < e2HalfAngleRadians ||
            maxHalfAngleRadians >= halfPi)
        {
            throw std::invalid_argument(
                "Gaussian beam angles must satisfy "
                "0 < e2HalfAngle <= maxHalfAngle < pi/2.");
        }

        // a 与 sMax 是斜率，不是角度；使用 tan 做精确几何转换。
        slopeScale = std::tan(e2HalfAngleRadians);
        maxSlope = std::tan(maxHalfAngleRadians);
        slopeScaleSquared = slopeScale * slopeScale;
        maxSlopeSquared = maxSlope * maxSlope;

        const double cutoffRatio = maxSlope / slopeScale;
        const double cutoffExponent = 2.0 * cutoffRatio * cutoffRatio;
        // Z = 1 - exp(-2*sMax^2/a^2)，缓存截断前的保留能量比例。
        retainedFraction = -std::expm1(-cutoffExponent);
        // b_s = C * exp(-2*s^2/a^2)，C 保证截止圆内的分布积分为 1。
        slopeDensityNormalization =
            (2.0 / std::numbers::pi_v<double>) /
            slopeScaleSquared / retainedFraction;

        if (!std::isfinite(slopeScaleSquared) ||
            slopeScaleSquared <= 0.0 ||
            !std::isfinite(maxSlopeSquared) ||
            !std::isfinite(retainedFraction) ||
            retainedFraction <= 0.0 ||
            retainedFraction > 1.0 ||
            !std::isfinite(slopeDensityNormalization) ||
            slopeDensityNormalization <= 0.0)
        {
            throw std::invalid_argument(
                "Gaussian beam parameters exceed the supported numerical range.");
        }
    }

    LidarBeamDirectionSample SampleDirection(const Point2f& u) const override
    {
        if (!std::isfinite(u.x) || !std::isfinite(u.y) ||
            u.x < 0.0f || u.x > 1.0f || u.y < 0.0f || u.y > 1.0f)
        {
            throw std::out_of_range("Gaussian beam samples must be finite and in [0, 1].");
        }

        // 连续反函数采样使用 [0,1)。接口允许输入 1，径向端点向内映射到
        // 最大的 float < 1；避免 Z 舍入为 1 时的 log(0)，以及极弱尾部 PDF 下溢。
        // 这只改变概率为零的精确端点，不改变连续目标分布。
        const double radialU = u.x == 1.0f ?
            double(std::nextafter(1.0f, 0.0f)) : double(u.x);
        const double s = slopeScale *
            std::sqrt(-0.5 * std::log1p(-radialU * retainedFraction));
        const double phi = 2.0 * std::numbers::pi_v<double> * double(u.y);
        const double invLength = 1.0 / std::sqrt(1.0 + s * s);
        const Vector3f direction(
            float(s * std::cos(phi) * invLength),
            float(s * std::sin(phi) * invLength),
            float(invLength));

        // 返回实际 float 方向的 PDF，使采样结果与后续查询保持一致。
        const double pdf = PdfDirection(direction);
        if (!std::isfinite(pdf) || pdf <= 0.0)
            throw std::runtime_error("Gaussian beam sample PDF is not representable.");

        // q = b，因此 b/q = 1；总脉冲能量与 1/N 由外部链路管理。
        return { direction, pdf, 1.0, false };
    }

    double EnergyDensityPerSr(const Vector3f& directionLocal) const override
    {
        const double x = directionLocal.x;
        const double y = directionLocal.y;
        const double z = directionLocal.z;
        const double lengthSquared = x * x + y * y + z * z;
        if (!std::isfinite(lengthSquared) ||
            std::abs(lengthSquared - 1.0) > 1e-5)
        {
            throw std::invalid_argument("Gaussian beam density requires a finite local unit direction.");
        }
        if (z <= 0.0)
            return 0.0;

        // 使用横向分量恢复斜率，避免窄角度下 float 的 z 和 cos(theta) 都舍入为 1。
        const double sx = x / z;
        const double sy = y / z;
        double sSquared = sx * sx + sy * sy;
        if (sSquared > maxSlopeSquared)
        {
            // Vector3f 的 float 分量可能使数学截止边界轻微越界。
            // 仅在相对舍入带内接受并钳制径向值；明显域外方向仍返回 0。
            constexpr double roundingTolerance =
                8.0 * std::numeric_limits<float>::epsilon();
            if (sSquared > maxSlopeSquared * (1.0 + roundingTolerance))
                return 0.0;
            sSquared = maxSlopeSquared;
        }

        // dOmega = dsx*dsy / (1+s^2)^(3/2)。
        // b_omega = b_s * (1+s^2)^(3/2)，单位 sr^-1。
        // 在对数域合并指数与雅可比，避免中间项先溢出/下溢。
        const double logDensity = std::log(slopeDensityNormalization) -
            2.0 * (sSquared / slopeScaleSquared) +
            1.5 * std::log1p(sSquared);
        return std::exp(logDensity);
    }

    double PdfDirection(const Vector3f& directionLocal) const override
    {
        // 第一版直接按物理光束分布采样，q(omega) = b(omega)。
        return EnergyDensityPerSr(directionLocal);
    }

    bool IsDeltaDirection() const override { return false; }

    // 未截断横截面高斯的 1/e^2 尺度对应半角，不是截止角。
    double E2HalfAngleRadians() const { return e2HalfAngleRadians; }
    double MaxHalfAngleRadians() const { return maxHalfAngleRadians; }
    // 仅提供截断诊断；不能再次乘到截断后定义的 pulseEnergyJ 上。
    double RetainedFraction() const { return retainedFraction; }

private:
    double e2HalfAngleRadians = 0.0;
    double maxHalfAngleRadians = 0.0;
    double slopeScale = 0.0;
    double maxSlope = 0.0;
    double slopeScaleSquared = 0.0;
    double maxSlopeSquared = 0.0;
    double retainedFraction = 0.0;
    double slopeDensityNormalization = 0.0;
};
