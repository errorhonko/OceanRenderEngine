#pragma once

#include "lidar/emitter/beam/LidarBeamProfile.h"
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

// 关于立体角均匀的圆锥光束，不是横截面面积均匀的圆盘。
// 采用 PBRT 的均匀圆锥反函数采样：cos(theta)=1-u*(1-cos(thetaMax))。
class UniformConeLidarBeamProfile final : public LidarBeamProfile
{
public:
    explicit UniformConeLidarBeamProfile(float halfAngleRadians)
        : halfAngleRadians(halfAngleRadians)
    {
        if (!std::isfinite(halfAngleRadians) || halfAngleRadians <= 0.0f ||
            double(halfAngleRadians) >= std::numbers::pi_v<double> / 2.0)
            throw std::invalid_argument("Uniform cone half angle must be in (0, pi/2); use Delta for zero.");
        // 1-cos(thetaMax)=2*sin^2(thetaMax/2)，避免窄锥体中的消减误差。
        const double sine = std::sin(double(halfAngleRadians) / 2.0);
        oneMinusCosMax = 2.0 * sine * sine;
        pdfPerSr = 1.0 / (2.0 * std::numbers::pi_v<double> * oneMinusCosMax);
        if (!std::isfinite(pdfPerSr) || pdfPerSr <= 0.0)
            throw std::invalid_argument("Uniform cone PDF exceeds the supported numerical range.");
    }

    LidarBeamDirectionSample SampleDirection(const Point2f& u) const override
    {
        if (!std::isfinite(u.x) || !std::isfinite(u.y) ||
            u.x < 0 || u.x > 1 || u.y < 0 || u.y > 1)
            throw std::out_of_range("Uniform cone samples must be finite and in [0, 1].");
        const double t = double(u.x) * oneMinusCosMax;
        const double cosine = 1.0 - t;
        // sin^2(theta)=t*(2-t)，保留 z 舍入为 1 时的横向方向分量。
        const double sine = std::sqrt(t * (2.0 - t));
        const double phi = 2.0 * std::numbers::pi_v<double> * double(u.y);
        return { Vector3f(float(sine * std::cos(phi)), float(sine * std::sin(phi)), float(cosine)),
            pdfPerSr, 1.0, false };
    }

    double EnergyDensityPerSr(const Vector3f& directionLocal) const override
    {
        const double x = directionLocal.x, y = directionLocal.y, z = directionLocal.z;
        const double lengthSquared = x*x + y*y + z*z;
        if (!std::isfinite(lengthSquared) || std::abs(lengthSquared-1.0) > 1e-5)
            throw std::invalid_argument("Uniform cone density requires a finite local unit direction.");
        if (z <= 0.0) return 0.0;
        const double angle = std::atan2(std::hypot(x,y),z);
        // 仅容纳 float 方向的边界舍入，不用 float cos(theta) 判断窄锥体。
        constexpr double tolerance = 8.0 * std::numeric_limits<float>::epsilon();
        return angle <= double(halfAngleRadians)*(1.0+tolerance) ? pdfPerSr : 0.0;
    }
    double PdfDirection(const Vector3f& directionLocal) const override
    {
        return EnergyDensityPerSr(directionLocal);
    }
    bool IsDeltaDirection() const override { return false; }
    float HalfAngleRadians() const { return halfAngleRadians; }

private:
    float halfAngleRadians;
    double oneMinusCosMax = 0.0;
    double pdfPerSr = 0.0;
};
