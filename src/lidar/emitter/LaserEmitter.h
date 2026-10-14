#pragma once

#include "reflection/microfacet/BRDFUtils.h"
#include "lidar/emitter/beam/LidarBeamProfile.h"
#include "lidar/emitter/beam/DeltaLidarBeamProfile.h"
#include "lidar/emitter/beam/UniformConeLidarBeamProfile.h"
#include "core/math/Point2f.h"
#include "core/ray/Ray.h"
#include "core/math/Vector3f.h"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

struct LaserEmissionSample
{
    Ray ray;

    // 激光中心波长，单位 nm。
    float wavelengthNm = 0.0f;

    // 脉冲发射时刻，单位 s。
    // 与扫描元数据、回波和测距统一为 double，使用相对仿真时钟。
    double emissionTimeSeconds = 0.0;

    // 已经除过采样 PDF 的能量权重，单位 J。
    double energyWeightJ = 0.0;

    // 关于立体角的方向 PDF，单位 sr^-1。
    // 理想细光束属于 Delta 分布，此值为 0。
    double directionPdf = 0.0;

    bool deltaDirection = false;
};

class LaserEmitter
{
public:
    LaserEmitter(
        const Vector3f& position,
        const Vector3f& direction,
        float wavelengthNm,
        float pulseEnergyJ,
        float divergenceHalfAngleRadians)
        : position(position),
        direction(direction),
        wavelengthNm(wavelengthNm),
        pulseEnergyJ(pulseEnergyJ),
        divergenceHalfAngleRadians(
            divergenceHalfAngleRadians)
    {
        if (!IsFinite(this->position))
        {
            throw std::invalid_argument(
                "Laser position must be finite.");
        }

        if (!IsFinite(this->direction) ||
            this->direction.near_zero())
        {
            throw std::invalid_argument(
                "Laser direction must be finite and nonzero.");
        }

        if (!std::isfinite(this->wavelengthNm) ||
            this->wavelengthNm <= 0.0f)
        {
            throw std::invalid_argument(
                "Laser wavelength must be positive.");
        }

        if (!std::isfinite(this->pulseEnergyJ) ||
            this->pulseEnergyJ <= 0.0f)
        {
            throw std::invalid_argument(
                "Laser pulse energy must be positive.");
        }

        if (!std::isfinite(
            this->divergenceHalfAngleRadians) ||
            this->divergenceHalfAngleRadians < 0.0f ||
            this->divergenceHalfAngleRadians >= PiOver2)
        {
            throw std::invalid_argument(
                "Laser divergence half-angle is invalid.");
        }

        this->direction =
            this->direction.normalize();
        this->beamProfile = MakeLegacyBeam(divergenceHalfAngleRadians);
    }

    // 可选空间光束入口；旧的 float 半角构造保持 Delta / 均匀锥体行为。
    LaserEmitter(
        const Vector3f& position,
        const Vector3f& direction,
        float wavelengthNm,
        float pulseEnergyJ,
        std::shared_ptr<const LidarBeamProfile> beamProfile)
        : LaserEmitter(position, direction, wavelengthNm, pulseEnergyJ, 0.0f)
    {
        if (!beamProfile)
            throw std::invalid_argument("Custom laser beam profile must not be null.");
        this->beamProfile = std::move(beamProfile);
        legacyAngleConfigured = false;
    }

    LaserEmissionSample SampleRay(
        const Point2f& u,
        double emissionTimeSeconds = 0.0) const
    {
        if (!std::isfinite(emissionTimeSeconds) ||
            emissionTimeSeconds < 0.0)
        {
            throw std::invalid_argument(
                "Laser emission time must be finite and nonnegative.");
        }

        if (!ValidUnitSample(u))
        {
            throw std::out_of_range(
                "Laser direction sample must be in [0, 1].");
        }

        // 所有模型统一通过此接口采样；不再保留内置 Delta / 锥体算法分支。
        {
            const auto sample = beamProfile->SampleDirection(u);
            const double x = sample.directionLocal.x;
            const double y = sample.directionLocal.y;
            const double z = sample.directionLocal.z;
            const double lengthSquared = x * x + y * y + z * z;
            if (!std::isfinite(lengthSquared) ||
                std::abs(lengthSquared - 1.0) > 1e-5 ||
                !std::isfinite(sample.energyWeight) || sample.energyWeight < 0.0 ||
                !std::isfinite(sample.pdfPerSr) ||
                sample.deltaDirection != beamProfile->IsDeltaDirection() ||
                (sample.deltaDirection && (sample.pdfPerSr != 0.0 || sample.energyWeight != 1.0)) ||
                (!sample.deltaDirection && sample.pdfPerSr <= 0.0))
            {
                throw std::domain_error("Laser beam profile returned an invalid direction sample.");
            }

            const double energyWeight = double(pulseEnergyJ) * sample.energyWeight;
            if (!std::isfinite(energyWeight))
                throw std::overflow_error("Laser emission energy weight overflow.");
            // 保留旧的轴向 Delta 射线原值，避免额外 normalize 引入舍入变化。
            const bool axialDelta = sample.deltaDirection &&
                sample.directionLocal.x == 0 && sample.directionLocal.y == 0 && sample.directionLocal.z == 1;
            const Vector3f worldDirection = axialDelta ? direction :
                Frame::FromZ(direction).FromLocal(sample.directionLocal).normalize();
            // 刚体旋转不改变立体角 PDF；模型已提供 b/q，不再乘高斯权重或 1/N。
            return { Ray(position, worldDirection), wavelengthNm, emissionTimeSeconds,
                energyWeight, sample.pdfPerSr, sample.deltaDirection };
        }

    }

    const Vector3f& Position() const
    {
        return position;
    }

    const Vector3f& Direction() const
    {
        return direction;
    }

    float WavelengthNm() const
    {
        return wavelengthNm;
    }

    float PulseEnergyJ() const
    {
        return pulseEnergyJ;
    }

    float DivergenceHalfAngleRadians() const
    {
        if (!legacyAngleConfigured)
            throw std::logic_error("Legacy divergence angle is not defined for a custom beam profile.");
        return divergenceHalfAngleRadians;
    }

    bool IsDeltaDirection() const
    {
        return beamProfile->IsDeltaDirection();
    }

    // 正常构造后始终非空，旧半角构造也持有具体光束模型。
    const std::shared_ptr<const LidarBeamProfile>& BeamProfile() const { return beamProfile; }

private:
    static std::shared_ptr<const LidarBeamProfile> MakeLegacyBeam(float halfAngle)
    {
        if (halfAngle == 0.0f)
        {
            static const auto delta = std::make_shared<const DeltaLidarBeamProfile>();
            return delta;
        }
        return std::make_shared<const UniformConeLidarBeamProfile>(halfAngle);
    }

    static bool IsFinite(
        const Vector3f& value)
    {
        return
            std::isfinite(value.x) &&
            std::isfinite(value.y) &&
            std::isfinite(value.z);
    }

    static bool ValidUnitSample(
        const Point2f& u)
    {
        return
            std::isfinite(u.x) &&
            std::isfinite(u.y) &&
            u.x >= 0.0f &&
            u.x <= 1.0f &&
            u.y >= 0.0f &&
            u.y <= 1.0f;
    }

    Vector3f position;
    Vector3f direction;

    float wavelengthNm = 0.0f;
    float pulseEnergyJ = 0.0f;

    float divergenceHalfAngleRadians = 0.0f;
    bool legacyAngleConfigured = true;
    std::shared_ptr<const LidarBeamProfile> beamProfile;
};
