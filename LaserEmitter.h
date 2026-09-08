#pragma once

#include "BRDFUtils.h"
#include "Point2f.h"
#include "Ray.h"
#include "Vector3f.h"

#include <cmath>
#include <stdexcept>

struct LaserEmissionSample
{
    Ray ray;

    // 激光中心波长，单位 nm。
    float wavelengthNm = 0.0f;

    // 脉冲发射时刻，单位 s。
    float emissionTimeSeconds = 0.0f;

    // 已经除过采样 PDF 的能量权重，单位 J。
    float energyWeightJ = 0.0f;

    // 关于立体角的方向 PDF，单位 sr^-1。
    // 理想细光束属于 Delta 分布，此值为 0。
    float directionPdf = 0.0f;

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
    }

    LaserEmissionSample SampleRay(
        const Point2f& u,
        float emissionTimeSeconds = 0.0f) const
    {
        if (!std::isfinite(emissionTimeSeconds) ||
            emissionTimeSeconds < 0.0f)
        {
            throw std::invalid_argument(
                "Laser emission time must be finite and nonnegative.");
        }

        if (!ValidUnitSample(u))
        {
            throw std::out_of_range(
                "Laser direction sample must be in [0, 1].");
        }

        if (divergenceHalfAngleRadians == 0.0f)
        {
            return LaserEmissionSample{
                Ray(position, direction),
                wavelengthNm,
                emissionTimeSeconds,
                pulseEnergyJ,
                0.0f,
                true
            };
        }

        const float cosThetaMax =
            std::cos(
                divergenceHalfAngleRadians);

        const Vector3f localDirection =
            BRDFUtils::SampleUniformCone(
                u,
                cosThetaMax);

        const Frame emitterFrame =
            Frame::FromZ(direction);

        const Vector3f worldDirection =
            emitterFrame.FromLocal(
                localDirection).normalize();

        const float directionPdf =
            BRDFUtils::UniformConePdf(
                cosThetaMax);

        return LaserEmissionSample{
            Ray(position, worldDirection),
            wavelengthNm,
            emissionTimeSeconds,
            pulseEnergyJ,
            directionPdf,
            false
        };
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
        return divergenceHalfAngleRadians;
    }

    bool IsDeltaDirection() const
    {
        return divergenceHalfAngleRadians == 0.0f;
    }

private:
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
};