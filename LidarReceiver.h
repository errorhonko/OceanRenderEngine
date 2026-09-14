#pragma once


#include "LaserEmitter.h"

#include <cmath>
#include <optional>
#include <stdexcept>
#include "Hittable.h"
struct LidarReturnGeometry
{
    // 从海面命中点指向接收器，后续计算 BSDF 会用到。
    Vector3f surfaceToReceiver;

    double outgoingDistanceMeters = 0.0;
    double returnDistanceMeters = 0.0;
    double pathLengthMeters = 0.0;

    // 相对于发射时刻的传播时间。
    double timeOfFlightSeconds = 0.0;

    // 发射时刻 + 传播时间。
    double arrivalTimeSeconds = 0.0;
};

class LidarReceiver
{
public:
    LidarReceiver(
        const Vector3f& position,
        const Vector3f& direction,
        float fovHalfAngleRadians,
        float apertureAreaSquareMeters = 0.0f,
        float opticalEfficiency = 1.0f)
        : position(position),
        direction(direction),
        fovHalfAngleRadians(fovHalfAngleRadians),
        apertureAreaSquareMeters(apertureAreaSquareMeters),
        opticalEfficiency(opticalEfficiency)
    {
        if (!IsFinite(position) ||
            !IsFinite(direction) ||
            direction.near_zero())
        {
            throw std::invalid_argument(
                "Receiver position or direction is invalid.");
        }

        if (!std::isfinite(fovHalfAngleRadians) ||
            fovHalfAngleRadians < 0.0f ||
            fovHalfAngleRadians >= PiOver2)
        {
            throw std::invalid_argument(
                "Receiver FOV half-angle is invalid.");
        }

        if (!std::isfinite(apertureAreaSquareMeters) ||
            apertureAreaSquareMeters < 0.0f)
        {
            throw std::invalid_argument(
                "Receiver aperture area is invalid.");
        }

        if (!std::isfinite(opticalEfficiency) ||
            opticalEfficiency < 0.0f ||
            opticalEfficiency > 1.0f)
        {
            throw std::invalid_argument(
                "Receiver optical efficiency is invalid.");
        }

        this->direction = direction.normalize();
        cosFovHalfAngle = std::cos(fovHalfAngleRadians);
    }

    // 参数方向是“接收器 -> 待观测点”。
    bool Accepts(
        const Vector3f& receiverToPoint) const
    {
        if (!IsFinite(receiverToPoint) ||
            receiverToPoint.near_zero())
        {
            return false;
        }

        const Vector3f unitDirection =
            receiverToPoint.normalize();

        return unitDirection.dot(direction) >=
            cosFovHalfAngle;
    }

    std::optional<LidarReturnGeometry>
        EvaluateGeometry(
            const LaserEmissionSample& emission,
            const Vector3f& surfacePoint) const
    {
        if (!IsFinite(surfacePoint) ||
            !IsFinite(emission.ray.orig) ||
            !std::isfinite(
                emission.emissionTimeSeconds))
        {
            throw std::invalid_argument(
                "Receiver geometry input is invalid.");
        }

        const Vector3f outgoingSegment =
            surfacePoint - emission.ray.orig;

        const Vector3f receiverToSurface =
            surfacePoint - position;

        const double outgoingDistance =
            outgoingSegment.norm();

        const double returnDistance =
            receiverToSurface.norm();

        if (!std::isfinite(outgoingDistance) ||
            !std::isfinite(returnDistance) ||
            outgoingDistance <= 0.0 ||
            returnDistance <= 0.0 ||
            !Accepts(receiverToSurface))
        {
            return std::nullopt;
        }

        constexpr double speedOfLightMetersPerSecond =
            299792458.0;

        const double pathLength =
            outgoingDistance + returnDistance;

        const double timeOfFlight =
            pathLength /
            speedOfLightMetersPerSecond;

        return LidarReturnGeometry{
            -receiverToSurface.normalize(),
            outgoingDistance,
            returnDistance,
            pathLength,
            timeOfFlight,
            static_cast<double>(
                emission.emissionTimeSeconds) +
                timeOfFlight
        };
    }

    std::optional<LidarReturnGeometry>
        EvaluateVisibleGeometry(
            const LaserEmissionSample& emission,
            const HitRecord& surfaceHit,
            const Hittable& world) const
    {
        auto geometry =
            EvaluateGeometry(
                emission,
                surfaceHit.point);

        if (!geometry)
            return std::nullopt;

        if (!IsFinite(surfaceHit.geometricNormal) ||
            surfaceHit.geometricNormal.near_zero())
        {
            return std::nullopt;
        }

        constexpr float rayEpsilon = 1e-4f;

        const Vector3f toReceiver =
            geometry->surfaceToReceiver;

        const Vector3f geometricNormal =
            surfaceHit.geometricNormal.normalize();

        const Vector3f offsetNormal =
            geometricNormal.dot(toReceiver) >= 0.0f
            ? geometricNormal
            : -geometricNormal;

        const Vector3f shadowOrigin =
            surfaceHit.point +
            offsetNormal * rayEpsilon;

        // 从偏移后的实际起点重新指向接收器。
        const Vector3f shadowSegment =
            position - shadowOrigin;

        const float shadowDistance =
            shadowSegment.norm();

        if (!std::isfinite(shadowDistance) ||
            shadowDistance <= 2.0f * rayEpsilon)
        {
            return std::nullopt;
        }

        const Ray shadowRay(
            shadowOrigin,
            shadowSegment);

        HitRecord blockingHit;

        if (world.hit(
            shadowRay,
            0.0f,
            shadowDistance - rayEpsilon,
            blockingHit))
        {
            return std::nullopt;
        }

        return geometry;
    }

    const Vector3f& Position() const
    {
        return position;
    }

    const Vector3f& Direction() const
    {
        return direction;
    }

    float FovHalfAngleRadians() const
    {
        return fovHalfAngleRadians;
    }
    double CollectionSolidAngle(
        const LidarReturnGeometry& geometry) const
    {
        if (apertureAreaSquareMeters == 0.0f ||
            !IsFinite(geometry.surfaceToReceiver) ||
            geometry.surfaceToReceiver.near_zero() ||
            !std::isfinite(geometry.returnDistanceMeters) ||
            geometry.returnDistanceMeters <= 0.0)
        {
            return 0.0;
        }

        // geometry 的方向是“海面 -> 接收器”；
        // 接收器光轴则指向海面，所以要取反。
        const Vector3f receiverToSurface =
            -geometry.surfaceToReceiver.normalize();

        const double cosAperture =
            direction.dot(receiverToSurface);

        if (cosAperture < cosFovHalfAngle)
            return 0.0;

        const double distance =
            geometry.returnDistanceMeters;

        return
            static_cast<double>(
                apertureAreaSquareMeters) *
            cosAperture /
            (distance * distance);
    }

    float ApertureAreaSquareMeters() const
    {
        return apertureAreaSquareMeters;
    }

    float OpticalEfficiency() const
    {
        return opticalEfficiency;
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

    Vector3f position;
    Vector3f direction;
    float fovHalfAngleRadians = 0.0f;
    float cosFovHalfAngle = 1.0f;

    float apertureAreaSquareMeters = 0.0f;
    float opticalEfficiency = 1.0f;
};