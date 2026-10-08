#pragma once


#include "LidarReceiver.h"
#include "HitRocord.h"
#include "LidarScatteringModel.h"
#include <cmath>
#include <optional>
#include <stdexcept>

struct LidarReturnSample
{
    double receivedEnergyJ = 0.0;
    double arrivalTimeSeconds = 0.0;
    float wavelengthNm = 0.0f;
};

inline std::optional<LidarReturnSample> EstimateSingleReturn(
    const LaserEmissionSample& emission,
    const HitRecord& hit,
    const LidarReceiver& receiver,
    const Hittable& world,
    const LidarScatteringModel& scatteringModel)
{
    if (!std::isfinite(emission.energyWeightJ) ||
        emission.energyWeightJ < 0.0f)
    {
        throw std::invalid_argument(
            "Invalid return energy input.");
    }

    const auto geometry =
        receiver.EvaluateVisibleGeometry(
            emission,
            hit,
            world);

    if (!geometry)
        return std::nullopt;

    const Vector3f ng =
        hit.geometricNormal.normalize();

    // 两个方向都采用“从表面向外”的约定。
    const Vector3f incidentDirection =
        -emission.ray.dir;

    const Vector3f outgoingDirection =
        geometry->surfaceToReceiver;

    const double cosIncident =
        ng.dot(incidentDirection);

    const double cosReturn =
        ng.dot(outgoingDirection);

    // 当前基线只处理海面正面反射。
    if (cosIncident <= 0.0 ||
        cosReturn <= 0.0)
    {
        return std::nullopt;
    }

    const double brdfAtWavelengthPerSr =
        scatteringModel.Evaluate(
            hit,
            incidentDirection,
            outgoingDirection,
            emission.wavelengthNm);

    // 不能只依靠具体模型的构造函数验证：
    // 任意派生模型都有可能返回非法结果。
    if (!std::isfinite(brdfAtWavelengthPerSr) ||
        brdfAtWavelengthPerSr < 0.0)
    {
        throw std::domain_error(
            "LiDAR scattering model returned an invalid BRDF.");
    }

    const double solidAngle =
        receiver.CollectionSolidAngle(*geometry);

    const double receivedEnergy =
        static_cast<double>(
            emission.energyWeightJ) *
        brdfAtWavelengthPerSr *
        cosReturn *
        solidAngle *
        receiver.OpticalEfficiency();

    if (!std::isfinite(receivedEnergy))
    {
        throw std::overflow_error(
            "Return energy overflow.");
    }

    if (receivedEnergy <= 0.0)
        return std::nullopt;

    return LidarReturnSample{
        receivedEnergy,
        geometry->arrivalTimeSeconds,
        emission.wavelengthNm
    };
}