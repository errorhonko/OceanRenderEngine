#pragma once


#include "LidarReceiver.h"
#include "HitRocord.h"

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
    double brdfAtWavelengthPerSr)
{
    if (!std::isfinite(brdfAtWavelengthPerSr) ||
        brdfAtWavelengthPerSr < 0.0 ||
        !std::isfinite(emission.energyWeightJ) ||
        emission.energyWeightJ < 0.0f)
    {
        throw std::invalid_argument("Invalid return energy input.");
    }

    // hit 必须是 emission.ray 实际打到的表面。
    const auto geometry =
        receiver.EvaluateVisibleGeometry(emission, hit, world);

    if (!geometry)
        return std::nullopt;

    const Vector3f ng = hit.geometricNormal.normalize();
    const double cosIncident = ng.dot(-emission.ray.dir);
    const double cosReturn =
        ng.dot(geometry->surfaceToReceiver);

    // 此基线只处理海面正面的反射。
    if (cosIncident <= 0.0 || cosReturn <= 0.0)
        return std::nullopt;

    const double solidAngle =
        receiver.CollectionSolidAngle(*geometry);

    const double receivedEnergy =
        static_cast<double>(emission.energyWeightJ) *
        brdfAtWavelengthPerSr *
        cosReturn *
        solidAngle *
        receiver.OpticalEfficiency();

    if (!std::isfinite(receivedEnergy))
        throw std::overflow_error("Return energy overflow.");

    if (receivedEnergy <= 0.0)
        return std::nullopt;

    return LidarReturnSample{
        receivedEnergy,
        geometry->arrivalTimeSeconds,
        emission.wavelengthNm
    };
}