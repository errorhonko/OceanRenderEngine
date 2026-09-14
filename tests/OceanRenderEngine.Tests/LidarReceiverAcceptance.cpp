#include "LidarReceiver.h"
#include "LidarReturnEstimator.h"
#include "HittableList.h"
#include "Sphere.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
void ExpectTrue(
    const std::string& testName,
    bool condition)
{
    if (!condition)
    {
        std::cerr << "[FAIL] " << testName << '\n';
        throw std::runtime_error(testName + " failed");
    }

    std::cout << "[PASS] " << testName << '\n';
}

template <typename Function>
void ExpectThrows(
    const std::string& testName,
    Function&& function)
{
    bool threw = false;

    try
    {
        function();
    }
    catch (const std::exception&)
    {
        threw = true;
    }

    ExpectTrue(testName, threw);
}

bool Near(
    double actual,
    double expected,
    double tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

bool VectorNear(
    const Vector3f& actual,
    const Vector3f& expected,
    float tolerance = 1e-5f)
{
    return
        Near(actual.x, expected.x, tolerance) &&
        Near(actual.y, expected.y, tolerance) &&
        Near(actual.z, expected.z, tolerance);
}
}

void RunLidarReceiverAcceptanceTests()
{
    std::cout << std::unitbuf;

    constexpr double speedOfLight = 299792458.0;
    const Vector3f sensorPosition(0.0f, 15.0f, 0.0f);
    const Vector3f seaPoint(0.0f, 0.0f, 0.0f);

    LaserEmitter emitter(
        sensorPosition,
        Vector3f(0.0f, -1.0f, 0.0f),
        532.0f,
        1.0e-3f,
        0.0f);

    LidarReceiver receiver(
        sensorPosition,
        Vector3f(0.0f, -2.0f, 0.0f),
        0.1f);

    ExpectTrue(
        "lidar receiver normalized configuration",
        VectorNear(receiver.Position(), sensorPosition) &&
        VectorNear(
            receiver.Direction(),
            Vector3f(0.0f, -1.0f, 0.0f)) &&
        Near(receiver.FovHalfAngleRadians(), 0.1f, 1e-6));

    ExpectTrue(
        "lidar receiver accepts inside FOV",
        receiver.Accepts(Vector3f(0.0f, -15.0f, 0.0f)) &&
        receiver.Accepts(Vector3f(0.5f, -15.0f, 0.0f)));

    ExpectTrue(
        "lidar receiver rejects outside FOV and zero direction",
        !receiver.Accepts(Vector3f(5.0f, -15.0f, 0.0f)) &&
        !receiver.Accepts(Vector3f(0.0f, 0.0f, 0.0f)));

    const LaserEmissionSample emission =
        emitter.SampleRay(Point2f(0.25f, 0.75f));

    const auto monostatic =
        receiver.EvaluateGeometry(emission, seaPoint);

    ExpectTrue(
        "lidar monostatic range and flight time",
        monostatic.has_value() &&
        Near(monostatic->outgoingDistanceMeters, 15.0, 1e-6) &&
        Near(monostatic->returnDistanceMeters, 15.0, 1e-6) &&
        Near(monostatic->pathLengthMeters, 30.0, 1e-6) &&
        Near(monostatic->timeOfFlightSeconds,
             30.0 / speedOfLight, 1e-14) &&
        Near(monostatic->arrivalTimeSeconds,
             30.0 / speedOfLight, 1e-14) &&
        VectorNear(monostatic->surfaceToReceiver,
                   Vector3f(0.0f, 1.0f, 0.0f)));

    const LaserEmissionSample delayedEmission =
        emitter.SampleRay(Point2f(0.5f, 0.5f), 2.0e-6f);

    const auto delayed =
        receiver.EvaluateGeometry(delayedEmission, seaPoint);

    ExpectTrue(
        "lidar arrival time includes emission time",
        delayed.has_value() &&
        Near(delayed->timeOfFlightSeconds,
             30.0 / speedOfLight, 1e-14) &&
        Near(delayed->arrivalTimeSeconds,
             static_cast<double>(delayedEmission.emissionTimeSeconds) +
                 30.0 / speedOfLight,
             1e-14));

    const Vector3f offsetPosition(3.0f, 15.0f, 0.0f);
    LidarReceiver bistaticReceiver(
        offsetPosition,
        seaPoint - offsetPosition,
        0.1f);

    const auto bistatic =
        bistaticReceiver.EvaluateGeometry(emission, seaPoint);

    const double expectedReturnDistance =
        std::sqrt(3.0 * 3.0 + 15.0 * 15.0);

    ExpectTrue(
        "lidar bistatic path uses separate legs",
        bistatic.has_value() &&
        Near(bistatic->outgoingDistanceMeters, 15.0, 1e-6) &&
        Near(bistatic->returnDistanceMeters,
             expectedReturnDistance, 2e-6) &&
        Near(bistatic->pathLengthMeters,
             15.0 + expectedReturnDistance, 2e-6) &&
        Near(bistatic->timeOfFlightSeconds,
             (15.0 + expectedReturnDistance) / speedOfLight,
             1e-14) &&
        VectorNear(bistatic->surfaceToReceiver,
                   (offsetPosition - seaPoint).normalize()));

    const Vector3f visibilityEmitterPosition(
        0.0f, 10.0f, 0.0f);
    const Vector3f visibilityReceiverPosition(
        4.0f, 10.0f, 0.0f);

    LaserEmitter visibilityEmitter(
        visibilityEmitterPosition,
        Vector3f(0.0f, -1.0f, 0.0f),
        532.0f,
        1.0e-3f,
        0.0f);

    LidarReceiver visibilityReceiver(
        visibilityReceiverPosition,
        seaPoint - visibilityReceiverPosition,
        0.1f);

    const LaserEmissionSample visibilityEmission =
        visibilityEmitter.SampleRay(
            Point2f(0.5f, 0.5f));

    const auto surface =
        std::make_shared<Sphere>(
            Vector3f(0.0f, -1.0f, 0.0f),
            1.0f,
            nullptr);

    HittableList clearReturnWorld(surface);
    HitRecord surfaceHit;

    const bool outboundHit =
        clearReturnWorld.hit(
            visibilityEmission.ray,
            1e-4f,
            std::numeric_limits<float>::infinity(),
            surfaceHit);

    ExpectTrue(
        "lidar visibility fixture hits surface",
        outboundHit &&
        VectorNear(surfaceHit.point, seaPoint) &&
        VectorNear(
            surfaceHit.geometricNormal,
            Vector3f(0.0f, 1.0f, 0.0f)));

    const auto clearReturn =
        visibilityReceiver.EvaluateVisibleGeometry(
            visibilityEmission,
            surfaceHit,
            clearReturnWorld);

    const double expectedVisibleReturnDistance =
        std::sqrt(4.0 * 4.0 + 10.0 * 10.0);

    ExpectTrue(
        "lidar visible return avoids surface self-shadow",
        clearReturn.has_value() &&
        Near(clearReturn->outgoingDistanceMeters,
             10.0, 1e-6) &&
        Near(clearReturn->returnDistanceMeters,
             expectedVisibleReturnDistance, 1e-6) &&
        Near(clearReturn->timeOfFlightSeconds,
             (10.0 + expectedVisibleReturnDistance) /
                 speedOfLight,
             1e-14));

    HittableList blockedReturnWorld(surface);
    blockedReturnWorld.add(
        std::make_shared<Sphere>(
            Vector3f(2.0f, 5.0f, 0.0f),
            0.5f,
            nullptr));

    HitRecord blockedWorldOutboundHit;
    const bool outboundStillHitsSurface =
        blockedReturnWorld.hit(
            visibilityEmission.ray,
            1e-4f,
            std::numeric_limits<float>::infinity(),
            blockedWorldOutboundHit);

    ExpectTrue(
        "lidar return blocker leaves outbound path clear",
        outboundStillHitsSurface &&
        VectorNear(
            blockedWorldOutboundHit.point,
            seaPoint));

    ExpectTrue(
        "lidar return blocker rejects visible geometry",
        !visibilityReceiver.EvaluateVisibleGeometry(
            visibilityEmission,
            blockedWorldOutboundHit,
            blockedReturnWorld).has_value());

    HittableList beyondReceiverWorld(surface);
    beyondReceiverWorld.add(
        std::make_shared<Sphere>(
            Vector3f(4.5f, 11.25f, 0.0f),
            0.1f,
            nullptr));

    ExpectTrue(
        "lidar return ray stops at receiver",
        visibilityReceiver.EvaluateVisibleGeometry(
            visibilityEmission,
            surfaceHit,
            beyondReceiverWorld).has_value());

    HitRecord invalidNormalHit = surfaceHit;
    invalidNormalHit.geometricNormal =
        Vector3f(0.0f, 0.0f, 0.0f);

    ExpectTrue(
        "lidar return rejects zero geometric normal",
        !visibilityReceiver.EvaluateVisibleGeometry(
            visibilityEmission,
            invalidNormalHit,
            clearReturnWorld).has_value());

    ExpectTrue(
        "lidar receiver rejects surface outside FOV",
        !receiver.EvaluateGeometry(
            emission,
            Vector3f(5.0f, 0.0f, 0.0f)).has_value());

    ExpectTrue(
        "lidar receiver rejects coincident surface",
        !receiver.EvaluateGeometry(
            emission,
            sensorPosition).has_value());

    const float nan =
        std::numeric_limits<float>::quiet_NaN();

    LidarReceiver apertureReceiver(
        sensorPosition,
        Vector3f(0.0f, -1.0f, 0.0f),
        0.5f,
        0.01f,
        0.8f);

    ExpectTrue(
        "lidar receiver aperture configuration",
        Near(apertureReceiver.ApertureAreaSquareMeters(),
             0.01f, 1e-8) &&
        Near(apertureReceiver.OpticalEfficiency(),
             0.8f, 1e-6) &&
        Near(receiver.ApertureAreaSquareMeters(),
             0.0, 0.0) &&
        Near(receiver.OpticalEfficiency(),
             1.0, 0.0));

    ExpectTrue(
        "lidar on-axis collection solid angle",
        monostatic.has_value() &&
        Near(apertureReceiver.CollectionSolidAngle(*monostatic),
             static_cast<double>(
                 apertureReceiver.ApertureAreaSquareMeters()) /
                 (15.0 * 15.0),
             1e-10) &&
        Near(receiver.CollectionSolidAngle(*monostatic),
             0.0, 0.0));

    const Vector3f offAxisSeaPoint(3.0f, 0.0f, 0.0f);
    const auto offAxis = apertureReceiver.EvaluateGeometry(
        emission,
        offAxisSeaPoint);
    const double offAxisRangeSquared = 3.0 * 3.0 + 15.0 * 15.0;
    const double offAxisCosine =
        15.0 / std::sqrt(offAxisRangeSquared);

    ExpectTrue(
        "lidar off-axis collection includes aperture cosine",
        offAxis.has_value() &&
        Near(apertureReceiver.CollectionSolidAngle(*offAxis),
             static_cast<double>(
                 apertureReceiver.ApertureAreaSquareMeters()) *
                 offAxisCosine / offAxisRangeSquared,
             1e-9));

    LidarReceiver doubleApertureReceiver(
        sensorPosition,
        Vector3f(0.0f, -1.0f, 0.0f),
        0.5f,
        0.02f,
        0.0f);

    ExpectTrue(
        "lidar solid angle scales with area not efficiency",
        offAxis.has_value() &&
        Near(doubleApertureReceiver.CollectionSolidAngle(*offAxis),
             2.0 * apertureReceiver.CollectionSolidAngle(*offAxis),
             1e-9) &&
        Near(doubleApertureReceiver.OpticalEfficiency(),
             0.0, 0.0));

    LidarReturnGeometry outsideFov = *monostatic;
    outsideFov.surfaceToReceiver =
        Vector3f(-1.0f, 0.0f, 0.0f);

    LidarReturnGeometry invalidRange = *monostatic;
    invalidRange.returnDistanceMeters = -1.0;

    LidarReturnGeometry nonfiniteRange = *monostatic;
    nonfiniteRange.returnDistanceMeters =
        std::numeric_limits<double>::quiet_NaN();

    LidarReturnGeometry invalidDirection = *monostatic;
    invalidDirection.surfaceToReceiver =
        Vector3f(0.0f, 0.0f, 0.0f);

    ExpectTrue(
        "lidar collection rejects outside FOV and invalid geometry",
        monostatic.has_value() &&
        Near(apertureReceiver.CollectionSolidAngle(outsideFov),
             0.0, 0.0) &&
        Near(apertureReceiver.CollectionSolidAngle(invalidRange),
             0.0, 0.0) &&
        Near(apertureReceiver.CollectionSolidAngle(nonfiniteRange),
             0.0, 0.0) &&
        Near(apertureReceiver.CollectionSolidAngle(invalidDirection),
             0.0, 0.0));

    ExpectThrows(
        "lidar receiver rejects negative aperture area",
        []
        {
            LidarReceiver invalid(
                Vector3f(0.0f, 0.0f, 0.0f),
                Vector3f(0.0f, -1.0f, 0.0f),
                0.1f,
                -0.01f);
        });

    ExpectThrows(
        "lidar receiver rejects nonfinite aperture area",
        [nan]
        {
            LidarReceiver invalid(
                Vector3f(0.0f, 0.0f, 0.0f),
                Vector3f(0.0f, -1.0f, 0.0f),
                0.1f,
                nan);
        });

    ExpectThrows(
        "lidar receiver rejects negative efficiency",
        []
        {
            LidarReceiver invalid(
                Vector3f(0.0f, 0.0f, 0.0f),
                Vector3f(0.0f, -1.0f, 0.0f),
                0.1f,
                0.01f,
                -0.1f);
        });

    ExpectThrows(
        "lidar receiver rejects efficiency outside unit interval",
        []
        {
            LidarReceiver invalid(
                Vector3f(0.0f, 0.0f, 0.0f),
                Vector3f(0.0f, -1.0f, 0.0f),
                0.1f,
                0.01f,
                1.1f);
        });

    ExpectThrows(
        "lidar receiver rejects nonfinite efficiency",
        [nan]
        {
            LidarReceiver invalid(
                Vector3f(0.0f, 0.0f, 0.0f),
                Vector3f(0.0f, -1.0f, 0.0f),
                0.1f,
                0.01f,
                nan);
        });

    ExpectThrows(
        "lidar receiver rejects invalid position",
        [nan]
        {
            LidarReceiver invalid(
                Vector3f(nan, 0.0f, 0.0f),
                Vector3f(0.0f, -1.0f, 0.0f),
                0.1f);
        });

    ExpectThrows(
        "lidar receiver rejects zero direction",
        []
        {
            LidarReceiver invalid(
                Vector3f(0.0f, 0.0f, 0.0f),
                Vector3f(0.0f, 0.0f, 0.0f),
                0.1f);
        });

    ExpectThrows(
        "lidar receiver rejects invalid FOV",
        []
        {
            LidarReceiver invalid(
                Vector3f(0.0f, 0.0f, 0.0f),
                Vector3f(0.0f, -1.0f, 0.0f),
                PiOver2);
        });

    ExpectThrows(
        "lidar receiver rejects invalid surface point",
        [&receiver, &emission, nan]
        {
            receiver.EvaluateGeometry(
                emission,
                Vector3f(nan, 0.0f, 0.0f));
        });

    HitRecord energyHit;
    const bool energyRayHit = clearReturnWorld.hit(
        emission.ray,
        1e-4f,
        std::numeric_limits<float>::infinity(),
        energyHit);

    ExpectTrue(
        "lidar return energy fixture hits surface",
        energyRayHit &&
        VectorNear(energyHit.point, seaPoint) &&
        VectorNear(energyHit.geometricNormal,
                   Vector3f(0.0f, 1.0f, 0.0f)));

    const double lambertBrdf =
        0.5 / std::acos(-1.0);

    const auto onAxisEnergy = EstimateSingleReturn(
        emission,
        energyHit,
        apertureReceiver,
        clearReturnWorld,
        lambertBrdf);

    const double expectedOnAxisEnergy =
        1.0e-3 * lambertBrdf *
        (0.01 / (15.0 * 15.0)) * 0.8;

    ExpectTrue(
        "lidar return energy on-axis Lambert baseline",
        onAxisEnergy.has_value() &&
        Near(onAxisEnergy->receivedEnergyJ,
             expectedOnAxisEnergy, 1e-13) &&
        Near(onAxisEnergy->arrivalTimeSeconds,
             30.0 / speedOfLight, 1e-14) &&
        Near(onAxisEnergy->wavelengthNm,
             532.0, 0.0));

    const auto delayedEnergy = EstimateSingleReturn(
        delayedEmission,
        energyHit,
        apertureReceiver,
        clearReturnWorld,
        lambertBrdf);

    ExpectTrue(
        "lidar return energy preserves emission time",
        delayedEnergy.has_value() &&
        Near(delayedEnergy->receivedEnergyJ,
             expectedOnAxisEnergy, 1e-13) &&
        Near(delayedEnergy->arrivalTimeSeconds,
             static_cast<double>(
                 delayedEmission.emissionTimeSeconds) +
                 30.0 / speedOfLight,
             1e-14));

    LidarReceiver bistaticEnergyReceiver(
        visibilityReceiverPosition,
        seaPoint - visibilityReceiverPosition,
        0.1f,
        0.01f,
        0.8f);

    const auto bistaticEnergy = EstimateSingleReturn(
        visibilityEmission,
        surfaceHit,
        bistaticEnergyReceiver,
        clearReturnWorld,
        lambertBrdf);

    const double bistaticRangeSquared =
        4.0 * 4.0 + 10.0 * 10.0;
    const double expectedBistaticEnergy =
        1.0e-3 * lambertBrdf *
        (10.0 / std::sqrt(bistaticRangeSquared)) *
        (0.01 / bistaticRangeSquared) * 0.8;

    ExpectTrue(
        "lidar return energy includes surface cosine",
        bistaticEnergy.has_value() &&
        Near(bistaticEnergy->receivedEnergyJ,
             expectedBistaticEnergy, 1e-13) &&
        Near(bistaticEnergy->arrivalTimeSeconds,
             (10.0 + std::sqrt(bistaticRangeSquared)) /
                 speedOfLight,
             1e-14));

    ExpectTrue(
        "lidar return energy rejects blocked return",
        !EstimateSingleReturn(
            visibilityEmission,
            blockedWorldOutboundHit,
            bistaticEnergyReceiver,
            blockedReturnWorld,
            lambertBrdf).has_value());

    LidarReceiver zeroEfficiencyReceiver(
        sensorPosition,
        Vector3f(0.0f, -1.0f, 0.0f),
        0.1f,
        0.01f,
        0.0f);

    ExpectTrue(
        "lidar return energy rejects zero collection",
        !EstimateSingleReturn(
            emission,
            energyHit,
            receiver,
            clearReturnWorld,
            lambertBrdf).has_value() &&
        !EstimateSingleReturn(
            emission,
            energyHit,
            zeroEfficiencyReceiver,
            clearReturnWorld,
            lambertBrdf).has_value() &&
        !EstimateSingleReturn(
            emission,
            energyHit,
            apertureReceiver,
            clearReturnWorld,
            0.0).has_value());

    HitRecord backFacingHit = energyHit;
    backFacingHit.geometricNormal =
        -backFacingHit.geometricNormal;

    ExpectTrue(
        "lidar return energy rejects back-facing surface",
        !EstimateSingleReturn(
            emission,
            backFacingHit,
            apertureReceiver,
            clearReturnWorld,
            lambertBrdf).has_value());

    ExpectThrows(
        "lidar return energy rejects negative BRDF",
        [&]
        {
            EstimateSingleReturn(
                emission,
                energyHit,
                apertureReceiver,
                clearReturnWorld,
                -0.1);
        });

    ExpectThrows(
        "lidar return energy rejects nonfinite BRDF",
        [&]
        {
            EstimateSingleReturn(
                emission,
                energyHit,
                apertureReceiver,
                clearReturnWorld,
                std::numeric_limits<double>::quiet_NaN());
        });
}
