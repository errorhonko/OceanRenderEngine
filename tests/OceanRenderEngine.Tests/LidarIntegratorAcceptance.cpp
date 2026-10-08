#include "LidarIntegrator.h"
#include "LambertianLidarScattering.h"

#include "HittableList.h"
#include "IndependentSampler.h"
#include "Sphere.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

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

class SequenceSampler final : public Sampler
{
public:
    explicit SequenceSampler(
        std::vector<Point2f> samples)
        : samples(std::move(samples))
    {
    }

    float Get1D() override
    {
        throw std::runtime_error(
            "LiDAR pulse unexpectedly requested a 1D sample.");
    }

    Point2f Get2D() override
    {
        if (nextSample >= samples.size())
        {
            throw std::runtime_error(
                "LiDAR sample sequence is exhausted.");
        }

        return samples[nextSample++];
    }

private:
    std::vector<Point2f> samples;
    std::size_t nextSample = 0;
};
}

void RunLidarIntegratorAcceptanceTests()
{
    constexpr double speedOfLight = 299792458.0;
    constexpr double lambertBrdf =
        0.5 / 3.14159265358979323846;

    const LambertianLidarScattering lambertScattering(0.5);

    const Vector3f sensorPosition(
        0.0f,
        15.0f,
        0.0f);

    const auto surface =
        std::make_shared<Sphere>(
            Vector3f(0.0f, -1.0f, 0.0f),
            1.0f,
            nullptr);

    HittableList world(surface);

    LaserEmitter pencilEmitter(
        sensorPosition,
        Vector3f(0.0f, -1.0f, 0.0f),
        532.0f,
        1.0e-3f,
        0.0f);

    LidarReceiver receiver(
        sensorPosition,
        Vector3f(0.0f, -1.0f, 0.0f),
        0.25f,
        0.01f,
        0.8f);

    LidarIntegrator oneSampleIntegrator(
        world,
        pencilEmitter,
        receiver,
        1,
        lambertScattering);

    SequenceSampler oneSampleSampler({
        Point2f(0.25f, 0.75f)
    });

    const LidarPulseResult oneSample =
        oneSampleIntegrator.SimulatePulse(
            oneSampleSampler);

    const double expectedEnergy =
        1.0e-3 *
        lambertBrdf *
        (0.01 / (15.0 * 15.0)) *
        0.8;

    ExpectTrue(
        "lidar integrator one-sample pulse",
        oneSample.emittedRayCount == 1 &&
        oneSample.surfaceHitCount == 1 &&
        oneSample.returns.size() == 1 &&
        Near(oneSample.TotalReceivedEnergyJ(),
             expectedEnergy, 1e-13) &&
        Near(oneSample.returns[0].arrivalTimeSeconds,
             30.0 / speedOfLight, 1e-14) &&
        Near(oneSample.returns[0].wavelengthNm,
             532.0, 0.0));

    constexpr std::size_t repeatedSampleCount = 8;

    LidarIntegrator repeatedIntegrator(
        world,
        pencilEmitter,
        receiver,
        repeatedSampleCount,
        lambertScattering);

    SequenceSampler repeatedSampler(
        std::vector<Point2f>(
            repeatedSampleCount,
            Point2f(0.5f, 0.5f)));

    const LidarPulseResult repeated =
        repeatedIntegrator.SimulatePulse(
            repeatedSampler);

    ExpectTrue(
        "lidar integrator sample-count invariance",
        repeated.emittedRayCount == repeatedSampleCount &&
        repeated.surfaceHitCount == repeatedSampleCount &&
        repeated.returns.size() == repeatedSampleCount &&
        Near(repeated.TotalReceivedEnergyJ(),
             expectedEnergy, 1e-13) &&
        Near(repeated.returns[0].receivedEnergyJ,
             expectedEnergy /
                 static_cast<double>(repeatedSampleCount),
             1e-14));

    constexpr float emissionTime = 2.0e-6f;
    SequenceSampler delayedSampler({
        Point2f(0.5f, 0.5f)
    });

    const LidarPulseResult delayed =
        oneSampleIntegrator.SimulatePulse(
            delayedSampler,
            emissionTime);

    ExpectTrue(
        "lidar integrator preserves emission time",
        delayed.returns.size() == 1 &&
        Near(delayed.returns[0].arrivalTimeSeconds,
             static_cast<double>(emissionTime) +
                 30.0 / speedOfLight,
             1e-14));

    LaserEmitter coneEmitter(
        sensorPosition,
        Vector3f(0.0f, -1.0f, 0.0f),
        532.0f,
        1.0e-3f,
        0.2f);

    LidarIntegrator hitAndMissIntegrator(
        world,
        coneEmitter,
        receiver,
        2,
        lambertScattering);

    // u.x == 0 samples the cone axis and hits the sphere.
    // u.x == 1 samples the cone boundary and misses it.
    SequenceSampler hitAndMissSampler({
        Point2f(0.0f, 0.0f),
        Point2f(1.0f, 0.0f)
    });

    const LidarPulseResult hitAndMiss =
        hitAndMissIntegrator.SimulatePulse(
            hitAndMissSampler);

    ExpectTrue(
        "lidar integrator misses remain in denominator",
        hitAndMiss.emittedRayCount == 2 &&
        hitAndMiss.surfaceHitCount == 1 &&
        hitAndMiss.returns.size() == 1 &&
        Near(hitAndMiss.TotalReceivedEnergyJ(),
             expectedEnergy / 2.0,
             1e-13));

    LidarIntegrator reproducibleIntegrator(
        world,
        coneEmitter,
        receiver,
        64,
        lambertScattering);

    IndependentSampler firstSampler(12345);
    IndependentSampler secondSampler(12345);

    const LidarPulseResult firstRun =
        reproducibleIntegrator.SimulatePulse(
            firstSampler);

    const LidarPulseResult secondRun =
        reproducibleIntegrator.SimulatePulse(
            secondSampler);

    bool matchingReturns =
        firstRun.returns.size() ==
        secondRun.returns.size();

    if (matchingReturns)
    {
        for (std::size_t i = 0;
             i < firstRun.returns.size();
             ++i)
        {
            matchingReturns =
                Near(firstRun.returns[i].receivedEnergyJ,
                     secondRun.returns[i].receivedEnergyJ,
                     0.0) &&
                Near(firstRun.returns[i].arrivalTimeSeconds,
                     secondRun.returns[i].arrivalTimeSeconds,
                     0.0);

            if (!matchingReturns)
                break;
        }
    }

    ExpectTrue(
        "lidar integrator fixed-seed reproducibility",
        firstRun.surfaceHitCount ==
            secondRun.surfaceHitCount &&
        matchingReturns);

    ExpectThrows(
        "lidar integrator rejects zero sample count",
        [&]
        {
            LidarIntegrator invalid(
                world,
                pencilEmitter,
                receiver,
                0,
                lambertScattering);
        });

    SequenceSampler invalidTimeSampler({
        Point2f(0.5f, 0.5f)
    });

    ExpectThrows(
        "lidar integrator rejects invalid emission time",
        [&]
        {
            oneSampleIntegrator.SimulatePulse(
                invalidTimeSampler,
                -1.0f);
        });
}
