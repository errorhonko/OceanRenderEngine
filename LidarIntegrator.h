#pragma once
#include "Hittable.h"
#include "LaserEmitter.h"
#include "LidarReceiver.h"
#include "LidarReturnEstimator.h"
#include "Sampler.h"

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

struct LidarPulseResult
{
    std::size_t emittedRayCount = 0;
    std::size_t surfaceHitCount = 0;

    // 每条回波的能量已经除以 emittedRayCount。
    std::vector<LidarReturnSample> returns;

    double TotalReceivedEnergyJ() const
    {
        double total = 0.0;

        for (const auto& sample : returns)
            total += sample.receivedEnergyJ;

        return total;
    }
};

class LidarIntegrator
{
public:
    LidarIntegrator(
        const Hittable& world,
        LaserEmitter emitter,
        LidarReceiver receiver,
        std::size_t samplesPerPulse,
        const LidarScatteringModel& scatteringModel)
        : world(world),
        emitter(std::move(emitter)),
        receiver(std::move(receiver)),
        scatteringModel(scatteringModel),
        samplesPerPulse(samplesPerPulse)
    {
        if (samplesPerPulse == 0)
        {
            throw std::invalid_argument(
                "LiDAR samples per pulse must be positive.");
        }
    }

    LidarPulseResult SimulatePulse(
        Sampler& sampler,
        double emissionTimeSeconds = 0.0) const
    {
        if (!std::isfinite(emissionTimeSeconds) ||
            emissionTimeSeconds < 0.0)
        {
            throw std::invalid_argument(
                "LiDAR emission time is invalid.");
        }

        LidarPulseResult result;
        result.emittedRayCount = samplesPerPulse;
        result.returns.reserve(samplesPerPulse);

        const double inverseSampleCount =
            1.0 / static_cast<double>(samplesPerPulse);

        for (std::size_t i = 0;
            i < samplesPerPulse;
            ++i)
        {
            const LaserEmissionSample emission =
                emitter.SampleRay(
                    sampler.Get2D(),
                    emissionTimeSeconds);

            HitRecord hit;

            if (!world.hit(
                emission.ray,
                1e-4f,
                std::numeric_limits<float>::infinity(),
                hit))
            {
                continue;
            }

            ++result.surfaceHitCount;

            auto returnSample =
                EstimateSingleReturn(
                    emission,
                    hit,
                    receiver,
                    world,
                    scatteringModel);

            if (!returnSample)
                continue;

            // 蒙特卡洛估计中的 1/N。
            returnSample->receivedEnergyJ *=
                inverseSampleCount;

            result.returns.push_back(*returnSample);
        }

        return result;
    }

private:
    const Hittable& world;

    LaserEmitter emitter;
    LidarReceiver receiver;
    const LidarScatteringModel& scatteringModel;
    std::size_t samplesPerPulse = 0;

};
