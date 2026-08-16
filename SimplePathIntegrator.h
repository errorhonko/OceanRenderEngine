#pragma once
#include "Integrator.h"
#include "UniformLightSampler.h"
class SimplePathIntegrator
	:public RayIntegrator
{
public:
    SimplePathIntegrator(
        int maxDepth,
        bool sampleLights,
        bool sampleBSDF,
        const Hittable& world,
        Camera camera,
        std::shared_ptr<Sampler> sampler,
        std::vector<std::shared_ptr<Light>> lights)
        : RayIntegrator(
            world,
            std::move(camera),
            std::move(sampler)),
        maxDepth(maxDepth),
        sampleLights(sampleLights),
        sampleBSDF(sampleBSDF),
        lightSampler(std::move(lights))
    {
    }

    Spectrum Li(
        Ray ray,
        Sampler& sampler) const override;

private:
    int maxDepth;
    bool sampleLights;
    bool sampleBSDF;

    UniformLightSampler lightSampler;
};