#include "SimplePathIntegrator.h"
#include "IndependentSampler.h"
#include "DiffuseMaterial.h"
#include "HittableList.h"
#include "Sphere.h"
#include "PointLight.h"
#include "DistantLight.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include "UniformInfiniteLight.h"
#include <utility>
Spectrum Trace(
    const Hittable& world,
    int maxDepth,
    const Ray& ray,
    std::vector<std::shared_ptr<Light>> lights = {},
    std::uint32_t seed = 42)
{
    Camera camera(800, 600);

    auto sampler =
        std::make_shared<IndependentSampler>(seed);

    SimplePathIntegrator integrator(
        maxDepth,
        false,
        true,
        world,
        camera,
        sampler,
        std::move(lights));

    return integrator.Li(ray, *sampler);
}
Spectrum TraceDirect(
    const Hittable& world,
    const Ray& ray,
    std::vector<std::shared_ptr<Light>> lights,
    std::uint32_t seed = 42)
{
    Camera camera(800, 600);

    auto sampler =
        std::make_shared<IndependentSampler>(seed);

    SimplePathIntegrator integrator(
        1,      // 允许处理第一个表面
        true,   // 开启灯光采样
        false,  // 不继续采样 BSDF
        world,
        camera,
        sampler,
        std::move(lights));

    return integrator.Li(ray, *sampler);
}

class EnvironmentSequenceSampler final : public Sampler
{
public:
    float Get1D() override
    {
        if (oneDCount++ != 0)
            throw std::runtime_error(
                "unexpected extra 1D environment sample");

        // Select the only light in the scene.
        return 0.0f;
    }

    Point2f Get2D() override
    {
        if (twoDCount == 0)
        {
            ++twoDCount;
            // Uniform sphere north pole: wi = (0, 0, 1).
            return Point2f(0.0f, 0.0f);
        }

        if (twoDCount == 1)
        {
            ++twoDCount;
            // Cosine hemisphere center: BSDF wi = (0, 0, 1).
            return Point2f(0.5f, 0.5f);
        }

        throw std::runtime_error(
            "unexpected extra 2D environment sample");
    }

private:
    int oneDCount = 0;
    int twoDCount = 0;
};

Spectrum TraceWithLightAndBSDFSampling(
    const Hittable& world,
    const Ray& ray,
    std::vector<std::shared_ptr<Light>> lights)
{
    Camera camera(800, 600);

    auto sampler =
        std::make_shared<EnvironmentSequenceSampler>();

    SimplePathIntegrator integrator(
        1,
        true,
        true,
        world,
        camera,
        sampler,
        std::move(lights));

    return integrator.Li(ray, *sampler);
}

void ExpectRGBNear(
    const std::string& testName,
    const Spectrum& actual,
    float expected,
    float epsilon = 1e-4f)
{
    float rgb[3];
    actual.ToRGB(rgb);

    for (int channel = 0; channel < 3; ++channel)
    {
        if (std::fabs(rgb[channel] - expected) > epsilon)
        {
            throw std::runtime_error(
                testName + " failed at channel " +
                std::to_string(channel) +
                ", actual = " +
                std::to_string(rgb[channel]));
        }
    }

    std::cout << "[PASS] " << testName << '\n';
}

void ExpectUniformInfiniteLightSample()
{
    UniformInfiniteLight environment(Spectrum(0.25f));

    LightSampleContext context{
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 0.0f, 1.0f),
        Vector3f(0.0f, 0.0f, 1.0f)
    };

    auto sample = environment.SampleLi(
        context,
        Point2f(0.0f, 0.0f));

    if (!sample)
        throw std::runtime_error(
            "uniform infinite light returned no sample");

    ExpectRGBNear(
        "environment sample radiance",
        sample->L,
        0.25f);

    constexpr float epsilon = 1e-4f;
    bool directionIsNorthPole =
        std::fabs(sample->wi.x) <= epsilon &&
        std::fabs(sample->wi.y) <= epsilon &&
        std::fabs(sample->wi.z - 1.0f) <= epsilon;
    bool directionIsUnitLength =
        std::fabs(sample->wi.norm() - 1.0f) <= epsilon;
    bool pdfIsUniformSphere =
        std::fabs(sample->pdf - Inv4Pi) <= epsilon;
    bool distanceIsInfinite =
        std::isinf(sample->distance);

    if (!directionIsNorthPole ||
        !directionIsUnitLength ||
        !pdfIsUniformSphere ||
        !distanceIsInfinite)
    {
        throw std::runtime_error(
            "uniform infinite light sample geometry failed");
    }

    std::cout <<
        "[PASS] environment sample geometry\n";
}

void ExpectSphereSurfaceSample(
    const std::shared_ptr<Material>& material)
{
    Sphere sphere(
        Vector3f(1.0f, 2.0f, 3.0f),
        2.0f,
        material);

    constexpr float epsilon = 1e-4f;
    float expectedArea = 16.0f * Pi;

    if (std::fabs(
            sphere.SurfaceArea() - expectedArea) > epsilon)
    {
        throw std::runtime_error(
            "sphere surface area failed");
    }

    auto northPole =
        sphere.Sample(Point2f(0.0f, 0.0f));

    if (!northPole)
        throw std::runtime_error(
            "sphere returned no north-pole sample");

    bool northPositionCorrect =
        std::fabs(northPole->p.x - 1.0f) <= epsilon &&
        std::fabs(northPole->p.y - 2.0f) <= epsilon &&
        std::fabs(northPole->p.z - 5.0f) <= epsilon;
    bool northNormalCorrect =
        std::fabs(northPole->n.x) <= epsilon &&
        std::fabs(northPole->n.y) <= epsilon &&
        std::fabs(northPole->n.z - 1.0f) <= epsilon;
    bool areaPdfCorrect =
        std::fabs(
            northPole->pdfArea - 1.0f / expectedArea) <=
        epsilon;

    if (!northPositionCorrect ||
        !northNormalCorrect ||
        !areaPdfCorrect)
    {
        throw std::runtime_error(
            "sphere north-pole surface sample failed");
    }

    // u.x = 0.5 maps to z = 0, while u.y = 0 maps to phi = 0.
    auto equator =
        sphere.Sample(Point2f(0.5f, 0.0f));

    if (!equator)
        throw std::runtime_error(
            "sphere returned no equator sample");

    bool equatorPositionCorrect =
        std::fabs(equator->p.x - 3.0f) <= epsilon &&
        std::fabs(equator->p.y - 2.0f) <= epsilon &&
        std::fabs(equator->p.z - 3.0f) <= epsilon;
    bool equatorNormalCorrect =
        std::fabs(equator->n.x - 1.0f) <= epsilon &&
        std::fabs(equator->n.y) <= epsilon &&
        std::fabs(equator->n.z) <= epsilon;

    if (!equatorPositionCorrect ||
        !equatorNormalCorrect)
    {
        throw std::runtime_error(
            "sphere equator surface sample failed");
    }

    Sphere invalidSphere(
        Vector3f(0.0f, 0.0f, 0.0f),
        0.0f,
        material);

    if (invalidSphere.Sample(Point2f(0.25f, 0.75f)))
        throw std::runtime_error(
            "zero-radius sphere returned a surface sample");

    std::cout <<
        "[PASS] sphere surface sampling\n";
}

int main()
{
    Ray centerRay(
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 0.0f, -1.0f));

    std::vector<std::shared_ptr<Light>> whiteEnvironment{
        std::make_shared<UniformInfiniteLight>(
            Spectrum(1.0f))
    };

    ExpectUniformInfiniteLightSample();

    // 1. 光线直接进入单位白色环境
    HittableList emptyWorld;

    ExpectRGBNear(
        "environment",
        Trace(
            emptyWorld,
            1,
            centerRay,
            whiteEnvironment),
        1.0f);

    ExpectRGBNear(
        "no environment",
        Trace(emptyWorld, 1, centerRay),
        0.0f);

    // 2. 一次反照率为 0.5 的漫反射
    HittableList diffuseWorld;

    auto material =
        std::make_shared<DiffuseMaterial>(
            Spectrum(0.5f));

    ExpectSphereSurfaceSample(material);

    diffuseWorld.add(
        std::make_shared<Sphere>(
            Vector3f(0.0f, 0.0f, -1.0f),
            0.5f,
            material));

    ExpectRGBNear(
        "diffuse throughput",
        Trace(
            diffuseWorld,
            1,
            centerRay,
            whiteEnvironment),
        0.5f);

    std::vector<std::shared_ptr<Light>> quarterEnvironment{
        std::make_shared<UniformInfiniteLight>(
            Spectrum(0.25f))
    };

    // Direct environment sample contributes 0.5. The following
    // diffuse BSDF escape must not add the environment a second time.
    ExpectRGBNear(
        "environment no double count",
        TraceWithLightAndBSDFSampling(
            diffuseWorld,
            centerRay,
            quarterEnvironment),
        0.5f);

    // 3. maxDepth=0，不发生表面弹射
    ExpectRGBNear(
        "zero depth",
        Trace(
            diffuseWorld,
            0,
            centerRay,
            whiteEnvironment),
        0.0f);

    std::vector<std::shared_ptr<Light>> nearLights{
    std::make_shared<PointLight>(
        Vector3f(0.0f, 0.0f, 0.5f),
        Spectrum(Pi))
    };

    ExpectRGBNear(
        "point light direct",
        TraceDirect(diffuseWorld, centerRay, nearLights),
        0.5f);

    std::vector<std::shared_ptr<Light>> farLights{
    std::make_shared<PointLight>(
        Vector3f(0.0f, 0.0f, 1.5f),
        Spectrum(Pi))
    };

    ExpectRGBNear(
        "point light inverse square",
        TraceDirect(diffuseWorld, centerRay, farLights),
        0.125f);

    HittableList blockedWorld;

    blockedWorld.add(
        std::make_shared<Sphere>(
            Vector3f(0.0f, 0.0f, -1.0f),
            0.5f,
            material));

    blockedWorld.add(
        std::make_shared<Sphere>(
            Vector3f(0.0f, 0.5f, 0.0f),
            0.1f,
            material));

    std::vector<std::shared_ptr<Light>> blockedLights{
        std::make_shared<PointLight>(
            Vector3f(0.0f, 1.0f, 0.5f),
            Spectrum(2.0f * std::sqrt(2.0f) * Pi))
    };

    ExpectRGBNear(
        "point light occluded",
        TraceDirect(blockedWorld, centerRay, blockedLights),
        0.0f);
    std::vector<std::shared_ptr<Light>> distantLights{
    std::make_shared<DistantLight>(
        Vector3f(0.0f, 0.0f, 1.0f),
        Spectrum(Pi))
    };

    ExpectRGBNear(
        "distant light direct",
        TraceDirect(
            diffuseWorld,
            centerRay,
            distantLights),
        0.5f);

    HittableList distantBlockedWorld;

    // 主射线命中的漫反射球
    distantBlockedWorld.add(
        std::make_shared<Sphere>(
            Vector3f(0.0f, 0.0f, -1.0f),
            0.5f,
            material));

    // 位于交点朝太阳方向上的遮挡球
    distantBlockedWorld.add(
        std::make_shared<Sphere>(
            Vector3f(0.0f, 0.0f, 0.5f),
            0.1f,
            material));

    ExpectRGBNear(
        "distant light occluded",
        TraceDirect(
            distantBlockedWorld,
            centerRay,
            distantLights),
        0.0f);

    std::cout << "All acceptance tests passed.\n";
    return 0;
}
