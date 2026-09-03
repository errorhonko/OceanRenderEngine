#include "SimplePathIntegrator.h"
#include "PathIntegrator.h"
#include "IndependentSampler.h"
#include "DiffuseMaterial.h"
#include "DielectricMaterial.h"
#include "HittableList.h"
#include "Sphere.h"
#include "Triangle.h"
#include "PointLight.h"
#include "DistantLight.h"
#include "DiffuseAreaLight.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include "UniformInfiniteLight.h"
#include <utility>

void RunElfouhailySpectrumAcceptanceTests();
void RunOceanFrequencyFieldAcceptanceTests();
void RunOceanFFTAcceptanceTests();
void RunOceanHeightFieldAcceptanceTests();
void RunMeshTriangleAcceptanceTests();
void RunOceanSurfaceMeshAcceptanceTests();

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

Spectrum TracePath(
    const Hittable& world,
    int maxDepth,
    const Ray& ray,
    std::vector<std::shared_ptr<Light>> lights = {},
    std::uint32_t seed = 42)
{
    Camera camera(800, 600);

    auto sampler =
        std::make_shared<IndependentSampler>(seed);

    PathIntegrator integrator(
        maxDepth,
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

Spectrum TraceSurfaceEmission(
    const Hittable& world,
    int maxDepth,
    const Ray& ray)
{
    Camera camera(800, 600);
    auto sampler =
        std::make_shared<IndependentSampler>(42);

    SimplePathIntegrator integrator(
        maxDepth,
        false,
        false,
        world,
        camera,
        sampler,
        {});

    return integrator.Li(ray, *sampler);
}

class OffsetProbeIntegrator final : public Integrator
{
public:
    explicit OffsetProbeIntegrator(
        const Hittable& world)
        : Integrator(world)
    {
    }

    void Render() override
    {
    }

    Vector3f Offset(
        const Vector3f& p,
        const Vector3f& geometricNormal,
        const Vector3f& direction) const
    {
        return OffsetRayOrigin(
            p,
            geometricNormal,
            direction);
    }
};

class SpecularPathSampler final : public Sampler
{
public:
    float Get1D() override
    {
        throw std::runtime_error(
            "pure specular vertex unexpectedly sampled a light");
    }

    Point2f Get2D() override
    {
        if (twoDCount++ != 0)
        {
            throw std::runtime_error(
                "pure specular path requested an extra 2D sample");
        }

        // Select the smooth dielectric reflection branch.
        return Point2f(0.0f, 0.0f);
    }

    int TwoDCount() const
    {
        return twoDCount;
    }

private:
    int twoDCount = 0;
};

class TestSpecularBxDF final : public BxDF
{
public:
    TestSpecularBxDF()
        : BxDF(static_cast<BxDFType>(
            BSDF_REFLECTION | BSDF_SPECULAR))
    {
    }

    Spectrum f(
        const Vector3f& wo,
        const Vector3f& wi,
        TransportMode mode) const override
    {
        (void)wo;
        (void)wi;
        (void)mode;
        return Spectrum(0.0f);
    }

    std::optional<BSDFSample> Sample_f(
        const Vector3f& wo,
        const Point2f& sample,
        TransportMode mode,
        BxDFReflectionType sampledType) const override
    {
        (void)sample;
        (void)mode;
        (void)sampledType;

        Vector3f wi(-wo.x, -wo.y, wo.z);
        float absCosTheta = BRDFUtils::AbsCosTheta(wi);

        if (absCosTheta == 0.0f)
            return std::nullopt;

        return BSDFSample(
            Spectrum(1.0f / absCosTheta),
            wi,
            1.0f,
            static_cast<BxDFType>(
                BSDF_REFLECTION | BSDF_SPECULAR));
    }

    Spectrum rho(
        const Vector3f& wo,
        int nSamples,
        const Point2f* samples) const override
    {
        (void)wo;
        (void)nSamples;
        (void)samples;
        return Spectrum(0.0f);
    }

    float Pdf(
        const Vector3f& wo,
        const Vector3f& wi,
        TransportMode mode,
        BxDFReflectionType sampleFlags) const override
    {
        (void)wo;
        (void)wi;
        (void)mode;
        (void)sampleFlags;
        return 0.0f;
    }
};

class TestSpecularMaterial final : public Material
{
public:
    BSDF GetBSDF(
        const MaterialEvalContext& ctx) const override
    {
        return BSDF(
            ctx.ns,
            ctx.dpdu,
            std::make_shared<TestSpecularBxDF>());
    }
};

class RouletteBxDF final : public BxDF
{
public:
    RouletteBxDF(
        float throughput,
        BxDFType sampleFlags,
        float eta = 1.0f)
        : BxDF(sampleFlags),
        throughput(throughput),
        sampleFlags(sampleFlags),
        eta(eta)
    {
    }

    Spectrum f(
        const Vector3f& wo,
        const Vector3f& wi,
        TransportMode mode) const override
    {
        (void)wo;
        (void)wi;
        (void)mode;
        return Spectrum(0.0f);
    }

    std::optional<BSDFSample> Sample_f(
        const Vector3f& wo,
        const Point2f& sample,
        TransportMode mode,
        BxDFReflectionType sampledType) const override
    {
        (void)sample;
        (void)mode;
        (void)sampledType;

        bool transmission =
            (sampleFlags & BSDF_TRANSMISSION) != 0;
        Vector3f wi = transmission
            ? -wo
            : Vector3f(-wo.x, -wo.y, wo.z);
        float absCosTheta = BRDFUtils::AbsCosTheta(wi);

        if (absCosTheta == 0.0f)
            return std::nullopt;

        return BSDFSample(
            Spectrum(throughput / absCosTheta),
            wi,
            1.0f,
            sampleFlags,
            eta);
    }

    Spectrum rho(
        const Vector3f& wo,
        int nSamples,
        const Point2f* samples) const override
    {
        (void)wo;
        (void)nSamples;
        (void)samples;
        return Spectrum(0.0f);
    }

    float Pdf(
        const Vector3f& wo,
        const Vector3f& wi,
        TransportMode mode,
        BxDFReflectionType sampleFlags) const override
    {
        (void)wo;
        (void)wi;
        (void)mode;
        (void)sampleFlags;
        return 0.0f;
    }

private:
    float throughput;
    BxDFType sampleFlags;
    float eta;
};

class RouletteMaterial final : public Material
{
public:
    RouletteMaterial(
        float throughput,
        BxDFType sampleFlags,
        float eta = 1.0f)
        : throughput(throughput),
        sampleFlags(sampleFlags),
        eta(eta)
    {
    }

    BSDF GetBSDF(
        const MaterialEvalContext& ctx) const override
    {
        return BSDF(
            ctx.ns,
            ctx.dpdu,
            std::make_shared<RouletteBxDF>(
                throughput,
                sampleFlags,
                eta));
    }

private:
    float throughput;
    BxDFType sampleFlags;
    float eta;
};

class TwoHitPathWorld final : public Hittable
{
public:
    explicit TwoHitPathWorld(
        std::vector<std::shared_ptr<Material>> materials)
        : materials(std::move(materials))
    {
    }

    bool hit(
        const Ray& ray,
        float tMin,
        float tMax,
        HitRecord& rec) const override
    {
        (void)tMin;
        (void)tMax;

        if (hitCount >= materials.size())
            return false;

        rec.t = 1.0f;
        rec.point = ray.at(rec.t);
        rec.u = 0.0f;
        rec.v = 0.0f;
        rec.material = materials[hitCount++];
        rec.dpdu = Vector3f(1.0f, 0.0f, 0.0f);
        rec.geometricNormal = Vector3f(0.0f, 0.0f, 1.0f);
        rec.normal = rec.geometricNormal;
        rec.areaLight.reset();
        return true;
    }

private:
    std::vector<std::shared_ptr<Material>> materials;
    mutable size_t hitCount = 0;
};

class RouletteSequenceSampler final : public Sampler
{
public:
    explicit RouletteSequenceSampler(
        float rouletteSample,
        bool forbidRouletteSample = false)
        : rouletteSample(rouletteSample),
        forbidRouletteSample(forbidRouletteSample)
    {
    }

    float Get1D() override
    {
        if (forbidRouletteSample)
            throw std::runtime_error(
                "etaScale path unexpectedly sampled Russian roulette");

        if (oneDCount++ != 0)
            throw std::runtime_error(
                "path requested an extra Russian roulette sample");

        return rouletteSample;
    }

    Point2f Get2D() override
    {
        if (twoDCount++ >= 2)
            throw std::runtime_error(
                "path requested an extra BSDF sample");

        return Point2f(0.5f, 0.5f);
    }

    int OneDCount() const
    {
        return oneDCount;
    }

private:
    float rouletteSample;
    bool forbidRouletteSample;
    int oneDCount = 0;
    int twoDCount = 0;
};

class AreaLightSequenceSampler final : public Sampler
{
public:
    float Get1D() override
    {
        if (oneDCount++ != 0)
            throw std::runtime_error(
                "unexpected extra 1D area-light sample");

        // Select the only light in the scene.
        return 0.0f;
    }

    Point2f Get2D() override
    {
        if (twoDCount == 0)
        {
            ++twoDCount;
            // Sample the south pole of the light sphere. It faces the receiver.
            return Point2f(1.0f, 0.0f);
        }

        if (twoDCount == 1)
        {
            ++twoDCount;
            // Cosine hemisphere center: BSDF wi follows the surface normal.
            return Point2f(0.5f, 0.5f);
        }

        throw std::runtime_error(
            "unexpected extra 2D area-light sample");
    }

private:
    int oneDCount = 0;
    int twoDCount = 0;
};

Spectrum TraceAreaLightDirect(
    const Hittable& world,
    const Ray& ray,
    std::vector<std::shared_ptr<Light>> lights)
{
    Camera camera(800, 600);
    auto sampler =
        std::make_shared<AreaLightSequenceSampler>();

    SimplePathIntegrator integrator(
        1,
        true,
        false,
        world,
        camera,
        sampler,
        std::move(lights));

    return integrator.Li(ray, *sampler);
}

Spectrum TraceAreaLightWithBSDFSampling(
    const Hittable& world,
    const Ray& ray,
    std::vector<std::shared_ptr<Light>> lights)
{
    Camera camera(800, 600);
    auto sampler =
        std::make_shared<AreaLightSequenceSampler>();

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

Spectrum TracePathAreaLight(
    const Hittable& world,
    const Ray& ray,
    std::vector<std::shared_ptr<Light>> lights)
{
    Camera camera(800, 600);
    auto sampler =
        std::make_shared<AreaLightSequenceSampler>();

    PathIntegrator integrator(
        1,
        world,
        camera,
        sampler,
        std::move(lights));

    return integrator.Li(ray, *sampler);
}

class TriangleAreaLightSampler final : public Sampler
{
public:
    float Get1D() override
    {
        if (oneDCount++ != 0)
            throw std::runtime_error(
                "unexpected extra 1D triangle-light sample");

        return 0.0f;
    }

    Point2f Get2D() override
    {
        if (twoDCount++ != 0)
            throw std::runtime_error(
                "unexpected extra 2D triangle-light sample");

        // PBRT triangle mapping sends this sample to the centroid.
        return Point2f(
            2.0f / 3.0f,
            2.0f / 3.0f);
    }

private:
    int oneDCount = 0;
    int twoDCount = 0;
};

Spectrum TraceTriangleAreaLightDirect(
    const Hittable& world,
    const Ray& ray,
    std::vector<std::shared_ptr<Light>> lights)
{
    Camera camera(800, 600);
    auto sampler =
        std::make_shared<TriangleAreaLightSampler>();

    SimplePathIntegrator integrator(
        1,
        true,
        false,
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

Spectrum TracePathEnvironment(
    const Hittable& world,
    const Ray& ray,
    std::vector<std::shared_ptr<Light>> lights)
{
    Camera camera(800, 600);

    auto sampler =
        std::make_shared<EnvironmentSequenceSampler>();

    PathIntegrator integrator(
        1,
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

void ExpectFloatNear(
    const std::string& testName,
    float actual,
    float expected,
    float epsilon = 1e-6f)
{
    if (!std::isfinite(actual) ||
        std::fabs(actual - expected) > epsilon)
    {
        throw std::runtime_error(
            testName + " failed, actual = " +
            std::to_string(actual) +
            ", expected = " +
            std::to_string(expected));
    }

    std::cout << "[PASS] " << testName << '\n';
}

void ExpectPowerHeuristic()
{
    ExpectFloatNear(
        "power heuristic equal PDFs",
        BRDFUtils::PowerHeuristic(1, 0.5f, 1, 0.5f),
        0.5f);

    ExpectFloatNear(
        "power heuristic favors larger PDF",
        BRDFUtils::PowerHeuristic(1, 0.25f, 1, 0.75f),
        0.1f);

    ExpectFloatNear(
        "power heuristic first technique only",
        BRDFUtils::PowerHeuristic(1, 0.5f, 1, 0.0f),
        1.0f);

    ExpectFloatNear(
        "power heuristic second technique only",
        BRDFUtils::PowerHeuristic(1, 0.0f, 1, 0.5f),
        0.0f);

    ExpectFloatNear(
        "power heuristic zero denominator",
        BRDFUtils::PowerHeuristic(1, 0.0f, 1, 0.0f),
        0.0f);

    float forward =
        BRDFUtils::PowerHeuristic(1, 0.25f, 1, 0.75f);
    float reverse =
        BRDFUtils::PowerHeuristic(1, 0.75f, 1, 0.25f);

    ExpectFloatNear(
        "power heuristic complementary weights",
        forward + reverse,
        1.0f);

    ExpectFloatNear(
        "power heuristic sample counts",
        BRDFUtils::PowerHeuristic(2, 0.25f, 1, 0.5f),
        0.5f);
}

void ExpectSpecularPathSkipsDirectLightSampling()
{
    BxDFType pureSpecular = static_cast<BxDFType>(
        BSDF_SPECULAR | BSDF_REFLECTION);

    if (IsNonSpecular(pureSpecular) ||
        !IsNonSpecular(DiffuseReflection) ||
        !IsNonSpecular(GlossyReflection))
    {
        throw std::runtime_error(
            "non-specular BSDF classification failed");
    }

    auto material =
        std::make_shared<TestSpecularMaterial>();

    HittableList world;
    world.add(
        std::make_shared<Sphere>(
            Vector3f(0.0f, 0.0f, -1.0f),
            0.5f,
            material));

    std::vector<std::shared_ptr<Light>> lights{
        std::make_shared<UniformInfiniteLight>(
            Spectrum(1.0f))
    };

    Camera camera(800, 600);
    auto sampler =
        std::make_shared<SpecularPathSampler>();

    PathIntegrator integrator(
        1,
        world,
        camera,
        sampler,
        std::move(lights));

    Spectrum result = integrator.Li(
        Ray(
            Vector3f(0.0f, 0.0f, 0.0f),
            Vector3f(0.0f, 0.0f, -1.0f)),
        *sampler);

    if (sampler->TwoDCount() != 1)
    {
        throw std::runtime_error(
            "pure specular path did not use exactly one BSDF sample");
    }

    ExpectRGBNear(
        "path specular skips direct light sampling",
        result,
        1.0f);
}

Spectrum TraceRoulettePath(
    const Hittable& world,
    const std::shared_ptr<RouletteSequenceSampler>& sampler)
{
    Camera camera(800, 600);
    std::vector<std::shared_ptr<Light>> lights{
        std::make_shared<UniformInfiniteLight>(
            Spectrum(1.0f))
    };
    PathIntegrator integrator(
        4,
        world,
        camera,
        sampler,
        std::move(lights));

    return integrator.Li(
        Ray(
            Vector3f(0.0f, 0.0f, 0.0f),
            Vector3f(0.0f, 0.0f, -1.0f)),
        *sampler);
}

void ExpectRussianRoulette()
{
    ExpectFloatNear(
        "spectrum maximum component",
        Spectrum(0.25f, 0.75f, 0.5f).MaxComponentValue(),
        0.75f);

    BxDFType reflectionFlags = static_cast<BxDFType>(
        BSDF_REFLECTION | BSDF_SPECULAR);
    auto halfThroughput =
        std::make_shared<RouletteMaterial>(
            0.5f,
            reflectionFlags);

    TwoHitPathWorld terminatedWorld({
        halfThroughput,
        halfThroughput
    });
    auto terminateSampler =
        std::make_shared<RouletteSequenceSampler>(0.5f);

    // After two bounces beta is 0.25, so q is 0.75. A sample of 0.5
    // terminates the path before it reaches the environment.
    ExpectRGBNear(
        "path Russian roulette terminates",
        TraceRoulettePath(
            terminatedWorld,
            terminateSampler),
        0.0f);

    if (terminateSampler->OneDCount() != 1)
        throw std::runtime_error(
            "termination branch did not draw exactly one roulette sample");

    TwoHitPathWorld survivingWorld({
        halfThroughput,
        halfThroughput
    });
    auto surviveSampler =
        std::make_shared<RouletteSequenceSampler>(0.8f);

    // A sample of 0.8 survives. Dividing beta by 1-q changes it from
    // 0.25 to 1, which is observed when the ray reaches the environment.
    ExpectRGBNear(
        "path Russian roulette survival compensation",
        TraceRoulettePath(
            survivingWorld,
            surviveSampler),
        1.0f);

    if (surviveSampler->OneDCount() != 1)
        throw std::runtime_error(
            "survival branch did not draw exactly one roulette sample");

    auto unitReflection =
        std::make_shared<RouletteMaterial>(
            1.0f,
            reflectionFlags);
    BxDFType transmissionFlags = static_cast<BxDFType>(
        BSDF_TRANSMISSION | BSDF_SPECULAR);
    auto quarterTransmission =
        std::make_shared<RouletteMaterial>(
            0.25f,
            transmissionFlags,
            2.0f);
    TwoHitPathWorld etaScaleWorld({
        unitReflection,
        quarterTransmission
    });
    auto etaScaleSampler =
        std::make_shared<RouletteSequenceSampler>(
            0.0f,
            true);

    // beta is 0.25 after transmission, while etaScale is 4. Their
    // product is 1, so Russian roulette must not be sampled.
    ExpectRGBNear(
        "path transmission etaScale avoids early roulette",
        TraceRoulettePath(
            etaScaleWorld,
            etaScaleSampler),
        0.25f);
}

void ExpectRefractionAndTotalInternalReflection()
{
    const Vector3f normal(0.0f, 0.0f, 1.0f);
    Vector3f refractNormal = normal;
    Vector3f transmittedDirection(9.0f, 8.0f, 7.0f);
    float relativeEta = -1.0f;

    bool refracted = Refract(
        Vector3f(0.0f, 0.0f, 1.0f),
        refractNormal,
        1.5f,
        &relativeEta,
        &transmittedDirection);

    bool normalIncidenceCorrect =
        refracted &&
        (transmittedDirection -
            Vector3f(0.0f, 0.0f, -1.0f)).norm() <= 1e-6f &&
        std::fabs(relativeEta - 1.5f) <= 1e-6f &&
        (refractNormal - normal).norm() <= 1e-6f;

    if (!normalIncidenceCorrect)
        throw std::runtime_error(
            "normal-incidence refraction failed");

    std::cout <<
        "[PASS] normal-incidence refraction\n";

    Vector3f tirNormal = normal;
    const Vector3f unchangedDirection(9.0f, 8.0f, 7.0f);
    Vector3f tirDirection = unchangedDirection;
    constexpr float unchangedEta = -1.0f;
    float tirEta = unchangedEta;

    // The ray is inside eta=1.5 glass and has sin(theta)=0.8.
    // This is above the glass-to-air critical angle, so no real
    // transmitted direction exists.
    bool tirRefracted = Refract(
        Vector3f(0.8f, 0.0f, -0.6f),
        tirNormal,
        1.5f,
        &tirEta,
        &tirDirection);

    bool totalInternalReflectionCorrect =
        !tirRefracted &&
        (tirNormal - normal).norm() <= 1e-6f &&
        (tirDirection - unchangedDirection).norm() <= 1e-6f &&
        tirEta == unchangedEta;

    if (!totalInternalReflectionCorrect)
        throw std::runtime_error(
            "total internal reflection handling failed");

    std::cout <<
        "[PASS] total internal reflection\n";
}

void ExpectDielectricRho()
{
    constexpr float eta = 1.5f;
    TrowbridgeReitzDistribution smoothDistribution(
        0.0f,
        0.0f);
    DielectricBxDF dielectric(
        eta,
        smoothDistribution);

    Vector3f wo(0.0f, 0.0f, 1.0f);
    Point2f reflectionSample[] = {
        Point2f(0.0f, 0.0f)
    };
    Point2f transmissionSample[] = {
        Point2f(0.5f, 0.0f)
    };

    ExpectRGBNear(
        "dielectric rho reflection sample",
        dielectric.rho(
            wo,
            1,
            reflectionSample),
        1.0f);

    ExpectRGBNear(
        "dielectric rho transmission sample",
        dielectric.rho(
            wo,
            1,
            transmissionSample),
        1.0f / (eta * eta));

    ExpectRGBNear(
        "dielectric rho zero samples",
        dielectric.rho(
            wo,
            0,
            nullptr),
        0.0f);

    MaterialEvalContext context{
        Vector3f(0.0f, 0.0f, 0.0f),
        wo,
        Vector3f(0.0f, 0.0f, 1.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
        0.0f,
        0.0f
    };
    DielectricMaterial material(
        eta,
        0.0f,
        0.0f,
        false);
    BSDF bsdf = material.GetBSDF(context);

    if (IsNonSpecular(bsdf.Flags()) ||
        !(static_cast<int>(bsdf.Flags()) & BSDF_SPECULAR))
    {
        throw std::runtime_error(
            "smooth dielectric material flags failed");
    }

    std::cout <<
        "[PASS] dielectric material instantiation\n";
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

void ExpectLightTypesAndDirectionPdfs(
    const std::shared_ptr<Material>& material)
{
    LightSampleContext context{
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 0.0f, 1.0f),
        Vector3f(0.0f, 0.0f, 1.0f)
    };

    PointLight point(
        Vector3f(0.0f, 0.0f, 1.0f),
        Spectrum(1.0f));
    DistantLight distant(
        Vector3f(0.0f, 0.0f, 1.0f),
        Spectrum(1.0f));
    UniformInfiniteLight environment(Spectrum(1.0f));

    auto triangle = std::make_shared<Triangle>(
        Vector3f(-1.0f, -1.0f, -2.0f),
        Vector3f(1.0f, -1.0f, -2.0f),
        Vector3f(0.0f, 1.0f, -2.0f),
        Vector3f(0.0f, 0.0f, 1.0f),
        Vector3f(0.0f, 0.0f, 1.0f),
        Vector3f(0.0f, 0.0f, 1.0f),
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
        Vector3f(0.5f, 1.0f, 0.0f),
        material);
    DiffuseAreaLight area(
        triangle,
        Spectrum(1.0f));

    if (point.Type() != LightType::DeltaPosition ||
        distant.Type() != LightType::DeltaDirection ||
        environment.Type() != LightType::Infinite ||
        area.Type() != LightType::Area ||
        !IsDeltaLight(point.Type()) ||
        !IsDeltaLight(distant.Type()) ||
        IsDeltaLight(environment.Type()) ||
        IsDeltaLight(area.Type()))
    {
        throw std::runtime_error(
            "light type classification failed");
    }

    std::cout << "[PASS] light type classification\n";

    Vector3f direction(0.0f, 0.0f, -1.0f);
    constexpr float epsilon = 1e-4f;

    if (point.PDF_Li(context, direction) != 0.0f ||
        distant.PDF_Li(context, direction) != 0.0f ||
        std::fabs(
            environment.PDF_Li(context, direction) - Inv4Pi) > epsilon ||
        environment.PDF_Li(
            context, Vector3f(0.0f, 0.0f, 0.0f)) != 0.0f)
    {
        throw std::runtime_error(
            "delta or infinite light PDF failed");
    }

    std::cout << "[PASS] delta and infinite light PDFs\n";

    // The triangle has area 2, is four squared-distance units away,
    // and faces the receiver: p_omega = 4 / (1 * 2) = 2 sr^-1.
    float expectedAreaPdf = 2.0f;
    float shapePdf = triangle->PDF(context.p, direction);
    float lightPdf = area.PDF_Li(context, direction);

    if (std::fabs(shapePdf - expectedAreaPdf) > epsilon ||
        std::fabs(lightPdf - expectedAreaPdf) > epsilon ||
        triangle->PDF(
            context.p, Vector3f(0.0f, 0.0f, 1.0f)) != 0.0f)
    {
        throw std::runtime_error(
            "triangle area light directional PDF failed");
    }

    std::cout <<
        "[PASS] triangle area light directional PDF\n";
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

void ExpectTriangleSurfaceSample(
    const std::shared_ptr<Material>& material)
{
    const Vector3f p0(0.0f, 0.0f, 0.0f);
    const Vector3f p1(2.0f, 0.0f, 0.0f);
    const Vector3f p2(0.0f, 2.0f, 0.0f);
    const Vector3f normal(0.0f, 0.0f, 1.0f);

    auto triangle = std::make_shared<Triangle>(
        p0,
        p1,
        p2,
        normal,
        normal,
        normal,
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 1.0f, 0.0f),
        material);

    constexpr float epsilon = 1e-4f;

    if (std::fabs(triangle->SurfaceArea() - 2.0f) > epsilon)
        throw std::runtime_error(
            "triangle surface area failed");

    std::cout <<
        "[PASS] triangle surface area\n";

    auto centroid = triangle->Sample(
        Point2f(2.0f / 3.0f, 2.0f / 3.0f));

    if (!centroid)
        throw std::runtime_error(
            "triangle returned no centroid sample");

    bool positionCorrect =
        std::fabs(centroid->p.x - 2.0f / 3.0f) <= epsilon &&
        std::fabs(centroid->p.y - 2.0f / 3.0f) <= epsilon &&
        std::fabs(centroid->p.z) <= epsilon;

    if (!positionCorrect)
        throw std::runtime_error(
            "triangle centroid sample position failed");

    std::cout <<
        "[PASS] triangle surface sample position\n";

    bool normalCorrect =
        std::fabs(centroid->n.x) <= epsilon &&
        std::fabs(centroid->n.y) <= epsilon &&
        std::fabs(centroid->n.z - 1.0f) <= epsilon;
    bool pdfCorrect =
        std::fabs(centroid->pdfArea - 0.5f) <= epsilon;

    if (!normalCorrect || !pdfCorrect)
        throw std::runtime_error(
            "triangle surface sample normal or PDF failed");

    std::cout <<
        "[PASS] triangle surface sample normal and PDF\n";

    Triangle degenerate(
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
        Vector3f(2.0f, 0.0f, 0.0f),
        normal,
        normal,
        normal,
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
        Vector3f(2.0f, 0.0f, 0.0f),
        material);

    if (degenerate.SurfaceArea() != 0.0f ||
        degenerate.Sample(Point2f(0.5f, 0.5f)))
    {
        throw std::runtime_error(
            "degenerate triangle returned a surface sample");
    }

    std::cout <<
        "[PASS] degenerate triangle sampling\n";

    auto areaLight =
        std::make_shared<DiffuseAreaLight>(
            triangle,
            Spectrum(0.5f));
    triangle->SetAreaLight(areaLight);

    HitRecord record;
    Ray ray(
        Vector3f(0.5f, 0.5f, 1.0f),
        Vector3f(0.0f, 0.0f, -1.0f));

    if (!triangle->hit(
            ray,
            RayEpsilon,
            std::numeric_limits<float>::infinity(),
            record) ||
        !record.areaLight ||
        record.areaLight.get() != areaLight.get())
    {
        throw std::runtime_error(
            "triangle hit record did not preserve its area light");
    }

    std::cout <<
        "[PASS] triangle area light association\n";
}

void ExpectTriangleDpdu(
    const std::shared_ptr<Material>& material)
{
    const Vector3f p0(0.0f, 0.0f, 0.0f);
    const Vector3f p1(2.0f, 0.0f, 0.0f);
    const Vector3f p2(0.0f, 2.0f, 0.0f);
    const Vector3f normal(0.0f, 0.0f, 1.0f);
    const Ray ray(
        Vector3f(0.5f, 0.5f, 1.0f),
        Vector3f(0.0f, 0.0f, -1.0f));

    auto expectDpdu = [&ray](
        const std::string& testName,
        const Triangle& triangle,
        const Vector3f& expected)
    {
        HitRecord record;
        if (!triangle.hit(
                ray,
                RayEpsilon,
                std::numeric_limits<float>::infinity(),
                record))
        {
            throw std::runtime_error(
                testName + " ray missed triangle");
        }

        constexpr float epsilon = 1e-4f;
        bool correct =
            std::fabs(record.dpdu.x - expected.x) <= epsilon &&
            std::fabs(record.dpdu.y - expected.y) <= epsilon &&
            std::fabs(record.dpdu.z - expected.z) <= epsilon;

        if (!correct)
            throw std::runtime_error(
                testName + " produced an incorrect dpdu");

        std::cout << "[PASS] " << testName << '\n';
    };

    Triangle standardUV(
        p0, p1, p2,
        normal, normal, normal,
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 1.0f, 0.0f),
        material);

    expectDpdu(
        "triangle dpdu standard UV",
        standardUV,
        Vector3f(2.0f, 0.0f, 0.0f));

    Triangle scaledUV(
        p0, p1, p2,
        normal, normal, normal,
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(2.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 1.0f, 0.0f),
        material);

    expectDpdu(
        "triangle dpdu scaled UV",
        scaledUV,
        Vector3f(1.0f, 0.0f, 0.0f));

    Triangle degenerateUV(
        p0, p1, p2,
        normal, normal, normal,
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
        Vector3f(2.0f, 0.0f, 0.0f),
        material);

    expectDpdu(
        "triangle dpdu degenerate UV fallback",
        degenerateUV,
        Vector3f(1.0f, 0.0f, 0.0f));
}

void ExpectGeometricAndShadingNormals(
    const std::shared_ptr<Material>& material)
{
    constexpr float epsilon = 1e-4f;

    Ray sphereRay(
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 0.0f, -1.0f));
    Sphere sphere(
        Vector3f(0.0f, 0.0f, -2.0f),
        1.0f,
        material);
    HitRecord sphereRecord;

    if (!sphere.hit(
            sphereRay,
            RayEpsilon,
            std::numeric_limits<float>::infinity(),
            sphereRecord) ||
        (sphereRecord.normal -
            sphereRecord.geometricNormal).norm() > epsilon)
    {
        throw std::runtime_error(
            "sphere geometric and shading normals differ");
    }

    std::cout <<
        "[PASS] sphere geometric and shading normals\n";

    const Vector3f p0(0.0f, 0.0f, 0.0f);
    const Vector3f p1(2.0f, 0.0f, 0.0f);
    const Vector3f p2(0.0f, 2.0f, 0.0f);
    const Vector3f tiltedNormal =
        Vector3f(0.0f, 1.0f, 1.0f).normalize();
    const Vector3f uv0(0.0f, 0.0f, 0.0f);
    const Vector3f uv1(1.0f, 0.0f, 0.0f);
    const Vector3f uv2(0.0f, 1.0f, 0.0f);

    Triangle smoothTriangle(
        p0, p1, p2,
        tiltedNormal,
        tiltedNormal,
        tiltedNormal,
        uv0, uv1, uv2,
        material);
    Ray triangleRay(
        Vector3f(0.5f, 0.5f, 1.0f),
        Vector3f(0.0f, 0.0f, -1.0f));
    HitRecord smoothRecord;

    if (!smoothTriangle.hit(
            triangleRay,
            RayEpsilon,
            std::numeric_limits<float>::infinity(),
            smoothRecord))
    {
        throw std::runtime_error(
            "ray missed smooth triangle");
    }

    bool geometricNormalCorrect =
        std::fabs(smoothRecord.geometricNormal.x) <= epsilon &&
        std::fabs(smoothRecord.geometricNormal.y) <= epsilon &&
        std::fabs(smoothRecord.geometricNormal.z - 1.0f) <= epsilon;
    bool shadingNormalCorrect =
        (smoothRecord.normal - tiltedNormal).norm() <= epsilon;
    bool normalsAreDistinct =
        (smoothRecord.normal -
            smoothRecord.geometricNormal).norm() > epsilon;

    if (!geometricNormalCorrect ||
        !shadingNormalCorrect ||
        !normalsAreDistinct)
    {
        throw std::runtime_error(
            "triangle geometric and shading normals were not separated");
    }

    std::cout <<
        "[PASS] triangle geometric and shading normals\n";

    const Vector3f reversedNormal(0.0f, 0.0f, -1.0f);
    Triangle reversedTriangle(
        p0, p1, p2,
        reversedNormal,
        reversedNormal,
        reversedNormal,
        uv0, uv1, uv2,
        material);
    HitRecord reversedRecord;

    if (!reversedTriangle.hit(
            triangleRay,
            RayEpsilon,
            std::numeric_limits<float>::infinity(),
            reversedRecord) ||
        reversedRecord.normal.dot(
            reversedRecord.geometricNormal) <= 0.0f ||
        std::fabs(reversedRecord.normal.z - 1.0f) > epsilon)
    {
        throw std::runtime_error(
            "reversed shading normal was not face-forwarded");
    }

    std::cout <<
        "[PASS] triangle shading normal hemisphere correction\n";

    auto sample = smoothTriangle.Sample(
        Point2f(2.0f / 3.0f, 2.0f / 3.0f));

    if (!sample ||
        (sample->n - smoothRecord.geometricNormal).norm() > epsilon)
    {
        throw std::runtime_error(
            "triangle hit and sample geometric normals differ");
    }

    std::cout <<
        "[PASS] triangle hit and sample geometric normals\n";
}

void ExpectGeometricRayOffsets()
{
    HittableList emptyWorld;
    OffsetProbeIntegrator integrator(emptyWorld);

    const Vector3f p(1.0f, 2.0f, 3.0f);
    const Vector3f ng(0.0f, 0.0f, 1.0f);
    constexpr float epsilon = 1e-7f;

    Vector3f front = integrator.Offset(
        p,
        ng,
        Vector3f(0.0f, 0.0f, 1.0f));

    if (std::fabs(front.x - p.x) > epsilon ||
        std::fabs(front.y - p.y) > epsilon ||
        std::fabs(front.z - (p.z + RayEpsilon)) > epsilon)
    {
        throw std::runtime_error(
            "front-side ray origin offset failed");
    }

    std::cout <<
        "[PASS] geometric ray offset front side\n";

    Vector3f back = integrator.Offset(
        p,
        ng,
        Vector3f(0.0f, 0.0f, -1.0f));

    if (std::fabs(back.x - p.x) > epsilon ||
        std::fabs(back.y - p.y) > epsilon ||
        std::fabs(back.z - (p.z - RayEpsilon)) > epsilon)
    {
        throw std::runtime_error(
            "back-side ray origin offset failed");
    }

    std::cout <<
        "[PASS] geometric ray offset back side\n";

    Vector3f grazingDirection =
        Vector3f(1.0f, 0.0f, 1e-6f).normalize();
    Vector3f grazing = integrator.Offset(
        p,
        ng,
        grazingDirection);

    // A normal-based offset keeps the full epsilon separation even when
    // the ray direction is almost tangent to the geometric surface.
    if (std::fabs(grazing.x - p.x) > epsilon ||
        std::fabs(grazing.y - p.y) > epsilon ||
        std::fabs(grazing.z - (p.z + RayEpsilon)) > epsilon)
    {
        throw std::runtime_error(
            "grazing ray origin offset lost normal separation");
    }

    std::cout <<
        "[PASS] geometric ray offset grazing direction\n";
}

void ExpectDiffuseAreaLightSample(
    const std::shared_ptr<Material>& material)
{
    auto shape = std::make_shared<Sphere>(
        Vector3f(0.0f, 0.0f, -2.0f),
        1.0f,
        material);

    LightSampleContext context{
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 0.0f, 1.0f),
        Vector3f(0.0f, 0.0f, 1.0f)
    };

    DiffuseAreaLight oneSided(
        shape,
        Spectrum(0.75f));

    // u = (0, 0) samples the sphere's north pole at (0, 0, -1).
    auto front = oneSided.SampleLi(
        context,
        Point2f(0.0f, 0.0f));

    if (!front)
        throw std::runtime_error(
            "one-sided area light returned no front sample");

    ExpectRGBNear(
        "area light sample radiance",
        front->L,
        0.75f);

    constexpr float epsilon = 1e-4f;
    bool frontDirectionCorrect =
        std::fabs(front->wi.x) <= epsilon &&
        std::fabs(front->wi.y) <= epsilon &&
        std::fabs(front->wi.z + 1.0f) <= epsilon;
    bool frontDistanceCorrect =
        std::fabs(front->distance - 1.0f) <= epsilon;
    bool frontPdfCorrect =
        std::fabs(front->pdf - Inv4Pi) <= epsilon;

    if (!frontDirectionCorrect ||
        !frontDistanceCorrect ||
        !frontPdfCorrect)
    {
        throw std::runtime_error(
            "area light front sample geometry failed");
    }

    std::cout <<
        "[PASS] area light front sample\n";

    // u.x = 1 samples the south pole. Its normal points away from ctx.
    auto rejectedBack = oneSided.SampleLi(
        context,
        Point2f(1.0f, 0.0f));

    if (rejectedBack)
        throw std::runtime_error(
            "one-sided area light accepted a back sample");

    DiffuseAreaLight twoSided(
        shape,
        Spectrum(0.75f),
        true);

    auto acceptedBack = twoSided.SampleLi(
        context,
        Point2f(1.0f, 0.0f));

    if (!acceptedBack)
        throw std::runtime_error(
            "two-sided area light rejected a back sample");

    // South pole is three units from ctx; p_omega = p_A * 3^2 / 1.
    float expectedBackPdf =
        9.0f * Inv4Pi;

    bool backDirectionCorrect =
        std::fabs(acceptedBack->wi.x) <= epsilon &&
        std::fabs(acceptedBack->wi.y) <= epsilon &&
        std::fabs(acceptedBack->wi.z + 1.0f) <= epsilon;
    bool backDistanceCorrect =
        std::fabs(acceptedBack->distance - 3.0f) <= epsilon;
    bool backPdfCorrect =
        std::fabs(
            acceptedBack->pdf - expectedBackPdf) <= epsilon;

    if (!backDirectionCorrect ||
        !backDistanceCorrect ||
        !backPdfCorrect)
    {
        throw std::runtime_error(
            "two-sided area light back sample failed");
    }

    DiffuseAreaLight missingShape(
        nullptr,
        Spectrum(1.0f));

    if (missingShape.SampleLi(
            context,
            Point2f(0.0f, 0.0f)))
    {
        throw std::runtime_error(
            "area light without a shape returned a sample");
    }

    std::cout <<
        "[PASS] area light sidedness and PDF\n";
}

void ExpectHitRecordAreaLight(
    const std::shared_ptr<Material>& material)
{
    auto emissiveShape = std::make_shared<Sphere>(
        Vector3f(0.0f, 0.0f, -2.0f),
        1.0f,
        material);

    auto areaLight =
        std::make_shared<DiffuseAreaLight>(
            emissiveShape,
            Spectrum(0.75f));

    emissiveShape->SetAreaLight(areaLight);

    Ray ray(
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 0.0f, -1.0f));

    HitRecord emissiveRecord;
    if (!emissiveShape->hit(
            ray,
            RayEpsilon,
            std::numeric_limits<float>::infinity(),
            emissiveRecord))
    {
        throw std::runtime_error(
            "ray missed emissive sphere");
    }

    if (!emissiveRecord.areaLight ||
        emissiveRecord.areaLight.get() != areaLight.get())
    {
        throw std::runtime_error(
            "hit record did not preserve its area light");
    }

    auto ordinaryShape = std::make_shared<Sphere>(
        Vector3f(0.0f, 0.0f, -2.0f),
        1.0f,
        material);

    HitRecord ordinaryRecord;
    if (!ordinaryShape->hit(
            ray,
            RayEpsilon,
            std::numeric_limits<float>::infinity(),
            ordinaryRecord))
    {
        throw std::runtime_error(
            "ray missed ordinary sphere");
    }

    if (ordinaryRecord.areaLight)
        throw std::runtime_error(
            "ordinary hit record unexpectedly has an area light");

    std::cout <<
        "[PASS] hit record area light association\n";
}

void ExpectAreaLightSurfaceEmission()
{
    auto emissiveShape = std::make_shared<Sphere>(
        Vector3f(0.0f, 0.0f, -2.0f),
        1.0f,
        nullptr);

    auto areaLight =
        std::make_shared<DiffuseAreaLight>(
            emissiveShape,
            Spectrum(0.75f));

    emissiveShape->SetAreaLight(areaLight);
    HittableList world(emissiveShape);

    Ray frontRay(
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 0.0f, -1.0f));

    ExpectRGBNear(
        "area light surface emission front",
        TraceSurfaceEmission(
            world,
            1,
            frontRay),
        0.75f);

    // Start inside the sphere and hit its outward-facing surface from behind.
    Ray backRay(
        Vector3f(0.0f, 0.0f, -2.0f),
        Vector3f(0.0f, 0.0f, 1.0f));

    ExpectRGBNear(
        "one-sided area light surface emission back",
        TraceSurfaceEmission(
            world,
            1,
            backRay),
        0.0f);

    ExpectRGBNear(
        "area light surface emission zero depth",
        TraceSurfaceEmission(
            world,
            0,
            frontRay),
        0.75f);
}

int main()
{
    RunElfouhailySpectrumAcceptanceTests();
    RunOceanFrequencyFieldAcceptanceTests();
    RunOceanFFTAcceptanceTests();
    RunOceanHeightFieldAcceptanceTests();
    RunMeshTriangleAcceptanceTests();
    RunOceanSurfaceMeshAcceptanceTests();

    Ray centerRay(
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 0.0f, -1.0f));

    std::vector<std::shared_ptr<Light>> whiteEnvironment{
        std::make_shared<UniformInfiniteLight>(
            Spectrum(1.0f))
    };

    ExpectPowerHeuristic();
    ExpectRefractionAndTotalInternalReflection();
    ExpectDielectricRho();
    ExpectSpecularPathSkipsDirectLightSampling();
    ExpectRussianRoulette();
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
        "path baseline environment",
        TracePath(
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

    ExpectLightTypesAndDirectionPdfs(material);
    ExpectSphereSurfaceSample(material);
    ExpectTriangleSurfaceSample(material);
    ExpectTriangleDpdu(material);
    ExpectGeometricAndShadingNormals(material);
    ExpectGeometricRayOffsets();
    ExpectDiffuseAreaLightSample(material);
    ExpectHitRecordAreaLight(material);
    ExpectAreaLightSurfaceEmission();

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

    HittableList areaLightWorld;

    areaLightWorld.add(
        std::make_shared<Sphere>(
            Vector3f(0.0f, 0.0f, -1.0f),
            0.5f,
            material));

    auto areaLightShape =
        std::make_shared<Sphere>(
            Vector3f(0.0f, 0.0f, 1.5f),
            1.0f,
            material);

    // Keep the emitter geometry in the world. The shadow ray must stop
    // just before the sampled point instead of treating the light as blocked.
    areaLightWorld.add(areaLightShape);

    auto areaLight =
        std::make_shared<DiffuseAreaLight>(
            areaLightShape,
            Spectrum(0.25f));

    areaLightShape->SetAreaLight(areaLight);

    std::vector<std::shared_ptr<Light>> areaLights{
        areaLight
    };

    ExpectRGBNear(
        "area light direct",
        TraceAreaLightDirect(
            areaLightWorld,
            centerRay,
            areaLights),
        0.5f);

    // The direct-light sample contributes 0.5. The following diffuse BSDF
    // ray hits the same emitter and must not add its radiance a second time.
    ExpectRGBNear(
        "area light no double count",
        TraceAreaLightWithBSDFSampling(
            areaLightWorld,
            centerRay,
            areaLights),
        0.5f);

    ExpectRGBNear(
        "path MIS area light",
        TracePathAreaLight(
            areaLightWorld,
            centerRay,
            areaLights),
        2.5f / 17.0f);

    HittableList triangleLightWorld;

    triangleLightWorld.add(
        std::make_shared<Sphere>(
            Vector3f(0.0f, 0.0f, -1.0f),
            0.5f,
            material));

    const Vector3f downwardNormal(0.0f, 0.0f, -1.0f);
    auto triangleLightShape =
        std::make_shared<Triangle>(
            Vector3f(-1.0f, -1.0f, 0.5f),
            Vector3f(0.0f, 2.0f, 0.5f),
            Vector3f(1.0f, -1.0f, 0.5f),
            downwardNormal,
            downwardNormal,
            downwardNormal,
            Vector3f(0.0f, 0.0f, 0.0f),
            Vector3f(0.5f, 1.0f, 0.0f),
            Vector3f(1.0f, 0.0f, 0.0f),
            nullptr);

    auto triangleLight =
        std::make_shared<DiffuseAreaLight>(
            triangleLightShape,
            Spectrum(Pi / 3.0f));

    triangleLightShape->SetAreaLight(triangleLight);
    triangleLightWorld.add(triangleLightShape);

    std::vector<std::shared_ptr<Light>> triangleLights{
        triangleLight
    };

    ExpectRGBNear(
        "triangle area light direct",
        TraceTriangleAreaLightDirect(
            triangleLightWorld,
            centerRay,
            triangleLights),
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

    // For both deterministic north-pole samples, p_light = 1/(4*pi)
    // and p_bsdf = 1/pi. The light and BSDF MIS weights are 1/17
    // and 16/17, respectively.
    ExpectRGBNear(
        "path MIS environment",
        TracePathEnvironment(
            diffuseWorld,
            centerRay,
            quarterEnvironment),
        2.5f / 17.0f);

    ExpectRGBNear(
        "path delta point light",
        TracePath(
            diffuseWorld,
            1,
            centerRay,
            nearLights),
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
