#include "SimplePathIntegrator.h"
#include "IndependentSampler.h"
#include "DiffuseMaterial.h"
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
