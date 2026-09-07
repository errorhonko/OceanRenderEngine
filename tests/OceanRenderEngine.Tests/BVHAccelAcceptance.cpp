#include "BVHAccel.h"

#include "DiffuseMaterial.h"
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
        throw std::runtime_error(testName + " failed");

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
    float first,
    float second,
    float epsilon = 1e-5f)
{
    return std::fabs(first - second) <= epsilon;
}

bool VectorNear(
    const Vector3f& first,
    const Vector3f& second,
    float epsilon = 1e-5f)
{
    return Near(first.x, second.x, epsilon) &&
           Near(first.y, second.y, epsilon) &&
           Near(first.z, second.z, epsilon);
}

class EmptyBoundsPrimitive final : public Hittable
{
public:
    bool hit(
        const Ray&,
        float,
        float,
        HitRecord&) const override
    {
        return false;
    }
};

class CountingPrimitive final : public Hittable
{
public:
    CountingPrimitive(
        std::shared_ptr<Hittable> primitive,
        int* hitCount)
        : primitive(std::move(primitive)),
          hitCount(hitCount)
    {
    }

    Bounds3f Bounds() const override
    {
        return primitive->Bounds();
    }

    bool hit(
        const Ray& ray,
        float tMin,
        float tMax,
        HitRecord& rec) const override
    {
        ++(*hitCount);
        return primitive->hit(
            ray,
            tMin,
            tMax,
            rec);
    }

private:
    std::shared_ptr<Hittable> primitive;
    int* hitCount = nullptr;
};
}

void RunBVHAccelAcceptanceTests()
{
    ExpectThrows(
        "BVH rejects empty primitive collection",
        []
        {
            BVHAccel bvh(
                std::vector<
                    std::shared_ptr<Hittable>>{});
        });

    const auto testSphere =
        std::make_shared<Sphere>(
            Vector3f(0.0f, 0.0f, -3.0f),
            1.0f,
            nullptr);

    ExpectThrows(
        "BVH rejects zero leaf capacity",
        [&testSphere]
        {
            BVHAccel bvh(
                std::vector<
                    std::shared_ptr<Hittable>>{
                        testSphere},
                0);
        });

    ExpectThrows(
        "BVH rejects null primitive",
        []
        {
            BVHAccel bvh(
                std::vector<
                    std::shared_ptr<Hittable>>{
                        nullptr});
        });

    ExpectThrows(
        "BVH rejects empty primitive bounds",
        []
        {
            BVHAccel bvh(
                std::vector<
                    std::shared_ptr<Hittable>>{
                        std::make_shared<
                            EmptyBoundsPrimitive>()});
        });

    const auto boundsSphere0 =
        std::make_shared<Sphere>(
            Vector3f(-3.0f, 0.0f, 0.0f),
            1.0f,
            nullptr);
    const auto boundsSphere1 =
        std::make_shared<Sphere>(
            Vector3f(4.0f, 2.0f, 0.0f),
            2.0f,
            nullptr);

    BVHAccel boundsBVH(
        std::vector<std::shared_ptr<Hittable>>{
            boundsSphere0,
            boundsSphere1});
    const Bounds3f aggregateBounds =
        boundsBVH.Bounds();

    ExpectTrue(
        "BVH aggregate bounds",
        VectorNear(
            aggregateBounds.pMin,
            Vector3f(-4.0f, -1.0f, -2.0f)) &&
        VectorNear(
            aggregateBounds.pMax,
            Vector3f(6.0f, 4.0f, 2.0f)));

    const auto nearMaterial =
        std::make_shared<DiffuseMaterial>(
            Spectrum(0.25f));
    const auto farMaterial =
        std::make_shared<DiffuseMaterial>(
            Spectrum(0.75f));
    const auto farSphere =
        std::make_shared<Sphere>(
            Vector3f(0.0f, 0.0f, -8.0f),
            1.0f,
            farMaterial);
    const auto nearSphere =
        std::make_shared<Sphere>(
            Vector3f(0.0f, 0.0f, -3.0f),
            1.0f,
            nearMaterial);

    BVHAccel splitBVH(
        std::vector<std::shared_ptr<Hittable>>{
            farSphere,
            nearSphere},
        1);

    const Ray centerRay(
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 0.0f, -1.0f));
    const float infinity =
        std::numeric_limits<float>::infinity();
    HitRecord closestRecord;

    ExpectTrue(
        "BVH closest hit across split",
        splitBVH.hit(
            centerRay,
            1e-4f,
            infinity,
            closestRecord) &&
        Near(closestRecord.t, 2.0f) &&
        closestRecord.material == nearMaterial);

    HitRecord rangeRecord;
    ExpectTrue(
        "BVH t interval rejection",
        !splitBVH.hit(
            centerRay,
            1e-4f,
            1.5f,
            rangeRecord));

    HitRecord missRecord;
    ExpectTrue(
        "BVH ray miss",
        !splitBVH.hit(
            Ray(
                Vector3f(5.0f, 0.0f, 0.0f),
                Vector3f(0.0f, 0.0f, -1.0f)),
            1e-4f,
            infinity,
            missRecord));

    int visibleHitCount = 0;
    int culledHitCount = 0;

    const auto visiblePrimitive =
        std::make_shared<CountingPrimitive>(
            std::make_shared<Sphere>(
                Vector3f(0.0f, 0.0f, -5.0f),
                1.0f,
                nullptr),
            &visibleHitCount);
    const auto culledPrimitive =
        std::make_shared<CountingPrimitive>(
            std::make_shared<Sphere>(
                Vector3f(100.0f, 0.0f, -5.0f),
                1.0f,
                nullptr),
            &culledHitCount);

    BVHAccel cullingBVH(
        std::vector<std::shared_ptr<Hittable>>{
            culledPrimitive,
            visiblePrimitive},
        1);
    HitRecord cullingRecord;

    ExpectTrue(
        "BVH prunes missed child bounds",
        cullingBVH.hit(
            centerRay,
            1e-4f,
            infinity,
            cullingRecord) &&
        visibleHitCount == 1 &&
        culledHitCount == 0);
}
