#include "Bounds3f.h"
#include "HittableList.h"
#include "Sphere.h"
#include "Triangle.h"

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
        throw std::runtime_error(testName + " failed");

    std::cout << "[PASS] " << testName << '\n';
}

bool Near(float a, float b, float epsilon = 1e-6f)
{
    return std::fabs(a - b) <= epsilon;
}

bool VectorNear(
    const Vector3f& a,
    const Vector3f& b,
    float epsilon = 1e-6f)
{
    return Near(a.x, b.x, epsilon) &&
           Near(a.y, b.y, epsilon) &&
           Near(a.z, b.z, epsilon);
}
}

void RunBounds3fAcceptanceTests()
{
    const Bounds3f bounds(
        Vector3f(2.0f, -1.0f, 3.0f),
        Vector3f(-2.0f, 4.0f, -3.0f));

    ExpectTrue(
        "bounds ordered construction",
        VectorNear(
            bounds.pMin,
            Vector3f(-2.0f, -1.0f, -3.0f)) &&
        VectorNear(
            bounds.pMax,
            Vector3f(2.0f, 4.0f, 3.0f)));

    ExpectTrue(
        "bounds diagonal and centroid",
        VectorNear(
            bounds.Diagonal(),
            Vector3f(4.0f, 5.0f, 6.0f)) &&
        VectorNear(
            bounds.Centroid(),
            Vector3f(0.0f, 1.5f, 0.0f)));

    ExpectTrue(
        "bounds surface area",
        Near(bounds.SurfaceArea(), 148.0f));

    ExpectTrue(
        "bounds maximum extent",
        bounds.MaximumExtent() == 2);

    const Bounds3f pointUnion =
        Union(bounds, Vector3f(5.0f, 2.0f, -4.0f));
    ExpectTrue(
        "bounds union point",
        VectorNear(
            pointUnion.pMin,
            Vector3f(-2.0f, -1.0f, -4.0f)) &&
        VectorNear(
            pointUnion.pMax,
            Vector3f(5.0f, 4.0f, 3.0f)));

    const Bounds3f boundsUnion =
        Union(
            bounds,
            Bounds3f(
                Vector3f(-5.0f, 0.0f, -1.0f),
                Vector3f(-4.0f, 8.0f, 1.0f)));
    ExpectTrue(
        "bounds union bounds",
        VectorNear(
            boundsUnion.pMin,
            Vector3f(-5.0f, -1.0f, -3.0f)) &&
        VectorNear(
            boundsUnion.pMax,
            Vector3f(2.0f, 8.0f, 3.0f)));

    const Bounds3f accumulatedPoint =
        Union(
            Bounds3f(),
            Vector3f(1.0f, 2.0f, 3.0f));
    ExpectTrue(
        "bounds union from empty",
        !accumulatedPoint.IsEmpty() &&
        VectorNear(
            accumulatedPoint.pMin,
            Vector3f(1.0f, 2.0f, 3.0f)) &&
        VectorNear(
            accumulatedPoint.pMax,
            Vector3f(1.0f, 2.0f, 3.0f)));

    const Bounds3f unitBounds(
        Vector3f(-1.0f, -1.0f, -1.0f),
        Vector3f(1.0f, 1.0f, 1.0f));
    const float infinity =
        std::numeric_limits<float>::infinity();

    ExpectTrue(
        "bounds outside ray hit",
        unitBounds.IntersectP(
            Ray(
                Vector3f(-2.0f, 0.0f, 0.0f),
                Vector3f(1.0f, 0.0f, 0.0f)),
            0.0f,
            infinity));

    ExpectTrue(
        "bounds inside ray hit",
        unitBounds.IntersectP(
            Ray(
                Vector3f(0.0f, 0.0f, 0.0f),
                Vector3f(1.0f, 0.0f, 0.0f)),
            0.0f,
            infinity));

    ExpectTrue(
        "bounds negative direction ray hit",
        unitBounds.IntersectP(
            Ray(
                Vector3f(2.0f, 0.0f, 0.0f),
                Vector3f(-1.0f, 0.0f, 0.0f)),
            0.0f,
            infinity));

    ExpectTrue(
        "bounds parallel ray miss",
        !unitBounds.IntersectP(
            Ray(
                Vector3f(2.0f, 0.0f, 0.0f),
                Vector3f(0.0f, 1.0f, 0.0f)),
            0.0f,
            infinity));

    ExpectTrue(
        "bounds grazing ray hit",
        unitBounds.IntersectP(
            Ray(
                Vector3f(-2.0f, 1.0f, 1.0f),
                Vector3f(1.0f, 0.0f, 0.0f)),
            0.0f,
            infinity));

    ExpectTrue(
        "bounds t interval rejection",
        !unitBounds.IntersectP(
            Ray(
                Vector3f(-2.0f, 0.0f, 0.0f),
                Vector3f(1.0f, 0.0f, 0.0f)),
            0.0f,
            0.5f));

    const Bounds3f emptyBounds;
    ExpectTrue(
        "bounds empty rejection",
        emptyBounds.IsEmpty() &&
        emptyBounds.SurfaceArea() == 0.0f &&
        !emptyBounds.IntersectP(
            Ray(
                Vector3f(0.0f, 0.0f, 0.0f),
                Vector3f(1.0f, 0.0f, 0.0f)),
            0.0f,
            infinity));

    const Triangle triangle(
        Vector3f(3.0f, -2.0f, 5.0f),
        Vector3f(-4.0f, 6.0f, 1.0f),
        Vector3f(2.0f, 0.5f, -7.0f),
        Vector3f(0.0f, 1.0f, 0.0f),
        Vector3f(0.0f, 1.0f, 0.0f),
        Vector3f(0.0f, 1.0f, 0.0f),
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 0.0f, 0.0f),
        nullptr);
    const Bounds3f triangleBounds = triangle.Bounds();

    ExpectTrue(
        "triangle bounds contain all vertices",
        VectorNear(
            triangleBounds.pMin,
            Vector3f(-4.0f, -2.0f, -7.0f)) &&
        VectorNear(
            triangleBounds.pMax,
            Vector3f(3.0f, 6.0f, 5.0f)));

    const Sphere sphere(
        Vector3f(1.0f, -2.0f, 3.0f),
        2.5f,
        nullptr);
    const Bounds3f sphereBounds = sphere.Bounds();

    ExpectTrue(
        "sphere bounds",
        VectorNear(
            sphereBounds.pMin,
            Vector3f(-1.5f, -4.5f, 0.5f)) &&
        VectorNear(
            sphereBounds.pMax,
            Vector3f(3.5f, 0.5f, 5.5f)));

    const HittableList emptyList;
    ExpectTrue(
        "empty hittable list bounds",
        emptyList.Bounds().IsEmpty());

    const auto firstSphere =
        std::make_shared<Sphere>(
            Vector3f(-2.0f, 1.0f, 4.0f),
            1.0f,
            nullptr);
    HittableList singleObjectList(firstSphere);
    const Bounds3f singleObjectBounds =
        singleObjectList.Bounds();

    ExpectTrue(
        "single object hittable list bounds",
        VectorNear(
            singleObjectBounds.pMin,
            Vector3f(-3.0f, 0.0f, 3.0f)) &&
        VectorNear(
            singleObjectBounds.pMax,
            Vector3f(-1.0f, 2.0f, 5.0f)));

    const auto secondSphere =
        std::make_shared<Sphere>(
            Vector3f(3.0f, -4.0f, 0.0f),
            2.0f,
            nullptr);
    singleObjectList.add(secondSphere);
    const Bounds3f multipleObjectBounds =
        singleObjectList.Bounds();

    ExpectTrue(
        "multiple object hittable list bounds",
        VectorNear(
            multipleObjectBounds.pMin,
            Vector3f(-3.0f, -6.0f, -2.0f)) &&
        VectorNear(
            multipleObjectBounds.pMax,
            Vector3f(5.0f, 2.0f, 5.0f)));
}
