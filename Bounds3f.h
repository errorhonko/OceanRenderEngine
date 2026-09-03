#pragma once

#include "Ray.h"

#include <algorithm>
#include <limits>

class Bounds3f
{
public:
    Bounds3f()
    {
        const float infinity =
            std::numeric_limits<float>::infinity();

        pMin = Vector3f(
            infinity,
            infinity,
            infinity);
        pMax = Vector3f(
            -infinity,
            -infinity,
            -infinity);
    }

    explicit Bounds3f(const Vector3f& p)
        : pMin(p),
          pMax(p)
    {
    }

    Bounds3f(
        const Vector3f& p1,
        const Vector3f& p2)
        : pMin(
              std::min(p1.x, p2.x),
              std::min(p1.y, p2.y),
              std::min(p1.z, p2.z)),
          pMax(
              std::max(p1.x, p2.x),
              std::max(p1.y, p2.y),
              std::max(p1.z, p2.z))
    {
    }

    bool IsEmpty() const
    {
        return pMin.x > pMax.x ||
               pMin.y > pMax.y ||
               pMin.z > pMax.z;
    }

    Vector3f Diagonal() const
    {
        return pMax - pMin;
    }

    Vector3f Centroid() const
    {
        return (pMin + pMax) * 0.5f;
    }

    float SurfaceArea() const
    {
        if (IsEmpty())
            return 0.0f;

        const Vector3f diagonal = Diagonal();
        return 2.0f *
            (diagonal.x * diagonal.y +
             diagonal.x * diagonal.z +
             diagonal.y * diagonal.z);
    }

    int MaximumExtent() const
    {
        const Vector3f diagonal = Diagonal();

        if (diagonal.x > diagonal.y &&
            diagonal.x > diagonal.z)
        {
            return 0;
        }

        if (diagonal.y > diagonal.z)
            return 1;

        return 2;
    }

    bool IntersectP(
        const Ray& ray,
        float tMin,
        float tMax) const
    {
        if (IsEmpty() || tMin > tMax)
            return false;

        for (int axis = 0; axis < 3; ++axis)
        {
            const float origin =
                axis == 0 ? ray.orig.x :
                axis == 1 ? ray.orig.y :
                            ray.orig.z;
            const float direction =
                axis == 0 ? ray.dir.x :
                axis == 1 ? ray.dir.y :
                            ray.dir.z;
            const float minimum =
                axis == 0 ? pMin.x :
                axis == 1 ? pMin.y :
                            pMin.z;
            const float maximum =
                axis == 0 ? pMax.x :
                axis == 1 ? pMax.y :
                            pMax.z;

            if (direction == 0.0f)
            {
                if (origin < minimum ||
                    origin > maximum)
                {
                    return false;
                }

                continue;
            }

            const float inverseDirection =
                1.0f / direction;
            float nearT =
                (minimum - origin) *
                inverseDirection;
            float farT =
                (maximum - origin) *
                inverseDirection;

            if (nearT > farT)
                std::swap(nearT, farT);

            tMin = std::max(tMin, nearT);
            tMax = std::min(tMax, farT);

            if (tMin > tMax)
                return false;
        }

        return true;
    }

    Vector3f pMin;
    Vector3f pMax;
};

inline Bounds3f Union(
    const Bounds3f& bounds,
    const Vector3f& point)
{
    return Bounds3f(
        Vector3f(
            std::min(bounds.pMin.x, point.x),
            std::min(bounds.pMin.y, point.y),
            std::min(bounds.pMin.z, point.z)),
        Vector3f(
            std::max(bounds.pMax.x, point.x),
            std::max(bounds.pMax.y, point.y),
            std::max(bounds.pMax.z, point.z)));
}

inline Bounds3f Union(
    const Bounds3f& first,
    const Bounds3f& second)
{
    return Bounds3f(
        Vector3f(
            std::min(first.pMin.x, second.pMin.x),
            std::min(first.pMin.y, second.pMin.y),
            std::min(first.pMin.z, second.pMin.z)),
        Vector3f(
            std::max(first.pMax.x, second.pMax.x),
            std::max(first.pMax.y, second.pMax.y),
            std::max(first.pMax.z, second.pMax.z)));
}
