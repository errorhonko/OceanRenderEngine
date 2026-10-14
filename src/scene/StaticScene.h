#pragma once

#include "geometry/acceleration/BVHAccel.h"
#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class SceneBuilder;
struct SceneObjectInfo
{
    std::string name;
    Bounds3f bounds;
    std::size_t triangleCount = 0;
};

// Read-only static geometry snapshot; usable directly as the scanner's world.
// Surface collection, NOT CSG: overlaps are allowed and nearest surface wins.
// Exact equal-distance hits have unspecified material/normal ownership in the
// existing BVH. Remove duplicate coplanar faces when that distinction matters.
class StaticScene final : public Hittable
{
public:
    std::size_t ObjectCount() const { return objects.size(); }
    const std::vector<SceneObjectInfo>& Objects() const { return objects; }
    Bounds3f Bounds() const override { return root ? root->Bounds() : Bounds3f(); }
    bool hit(const Ray& ray, float tMin, float tMax, HitRecord& record) const override
    {
        return root && root->hit(ray, tMin, tMax, record);
    }
    // Optional O(n^2) broad-phase diagnostic. Touching boxes are included.
    // A reported pair is NOT proof of triangle collision or solid overlap.
    std::vector<std::array<std::size_t, 2>> PotentialBoundsOverlaps() const
    {
        std::vector<std::array<std::size_t, 2>> pairs;
        for (std::size_t i = 0; i < objects.size(); ++i)
            for (std::size_t j = i + 1; j < objects.size(); ++j)
            {
                const auto& a = objects[i].bounds;
                const auto& b = objects[j].bounds;
                if (a.pMin.x <= b.pMax.x && b.pMin.x <= a.pMax.x &&
                    a.pMin.y <= b.pMax.y && b.pMin.y <= a.pMax.y &&
                    a.pMin.z <= b.pMax.z && b.pMin.z <= a.pMax.z)
                    pairs.push_back({i, j});
            }
        return pairs;
    }
private:
    friend class SceneBuilder;
    StaticScene(std::vector<std::shared_ptr<Hittable>> primitives,
        std::vector<SceneObjectInfo> objects, std::size_t leafCapacity)
        : objects(std::move(objects))
    {
        if (!primitives.empty())
            root = std::make_unique<BVHAccel>(std::move(primitives), leafCapacity);
    }
    std::vector<SceneObjectInfo> objects;
    std::unique_ptr<BVHAccel> root;
};
