#pragma once

#include "Hittable.h"

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>
#include <algorithm>

class BVHAccel final : public Hittable
{
private:
    struct BVHPrimitiveInfo
    {
        std::size_t primitiveIndex = 0;
        Bounds3f bounds;
        Vector3f centroid;

        BVHPrimitiveInfo(
            std::size_t primitiveIndex,
            const Bounds3f& bounds)
            : primitiveIndex(primitiveIndex),
            bounds(bounds),
            centroid(bounds.Centroid())
        {
        }
    };

    struct BVHBuildNode
    {
        Bounds3f bounds;

        std::unique_ptr<BVHBuildNode> left;
        std::unique_ptr<BVHBuildNode> right;

        std::size_t firstPrimitiveOffset = 0;
        std::size_t primitiveCount = 0;

        int splitAxis = 0;

        bool IsLeaf() const
        {
            return primitiveCount > 0;
        }

        void InitLeaf(
            std::size_t firstPrimitiveOffset,
            std::size_t primitiveCount,
            const Bounds3f& bounds)
        {
            this->firstPrimitiveOffset =
                firstPrimitiveOffset;
            this->primitiveCount =
                primitiveCount;
            this->bounds = bounds;

            left.reset();
            right.reset();
        }

        void InitInterior(
            int splitAxis,
            std::unique_ptr<BVHBuildNode> left,
            std::unique_ptr<BVHBuildNode> right)
        {
            this->splitAxis = splitAxis;
            this->left = std::move(left);
            this->right = std::move(right);

            primitiveCount = 0;

            bounds = Union(
                this->left->bounds,
                this->right->bounds);
        }
    };

public:
    explicit BVHAccel(
        std::vector<std::shared_ptr<Hittable>> primitives,
        std::size_t maxPrimitivesInNode = 4)
        : maxPrimitivesInNode(maxPrimitivesInNode),
        primitives(std::move(primitives))
    {
        if (this->primitives.empty())
        {
            throw std::invalid_argument(
                "BVH requires at least one primitive.");
        }

        if (this->maxPrimitivesInNode == 0)
        {
            throw std::invalid_argument(
                "BVH leaf capacity must be positive.");
        }

        Rebuild();
    }
    void Rebuild()
    {
        std::vector<BVHPrimitiveInfo> primitiveInfo;
        primitiveInfo.reserve(primitives.size());

        for (std::size_t primitiveIndex = 0;
            primitiveIndex < primitives.size();
            ++primitiveIndex)
        {
            const auto& primitive =
                primitives[primitiveIndex];

            if (!primitive)
            {
                throw std::invalid_argument(
                    "BVH cannot contain a null primitive.");
            }

            const Bounds3f bounds =
                primitive->Bounds();

            if (bounds.IsEmpty())
            {
                throw std::invalid_argument(
                    "BVH primitive has empty bounds.");
            }

            primitiveInfo.emplace_back(
                primitiveIndex,
                bounds);
        }

        std::vector<std::shared_ptr<Hittable>>
            orderedPrimitives;

        orderedPrimitives.reserve(
            primitives.size());

        std::unique_ptr<BVHBuildNode> newRoot =
            BuildRecursive(
                primitiveInfo,
                0,
                primitiveInfo.size(),
                orderedPrimitives);

        primitives =
            std::move(orderedPrimitives);

        root =
            std::move(newRoot);
    }
    void Refit()
    {
        if (!root)
            return;

        RefitNode(root.get());
    }


    Bounds3f Bounds() const override
    {
        if (!root)
            return Bounds3f();

        return root->bounds;
    }

    bool hit(
        const Ray& ray,
        float tMin,
        float tMax,
        HitRecord& rec) const override
    {
        if (!root)
            return false;

        return HitNode(
            root.get(),
            ray,
            tMin,
            tMax,
            rec);
    }

private:
    Bounds3f RefitNode(
        BVHBuildNode* node)
    {
        if (!node)
        {
            throw std::logic_error(
                "BVH refit encountered a null node.");
        }

        if (node->IsLeaf())
        {
            Bounds3f leafBounds;

            for (std::size_t index = 0;
                index < node->primitiveCount;
                ++index)
            {
                const std::size_t primitiveIndex =
                    node->firstPrimitiveOffset +
                    index;

                const auto& primitive =
                    primitives[primitiveIndex];

                if (!primitive)
                {
                    throw std::logic_error(
                        "BVH refit encountered a null primitive.");
                }

                const Bounds3f primitiveBounds =
                    primitive->Bounds();

                if (primitiveBounds.IsEmpty())
                {
                    throw std::invalid_argument(
                        "BVH primitive has empty bounds during refit.");
                }

                leafBounds = Union(
                    leafBounds,
                    primitiveBounds);
            }

            node->bounds = leafBounds;
            return node->bounds;
        }

        if (!node->left || !node->right)
        {
            throw std::logic_error(
                "BVH interior node has a missing child.");
        }

        const Bounds3f leftBounds =
            RefitNode(
                node->left.get());

        const Bounds3f rightBounds =
            RefitNode(
                node->right.get());

        node->bounds = Union(
            leftBounds,
            rightBounds);

        return node->bounds;
    }
    bool HitNode(
        const BVHBuildNode* node,
        const Ray& ray,
        float tMin,
        float tMax,
        HitRecord& rec) const
    {
        if (!node)
            return false;

        if (!node->bounds.IntersectP(
            ray,
            tMin,
            tMax))
        {
            return false;
        }

        if (node->IsLeaf())
        {
            bool hitAnything = false;
            float closestSoFar = tMax;

            HitRecord temporaryRecord;

            for (std::size_t index = 0;
                index < node->primitiveCount;
                ++index)
            {
                const std::size_t primitiveIndex =
                    node->firstPrimitiveOffset +
                    index;

                if (primitives[primitiveIndex]->hit(
                    ray,
                    tMin,
                    closestSoFar,
                    temporaryRecord))
                {
                    hitAnything = true;
                    closestSoFar =
                        temporaryRecord.t;
                    rec = temporaryRecord;
                }
            }

            return hitAnything;
        }

        const BVHBuildNode* firstChild =
            node->left.get();

        const BVHBuildNode* secondChild =
            node->right.get();

        if (Coordinate(
            ray.dir,
            node->splitAxis) < 0.0f)
        {
            std::swap(
                firstChild,
                secondChild);
        }

        const bool hitFirst =
            HitNode(
                firstChild,
                ray,
                tMin,
                tMax,
                rec);

        const float secondTMax =
            hitFirst ? rec.t : tMax;

        const bool hitSecond =
            HitNode(
                secondChild,
                ray,
                tMin,
                secondTMax,
                rec);

        return hitFirst || hitSecond;
    }

    std::unique_ptr<BVHBuildNode> BuildRecursive(
        std::vector<BVHPrimitiveInfo>& primitiveInfo,
        std::size_t start,
        std::size_t end,
        std::vector<std::shared_ptr<Hittable>>&
        orderedPrimitives)
    {
        if (start >= end)
        {
            throw std::logic_error(
                "BVH received an empty build range.");
        }

        auto node =
            std::make_unique<BVHBuildNode>();

        Bounds3f bounds;

        for (std::size_t index = start;
            index < end;
            ++index)
        {
            bounds = Union(
                bounds,
                primitiveInfo[index].bounds);
        }

        const std::size_t primitiveCount =
            end - start;

        auto createLeaf =
            [&]() -> std::unique_ptr<BVHBuildNode>
            {
                const std::size_t firstPrimitiveOffset =
                    orderedPrimitives.size();

                for (std::size_t index = start;
                    index < end;
                    ++index)
                {
                    const std::size_t primitiveIndex =
                        primitiveInfo[index]
                        .primitiveIndex;

                    orderedPrimitives.push_back(
                        primitives[primitiveIndex]);
                }

                node->InitLeaf(
                    firstPrimitiveOffset,
                    primitiveCount,
                    bounds);

                return std::move(node);
            };

        if (primitiveCount <= maxPrimitivesInNode ||
            bounds.SurfaceArea() == 0.0f)
        {
            return createLeaf();
        }

        Bounds3f centroidBounds;

        for (std::size_t index = start;
            index < end;
            ++index)
        {
            centroidBounds = Union(
                centroidBounds,
                primitiveInfo[index].centroid);
        }

        const int splitAxis =
            centroidBounds.MaximumExtent();

        const float minimumCentroid =
            Coordinate(
                centroidBounds.pMin,
                splitAxis);

        const float maximumCentroid =
            Coordinate(
                centroidBounds.pMax,
                splitAxis);

        if (minimumCentroid == maximumCentroid)
        {
            return createLeaf();
        }

        const std::size_t middle =
            start + primitiveCount / 2;

        std::nth_element(
            primitiveInfo.begin() + start,
            primitiveInfo.begin() + middle,
            primitiveInfo.begin() + end,
            [splitAxis](
                const BVHPrimitiveInfo& first,
                const BVHPrimitiveInfo& second)
            {
                return Coordinate(
                    first.centroid,
                    splitAxis) <
                    Coordinate(
                        second.centroid,
                        splitAxis);
            });

        std::unique_ptr<BVHBuildNode> left =
            BuildRecursive(
                primitiveInfo,
                start,
                middle,
                orderedPrimitives);

        std::unique_ptr<BVHBuildNode> right =
            BuildRecursive(
                primitiveInfo,
                middle,
                end,
                orderedPrimitives);

        node->InitInterior(
            splitAxis,
            std::move(left),
            std::move(right));

        return node;
    }

    static float Coordinate(
        const Vector3f& value,
        int axis)
    {
        if (axis == 0)
            return value.x;

        if (axis == 1)
            return value.y;

        return value.z;
    }

    std::size_t maxPrimitivesInNode = 4;

    std::vector<std::shared_ptr<Hittable>>
        primitives;

    std::unique_ptr<BVHBuildNode> root;
};