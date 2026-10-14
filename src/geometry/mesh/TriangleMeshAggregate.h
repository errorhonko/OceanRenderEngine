#pragma once

#include "geometry/acceleration/BVHAccel.h"
#include "geometry/mesh/MeshTriangle.h"
#include "geometry/mesh/TriangleMesh.h"

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>

class TriangleMeshAggregate final :
    public Hittable
{
public:
    TriangleMeshAggregate(
        std::shared_ptr<TriangleMesh> mesh,
        std::shared_ptr<Material> material,
        std::size_t maxPrimitivesInNode = 4)
        : mesh(std::move(mesh))
    {
        if (!this->mesh)
        {
            throw std::invalid_argument(
                "Triangle mesh aggregate requires a mesh.");
        }

        if (this->mesh->Triangles().empty())
        {
            throw std::invalid_argument(
                "Triangle mesh aggregate requires triangles.");
        }

        bvh = std::make_unique<BVHAccel>(
            CreateMeshHittables(
                this->mesh,
                material),
            maxPrimitivesInNode);
    }

    // One resolved binding per face. The legacy uniform render-material
    // constructor above remains valid, including nullptr call sites.
    TriangleMeshAggregate(std::shared_ptr<TriangleMesh> mesh,
        const SurfaceMaterialBindings& bindings,
        std::size_t maxPrimitivesInNode = 4)
        : mesh(std::move(mesh))
    {
        if (!this->mesh || this->mesh->Triangles().empty())
            throw std::invalid_argument("Triangle mesh aggregate requires a nonempty mesh.");
        bvh = std::make_unique<BVHAccel>(
            CreateMeshHittables(this->mesh, bindings), maxPrimitivesInNode);
    }

    Bounds3f Bounds() const override
    {
        return bvh->Bounds();
    }

    bool hit(
        const Ray& ray,
        float tMin,
        float tMax,
        HitRecord& rec) const override
    {
        return bvh->hit(
            ray,
            tMin,
            tMax,
            rec);
    }

    void Refit()
    {
        bvh->Refit();
    }

    void Rebuild()
    {
        bvh->Rebuild();
    }

    const std::shared_ptr<TriangleMesh>&
        Mesh() const
    {
        return mesh;
    }

private:
    std::shared_ptr<TriangleMesh> mesh;
    std::unique_ptr<BVHAccel> bvh;
};
