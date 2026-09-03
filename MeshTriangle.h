#pragma once

#include "Hittable.h"
#include "TriangleMesh.h"
#include "material.h"

#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

class MeshTriangle :
    public Hittable
{
public:
    MeshTriangle(
        std::shared_ptr<TriangleMesh> mesh,
        std::size_t triangleIndex,
        std::shared_ptr<Material> material)
        : mesh(std::move(mesh)),
          triangleIndex(triangleIndex),
          material(std::move(material))
    {
        if (!this->mesh)
        {
            throw std::invalid_argument(
                "Mesh triangle requires a mesh.");
        }

        if (triangleIndex >=
            this->mesh->Triangles().size())
        {
            throw std::out_of_range(
                "Mesh triangle index is out of range.");
        }
    }

    bool hit(
        const Ray& ray,
        float tMin,
        float tMax,
        HitRecord& rec) const override
    {
        const TriangleIndices& indices =
            mesh->Triangles()[triangleIndex];

        const std::vector<MeshVertex>& vertices =
            mesh->Vertices();

        const MeshVertex& vertex0 =
            vertices[indices[0]];
        const MeshVertex& vertex1 =
            vertices[indices[1]];
        const MeshVertex& vertex2 =
            vertices[indices[2]];

        const Vector3f& p0 = vertex0.position;
        const Vector3f& p1 = vertex1.position;
        const Vector3f& p2 = vertex2.position;

        const Vector3f edge1 = p1 - p0;
        const Vector3f edge2 = p2 - p0;
        const Vector3f pVector =
            ray.dir.cross(edge2);
        const float determinant =
            edge1.dot(pVector);

        if (std::fabs(determinant) < 1e-5f)
            return false;

        const float inverseDeterminant =
            1.0f / determinant;
        const Vector3f originOffset =
            ray.orig - p0;
        const float beta =
            originOffset.dot(pVector) *
            inverseDeterminant;

        if (beta < 0.0f || beta > 1.0f)
            return false;

        const Vector3f qVector =
            originOffset.cross(edge1);
        const float gamma =
            ray.dir.dot(qVector) *
            inverseDeterminant;

        if (gamma < 0.0f ||
            beta + gamma > 1.0f)
        {
            return false;
        }

        const float t =
            edge2.dot(qVector) *
            inverseDeterminant;

        if (t < tMin || t > tMax)
            return false;

        const float alpha =
            1.0f - beta - gamma;

        rec.t = t;
        rec.point = ray.at(t);
        rec.material = material;

        const Vector3f geometricNormal =
            edge1.cross(edge2).normalize();
        rec.geometricNormal = geometricNormal;

        Vector3f shadingNormal =
            vertex0.normal * alpha +
            vertex1.normal * beta +
            vertex2.normal * gamma;

        if (shadingNormal.near_zero())
        {
            shadingNormal = geometricNormal;
        }
        else
        {
            shadingNormal =
                shadingNormal.normalize();

            if (shadingNormal.dot(
                    geometricNormal) < 0.0f)
            {
                shadingNormal =
                    -shadingNormal;
            }
        }

        rec.normal = shadingNormal;

        rec.u =
            alpha * vertex0.uv.x +
            beta * vertex1.uv.x +
            gamma * vertex2.uv.x;
        rec.v =
            alpha * vertex0.uv.y +
            beta * vertex1.uv.y +
            gamma * vertex2.uv.y;

        const float du1 =
            vertex1.uv.x - vertex0.uv.x;
        const float dv1 =
            vertex1.uv.y - vertex0.uv.y;
        const float du2 =
            vertex2.uv.x - vertex0.uv.x;
        const float dv2 =
            vertex2.uv.y - vertex0.uv.y;
        const float uvDeterminant =
            du1 * dv2 - dv1 * du2;

        if (std::fabs(uvDeterminant) > 1e-8f)
        {
            const float inverseUvDeterminant =
                1.0f / uvDeterminant;

            rec.dpdu =
                (edge1 * dv2 -
                 edge2 * dv1) *
                inverseUvDeterminant;
        }
        else
        {
            Vector3f tangent =
                edge1 -
                shadingNormal *
                edge1.dot(shadingNormal);

            if (tangent.near_zero())
            {
                tangent =
                    edge2 -
                    shadingNormal *
                    edge2.dot(shadingNormal);
            }

            rec.dpdu = tangent.normalize();
        }

        SetHitAreaLight(rec);
        return true;
    }
    Bounds3f Bounds() const override
    {
        const TriangleIndices& indices =
            mesh->Triangles()[triangleIndex];

        const auto& vertices =
            mesh->Vertices();

        Bounds3f bounds(
            vertices[indices[0]].position);

        bounds = Union(
            bounds,
            vertices[indices[1]].position);

        bounds = Union(
            bounds,
            vertices[indices[2]].position);

        return bounds;
    }
    float SurfaceArea() const override
    {
        const TriangleIndices& indices =
            mesh->Triangles()[triangleIndex];
        const std::vector<MeshVertex>& vertices =
            mesh->Vertices();

        const Vector3f& p0 =
            vertices[indices[0]].position;
        const Vector3f& p1 =
            vertices[indices[1]].position;
        const Vector3f& p2 =
            vertices[indices[2]].position;

        return 0.5f *
            (p1 - p0).cross(p2 - p0).norm();
    }

    std::size_t TriangleIndex() const
    {
        return triangleIndex;
    }

private:
    std::shared_ptr<TriangleMesh> mesh;
    std::size_t triangleIndex = 0;
    std::shared_ptr<Material> material;
};

inline std::vector<std::shared_ptr<MeshTriangle>>
CreateMeshTriangles(
    const std::shared_ptr<TriangleMesh>& mesh,
    const std::shared_ptr<Material>& material)
{
    if (!mesh)
    {
        throw std::invalid_argument(
            "Mesh triangle collection requires a mesh.");
    }

    std::vector<std::shared_ptr<MeshTriangle>> triangles;
    triangles.reserve(mesh->Triangles().size());

    for (std::size_t triangleIndex = 0;
         triangleIndex < mesh->Triangles().size();
         ++triangleIndex)
    {
        triangles.push_back(
            std::make_shared<MeshTriangle>(
                mesh,
                triangleIndex,
                material));
    }

    return triangles;
}
