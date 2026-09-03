#pragma once

#include "Vector3f.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

struct MeshVertex
{
    Vector3f position;
    Vector3f normal;
    Vector3f uv;
};

using TriangleIndices =
    std::array<std::uint32_t, 3>;

class TriangleMesh
{
public:
    TriangleMesh(
        std::vector<MeshVertex> vertices,
        std::vector<TriangleIndices> triangles)
        : vertices(std::move(vertices)),
          triangles(std::move(triangles))
    {
        for (const TriangleIndices& triangle :
             this->triangles)
        {
            for (std::uint32_t index : triangle)
            {
                if (index >= this->vertices.size())
                {
                    throw std::out_of_range(
                        "Triangle mesh index is out of range.");
                }
            }
        }
    }

    const std::vector<MeshVertex>&
        Vertices() const
    {
        return vertices;
    }

    const std::vector<TriangleIndices>&
        Triangles() const
    {
        return triangles;
    }

    void UpdateVertex(
        std::size_t index,
        const Vector3f& position,
        const Vector3f& normal)
    {
        if (index >= vertices.size())
        {
            throw std::out_of_range(
                "Mesh vertex index is out of range.");
        }

        vertices[index].position = position;
        vertices[index].normal = normal;
    }

private:
    std::vector<MeshVertex> vertices;
    std::vector<TriangleIndices> triangles;
};
