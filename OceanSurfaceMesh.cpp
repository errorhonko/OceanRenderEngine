#include "OceanSurfaceMesh.h"
#include "OceanHeightField.h"
#include <cmath>
#include <stdexcept>
int OceanSurfaceMesh::Wrap(
    int index) const
{
    index %= resolution;

    if (index < 0)
        index += resolution;

    return index;
}


OceanSurfaceMesh::OceanSurfaceMesh(
    int resolution,
    float patchLength)
    : resolution(resolution),
    patchLength(patchLength)
{
    if (resolution < 2)
    {
        throw std::invalid_argument(
            "Ocean mesh resolution "
            "must be at least 2.");
    }

    if (!std::isfinite(patchLength) ||
        patchLength <= 0.0f)
    {
        throw std::invalid_argument(
            "Ocean mesh patch length "
            "must be positive and finite.");
    }

    spacing =
        patchLength /
        static_cast<float>(resolution);

    const std::size_t vertexResolution =
        static_cast<std::size_t>(
            resolution + 1);

    std::vector<MeshVertex> vertices;
    vertices.resize(
        vertexResolution *
        vertexResolution);

    // X/Z 坐标和 UV 不随时间改变，
    // 因此只在构造时初始化一次。
    for (int z = 0;
        z <= resolution;
        ++z)
    {
        for (int x = 0;
            x <= resolution;
            ++x)
        {
            MeshVertex& vertex =
                vertices[VertexIndex(x, z)];

            vertex.position =
                Vector3f(
                    -0.5f * patchLength +
                    x * spacing,
                    0.0f,
                    -0.5f * patchLength +
                    z * spacing);

            vertex.normal =
                Vector3f(
                    0.0f,
                    1.0f,
                    0.0f);

            vertex.uv =
                Vector3f(
                    static_cast<float>(x) /
                    resolution,
                    static_cast<float>(z) /
                    resolution,
                    0.0f);
        }
    }

    std::vector<TriangleIndices> triangles;
    BuildIndices(triangles);

    mesh = std::make_shared<TriangleMesh>(
        std::move(vertices),
        std::move(triangles));
}

std::size_t OceanSurfaceMesh::VertexIndex(
    int x,
    int z) const
{
    const std::size_t vertexResolution =
        static_cast<std::size_t>(
            resolution + 1);

    return
        static_cast<std::size_t>(z) *
        vertexResolution +
        static_cast<std::size_t>(x);
}

Vector3f OceanSurfaceMesh::ComputeNormal(
    const OceanHeightField& heightField,
    int x,
    int z) const
{
    const int centerX =
        Wrap(x);

    const int centerZ =
        Wrap(z);

    const float left =
        heightField.Height(
            Wrap(centerX - 1),
            centerZ);

    const float right =
        heightField.Height(
            Wrap(centerX + 1),
            centerZ);

    const float back =
        heightField.Height(
            centerX,
            Wrap(centerZ - 1));

    const float front =
        heightField.Height(
            centerX,
            Wrap(centerZ + 1));

    const float dhdx =
        (right - left) /
        (2.0f * spacing);

    const float dhdz =
        (front - back) /
        (2.0f * spacing);

    return Vector3f(
        -dhdx,
        1.0f,
        -dhdz).normalize();
}

void OceanSurfaceMesh::BuildIndices(
    std::vector<TriangleIndices>& triangles) const
{
    triangles.clear();

    triangles.reserve(
        2 *
        static_cast<std::size_t>(
            resolution) *
        static_cast<std::size_t>(
            resolution));

    for (int z = 0;
        z < resolution;
        ++z)
    {
        for (int x = 0;
            x < resolution;
            ++x)
        {
            const std::uint32_t i00 =
                static_cast<std::uint32_t>(
                    VertexIndex(x, z));

            const std::uint32_t i10 =
                static_cast<std::uint32_t>(
                    VertexIndex(x + 1, z));

            const std::uint32_t i01 =
                static_cast<std::uint32_t>(
                    VertexIndex(x, z + 1));

            const std::uint32_t i11 =
                static_cast<std::uint32_t>(
                    VertexIndex(x + 1, z + 1));

            // 平坦海面时几何法线指向 +Y。
            triangles.push_back(
                { i00, i01, i10 });

            triangles.push_back(
                { i10, i01, i11 });
        }
    }
}

void OceanSurfaceMesh::Update(
    const OceanHeightField& heightField)
{
    if (heightField.Resolution() != resolution)
    {
        throw std::invalid_argument(
            "Ocean mesh and height field "
            "resolutions must match.");
    }

    for (int z = 0; z <= resolution; ++z)
    {
        for (int x = 0; x <= resolution; ++x)
        {
            const int sampleX = Wrap(x);
            const int sampleZ = Wrap(z);

            const std::size_t vertexIndex =
                VertexIndex(x, z);

            Vector3f position =
                mesh->Vertices()[vertexIndex].position;

            position.y =
                heightField.Height(
                    sampleX,
                    sampleZ);

            mesh->UpdateVertex(
                vertexIndex,
                position,
                ComputeNormal(
                    heightField,
                    sampleX,
                    sampleZ));
        }
    }
}
