#include "OceanSurfaceMesh.h"

#include "OceanFrequencyField.h"
#include "OceanHeightField.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
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

bool Near(float a, float b, float epsilon = 1e-5f)
{
    return std::fabs(a - b) <= epsilon;
}

bool VectorNear(
    const Vector3f& a,
    const Vector3f& b,
    float epsilon = 1e-5f)
{
    return Near(a.x, b.x, epsilon) &&
           Near(a.y, b.y, epsilon) &&
           Near(a.z, b.z, epsilon);
}
}

void RunOceanSurfaceMeshAcceptanceTests()
{
    constexpr int resolution = 4;
    constexpr float patchLength = 8.0f;
    constexpr float spacing =
        patchLength / static_cast<float>(resolution);

    OceanFrequencyConfig config;
    config.resolution = resolution;
    config.patchLength = patchLength;
    config.seed = 17;

    const auto dispersion =
        [](float k)
        {
            return 1.0f + k;
        };

    OceanFrequencyField zeroFrequencyField(
        config,
        [](float, float) { return 0.0f; },
        dispersion);
    OceanHeightField zeroHeightField(zeroFrequencyField);
    zeroHeightField.Update(0.0f);

    OceanSurfaceMesh mesh(resolution, patchLength);
    mesh.Update(zeroHeightField);

    ExpectTrue(
        "ocean surface mesh resolution",
        mesh.Resolution() == resolution);

    ExpectTrue(
        "ocean surface mesh patch length",
        mesh.PatchLength() == patchLength);

    ExpectTrue(
        "ocean surface mesh vertex count",
        mesh.Vertices().size() ==
            static_cast<std::size_t>(resolution + 1) *
            static_cast<std::size_t>(resolution + 1));

    ExpectTrue(
        "ocean surface mesh triangle count",
        mesh.Triangles().size() ==
            2 * static_cast<std::size_t>(resolution) *
            static_cast<std::size_t>(resolution));

    const std::size_t rowSize =
        static_cast<std::size_t>(resolution + 1);
    const auto index =
        [rowSize](int x, int z)
        {
            return static_cast<std::size_t>(z) * rowSize +
                   static_cast<std::size_t>(x);
        };

    const MeshVertex& lowerLeft = mesh.Vertices()[index(0, 0)];
    const MeshVertex& upperRight =
        mesh.Vertices()[index(resolution, resolution)];
    ExpectTrue(
        "ocean surface mesh positions and uv",
        VectorNear(
            lowerLeft.position,
            Vector3f(-4.0f, 0.0f, -4.0f)) &&
        VectorNear(
            upperRight.position,
            Vector3f(4.0f, 0.0f, 4.0f)) &&
        VectorNear(lowerLeft.uv, Vector3f(0.0f, 0.0f, 0.0f)) &&
        VectorNear(upperRight.uv, Vector3f(1.0f, 1.0f, 0.0f)));

    bool flatValuesAreCorrect = true;
    for (const MeshVertex& vertex : mesh.Vertices())
    {
        flatValuesAreCorrect =
            flatValuesAreCorrect &&
            vertex.position.y == 0.0f &&
            VectorNear(vertex.normal, Vector3f(0.0f, 1.0f, 0.0f));
    }
    ExpectTrue(
        "ocean surface mesh flat height and normals",
        flatValuesAreCorrect);

    bool periodicSeamMatches = true;
    for (int z = 0; z <= resolution; ++z)
    {
        const MeshVertex& first = mesh.Vertices()[index(0, z)];
        const MeshVertex& last =
            mesh.Vertices()[index(resolution, z)];
        periodicSeamMatches =
            periodicSeamMatches &&
            Near(first.position.y, last.position.y) &&
            VectorNear(first.normal, last.normal);
    }
    for (int x = 0; x <= resolution; ++x)
    {
        const MeshVertex& first = mesh.Vertices()[index(x, 0)];
        const MeshVertex& last =
            mesh.Vertices()[index(x, resolution)];
        periodicSeamMatches =
            periodicSeamMatches &&
            Near(first.position.y, last.position.y) &&
            VectorNear(first.normal, last.normal);
    }
    ExpectTrue(
        "ocean surface mesh periodic seam",
        periodicSeamMatches);

    bool validIndicesAndWinding = true;
    for (const TriangleIndices& triangle : mesh.Triangles())
    {
        if (triangle[0] >= mesh.Vertices().size() ||
            triangle[1] >= mesh.Vertices().size() ||
            triangle[2] >= mesh.Vertices().size())
        {
            validIndicesAndWinding = false;
            break;
        }

        const Vector3f& p0 = mesh.Vertices()[triangle[0]].position;
        const Vector3f& p1 = mesh.Vertices()[triangle[1]].position;
        const Vector3f& p2 = mesh.Vertices()[triangle[2]].position;
        const Vector3f geometricNormal =
            (p1 - p0).cross(p2 - p0);
        validIndicesAndWinding =
            validIndicesAndWinding && geometricNormal.y > 0.0f;
    }
    ExpectTrue(
        "ocean surface mesh indices and winding",
        validIndicesAndWinding);

    const std::vector<TriangleIndices> originalTriangles =
        mesh.Triangles();

    OceanFrequencyField dynamicFrequencyField(
        config,
        [](float, float) { return 0.1f; },
        dispersion);
    OceanHeightField dynamicHeightField(dynamicFrequencyField);
    dynamicHeightField.Update(0.65f);
    mesh.Update(dynamicHeightField);

    bool heightsMatch = true;
    bool normalsMatch = true;
    for (int z = 0; z <= resolution; ++z)
    {
        for (int x = 0; x <= resolution; ++x)
        {
            const int sampleX = x % resolution;
            const int sampleZ = z % resolution;
            const MeshVertex& vertex = mesh.Vertices()[index(x, z)];

            heightsMatch =
                heightsMatch &&
                Near(
                    vertex.position.y,
                    dynamicHeightField.Height(sampleX, sampleZ));

            const int leftX =
                (sampleX - 1 + resolution) % resolution;
            const int rightX = (sampleX + 1) % resolution;
            const int backZ =
                (sampleZ - 1 + resolution) % resolution;
            const int frontZ = (sampleZ + 1) % resolution;
            const float dhdx =
                (dynamicHeightField.Height(rightX, sampleZ) -
                 dynamicHeightField.Height(leftX, sampleZ)) /
                (2.0f * spacing);
            const float dhdz =
                (dynamicHeightField.Height(sampleX, frontZ) -
                 dynamicHeightField.Height(sampleX, backZ)) /
                (2.0f * spacing);
            const Vector3f expectedNormal =
                Vector3f(-dhdx, 1.0f, -dhdz).normalize();

            normalsMatch =
                normalsMatch &&
                VectorNear(vertex.normal, expectedNormal) &&
                Near(vertex.normal.norm(), 1.0f) &&
                vertex.normal.y > 0.0f;
        }
    }

    ExpectTrue(
        "ocean surface mesh dynamic heights",
        heightsMatch);

    ExpectTrue(
        "ocean surface mesh central difference normals",
        normalsMatch);

    ExpectTrue(
        "ocean surface mesh topology remains stable",
        mesh.Triangles() == originalTriangles);

    OceanFrequencyConfig mismatchConfig = config;
    mismatchConfig.resolution = 8;
    OceanFrequencyField mismatchFrequencyField(
        mismatchConfig,
        [](float, float) { return 0.0f; },
        dispersion);
    OceanHeightField mismatchHeightField(mismatchFrequencyField);
    mismatchHeightField.Update(0.0f);

    ExpectThrows(
        "ocean surface mesh resolution mismatch rejected",
        [&mesh, &mismatchHeightField]
        {
            mesh.Update(mismatchHeightField);
        });

    ExpectThrows(
        "ocean surface mesh invalid resolution rejected",
        []
        {
            OceanSurfaceMesh invalidMesh(1, 8.0f);
        });

    ExpectThrows(
        "ocean surface mesh invalid patch length rejected",
        []
        {
            OceanSurfaceMesh invalidMesh(4, 0.0f);
        });

    ExpectThrows(
        "ocean surface mesh non-finite patch length rejected",
        []
        {
            OceanSurfaceMesh invalidMesh(
                4,
                std::numeric_limits<float>::quiet_NaN());
        });
}
