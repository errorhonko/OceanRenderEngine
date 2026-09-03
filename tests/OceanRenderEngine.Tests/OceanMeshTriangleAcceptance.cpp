#include "MeshTriangle.h"

#include "DiffuseMaterial.h"
#include "OceanFrequencyField.h"
#include "OceanHeightField.h"
#include "OceanSurfaceMesh.h"

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

void RunMeshTriangleAcceptanceTests()
{
    OceanFrequencyConfig config;
    config.resolution = 4;
    config.patchLength = 8.0f;
    config.seed = 23;

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

    OceanSurfaceMesh oceanSurface(
        config.resolution,
        config.patchLength);
    oceanSurface.Update(zeroHeightField);
    const std::shared_ptr<TriangleMesh> mesh =
        oceanSurface.Mesh();

    auto material =
        std::make_shared<DiffuseMaterial>(
            Spectrum(0.5f));

    MeshTriangle triangle(
        mesh,
        0,
        material);

    const Bounds3f initialBounds =
        triangle.Bounds();

    ExpectTrue(
        "ocean mesh triangle index",
        triangle.TriangleIndex() == 0);

    ExpectTrue(
        "ocean mesh triangle flat area",
        Near(triangle.SurfaceArea(), 2.0f));

    const TriangleIndices& indices =
        mesh->Triangles()[0];
    const MeshVertex& vertex0 =
        mesh->Vertices()[indices[0]];
    const MeshVertex& vertex1 =
        mesh->Vertices()[indices[1]];
    const MeshVertex& vertex2 =
        mesh->Vertices()[indices[2]];

    const Bounds3f expectedInitialBounds =
        Union(
            Union(
                Bounds3f(vertex0.position),
                vertex1.position),
            vertex2.position);

    ExpectTrue(
        "ocean mesh triangle initial bounds",
        VectorNear(
            initialBounds.pMin,
            expectedInitialBounds.pMin) &&
        VectorNear(
            initialBounds.pMax,
            expectedInitialBounds.pMax));

    const Vector3f centroid =
        (vertex0.position +
         vertex1.position +
         vertex2.position) / 3.0f;

    const Ray frontRay(
        Vector3f(centroid.x, 2.0f, centroid.z),
        Vector3f(0.0f, -1.0f, 0.0f));
    HitRecord frontRecord;
    const bool frontHit =
        triangle.hit(
            frontRay,
            1e-4f,
            std::numeric_limits<float>::infinity(),
            frontRecord);

    ExpectTrue(
        "ocean mesh triangle front hit",
        frontHit &&
        Near(frontRecord.t, 2.0f) &&
        VectorNear(frontRecord.point, centroid) &&
        VectorNear(
            frontRecord.geometricNormal,
            Vector3f(0.0f, 1.0f, 0.0f)) &&
        VectorNear(
            frontRecord.normal,
            Vector3f(0.0f, 1.0f, 0.0f)) &&
        frontRecord.material == material);

    ExpectTrue(
        "ocean mesh triangle interpolated attributes",
        Near(frontRecord.u, 1.0f / 12.0f) &&
        Near(frontRecord.v, 1.0f / 12.0f) &&
        VectorNear(
            frontRecord.dpdu,
            Vector3f(8.0f, 0.0f, 0.0f)));

    const Ray backRay(
        Vector3f(centroid.x, -2.0f, centroid.z),
        Vector3f(0.0f, 1.0f, 0.0f));
    HitRecord backRecord;
    ExpectTrue(
        "ocean mesh triangle double sided hit",
        triangle.hit(
            backRay,
            1e-4f,
            std::numeric_limits<float>::infinity(),
            backRecord) &&
        Near(backRecord.t, 2.0f) &&
        VectorNear(
            backRecord.geometricNormal,
            Vector3f(0.0f, 1.0f, 0.0f)));

    HitRecord rejectedRecord;
    ExpectTrue(
        "ocean mesh triangle t range rejection",
        !triangle.hit(
            frontRay,
            1e-4f,
            1.0f,
            rejectedRecord));

    const Ray missRay(
        Vector3f(0.0f, 2.0f, 0.0f),
        Vector3f(0.0f, -1.0f, 0.0f));
    HitRecord missRecord;
    ExpectTrue(
        "ocean mesh triangle outside miss",
        !triangle.hit(
            missRay,
            1e-4f,
            std::numeric_limits<float>::infinity(),
            missRecord));

    OceanFrequencyField dynamicFrequencyField(
        config,
        [](float, float) { return 0.1f; },
        dispersion);
    OceanHeightField dynamicHeightField(dynamicFrequencyField);
    dynamicHeightField.Update(0.4f);
    oceanSurface.Update(dynamicHeightField);

    const MeshVertex& updated0 =
        mesh->Vertices()[indices[0]];
    const MeshVertex& updated1 =
        mesh->Vertices()[indices[1]];
    const MeshVertex& updated2 =
        mesh->Vertices()[indices[2]];
    const Vector3f updatedCentroid =
        (updated0.position +
         updated1.position +
         updated2.position) / 3.0f;
    const Vector3f edge1 =
        updated1.position - updated0.position;
    const Vector3f edge2 =
        updated2.position - updated0.position;
    const float expectedUpdatedArea =
        0.5f * edge1.cross(edge2).norm();
    const Bounds3f updatedBounds =
        triangle.Bounds();
    const Bounds3f expectedUpdatedBounds =
        Union(
            Union(
                Bounds3f(updated0.position),
                updated1.position),
            updated2.position);

    const Ray updatedRay(
        Vector3f(
            updatedCentroid.x,
            updatedCentroid.y + 10.0f,
            updatedCentroid.z),
        Vector3f(0.0f, -1.0f, 0.0f));
    HitRecord updatedRecord;
    const bool updatedHit =
        triangle.hit(
            updatedRay,
            1e-4f,
            std::numeric_limits<float>::infinity(),
            updatedRecord);

    ExpectTrue(
        "ocean mesh triangle observes shared vertex update",
        updatedHit &&
        std::fabs(updatedCentroid.y) > 1e-6f &&
        VectorNear(updatedRecord.point, updatedCentroid, 2e-5f) &&
        Near(triangle.SurfaceArea(), expectedUpdatedArea));

    ExpectTrue(
        "ocean mesh triangle bounds observe shared vertex update",
        VectorNear(
            updatedBounds.pMin,
            expectedUpdatedBounds.pMin) &&
        VectorNear(
            updatedBounds.pMax,
            expectedUpdatedBounds.pMax) &&
        (!VectorNear(
             updatedBounds.pMin,
             initialBounds.pMin) ||
         !VectorNear(
             updatedBounds.pMax,
             initialBounds.pMax)));

    ExpectThrows(
        "ocean mesh triangle null mesh rejected",
        [&material]
        {
            MeshTriangle invalidTriangle(
                nullptr,
                0,
                material);
        });

    ExpectThrows(
        "ocean mesh triangle invalid index rejected",
        [&mesh, &material]
        {
            MeshTriangle invalidTriangle(
                mesh,
                mesh->Triangles().size(),
                material);
        });

    const auto allTriangles =
        CreateMeshTriangles(
            mesh,
            material);

    bool sequentialIndices =
        allTriangles.size() ==
        mesh->Triangles().size();

    for (std::size_t triangleIndex = 0;
         triangleIndex < allTriangles.size();
         ++triangleIndex)
    {
        sequentialIndices =
            sequentialIndices &&
            allTriangles[triangleIndex] &&
            allTriangles[triangleIndex]->TriangleIndex() ==
                triangleIndex;
    }

    ExpectTrue(
        "mesh triangle factory count and indices",
        sequentialIndices);

    ExpectThrows(
        "mesh triangle factory null mesh rejected",
        [&material]
        {
            CreateMeshTriangles(
                nullptr,
                material);
        });
}
