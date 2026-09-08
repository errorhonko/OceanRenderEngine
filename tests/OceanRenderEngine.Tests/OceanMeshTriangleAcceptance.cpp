#include "MeshTriangle.h"

#include "BVHAccel.h"
#include "DiffuseMaterial.h"
#include "HittableList.h"
#include "OceanFrequencyField.h"
#include "OceanHeightField.h"
#include "OceanSurfaceMesh.h"
#include "TriangleMeshAggregate.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
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

    const auto meshHittables =
        CreateMeshHittables(
            mesh,
            material);

    bool validMeshHittables =
        meshHittables.size() ==
        mesh->Triangles().size();

    for (const auto& meshHittable : meshHittables)
    {
        validMeshHittables =
            validMeshHittables &&
            meshHittable != nullptr;
    }

    ExpectTrue(
        "mesh hittable factory count",
        validMeshHittables);

    ExpectThrows(
        "mesh hittable factory null mesh rejected",
        [&material]
        {
            CreateMeshHittables(
                nullptr,
                material);
        });

    HittableList linearMesh;

    for (const auto& meshHittable : meshHittables)
        linearMesh.add(meshHittable);

    BVHAccel meshBVH(
        meshHittables,
        2);

    const Bounds3f linearBounds =
        linearMesh.Bounds();
    const Bounds3f bvhBounds =
        meshBVH.Bounds();

    ExpectTrue(
        "ocean mesh BVH aggregate bounds",
        VectorNear(
            linearBounds.pMin,
            bvhBounds.pMin) &&
        VectorNear(
            linearBounds.pMax,
            bvhBounds.pMax));

    const std::vector<std::size_t>
        representativeTriangleIndices =
        {
            0,
            mesh->Triangles().size() / 2,
            mesh->Triangles().size() - 1
        };

    bool representativeHitsMatch = true;

    for (const std::size_t testTriangleIndex :
         representativeTriangleIndices)
    {
        const TriangleIndices& testIndices =
            mesh->Triangles()[testTriangleIndex];

        const Vector3f testCentroid =
            (mesh->Vertices()[testIndices[0]].position +
             mesh->Vertices()[testIndices[1]].position +
             mesh->Vertices()[testIndices[2]].position) /
            3.0f;

        const Ray testRay(
            Vector3f(
                testCentroid.x,
                testCentroid.y + 20.0f,
                testCentroid.z),
            Vector3f(0.0f, -1.0f, 0.0f));

        HitRecord linearRecord;
        HitRecord bvhRecord;

        const bool linearHit =
            linearMesh.hit(
                testRay,
                1e-4f,
                std::numeric_limits<float>::infinity(),
                linearRecord);

        const bool bvhHit =
            meshBVH.hit(
                testRay,
                1e-4f,
                std::numeric_limits<float>::infinity(),
                bvhRecord);

        representativeHitsMatch =
            representativeHitsMatch &&
            linearHit &&
            bvhHit &&
            Near(linearRecord.t, bvhRecord.t, 2e-5f) &&
            VectorNear(
                linearRecord.point,
                bvhRecord.point,
                2e-5f) &&
            VectorNear(
                linearRecord.geometricNormal,
                bvhRecord.geometricNormal,
                2e-5f) &&
            VectorNear(
                linearRecord.normal,
                bvhRecord.normal,
                2e-5f) &&
            linearRecord.material == bvhRecord.material;
    }

    ExpectTrue(
        "ocean mesh BVH matches linear hits",
        representativeHitsMatch);

    const Ray outsideMeshRay(
        Vector3f(
            config.patchLength,
            20.0f,
            0.0f),
        Vector3f(0.0f, -1.0f, 0.0f));
    HitRecord linearMissRecord;
    HitRecord bvhMissRecord;

    ExpectTrue(
        "ocean mesh BVH matches linear miss",
        !linearMesh.hit(
            outsideMeshRay,
            1e-4f,
            std::numeric_limits<float>::infinity(),
            linearMissRecord) &&
        !meshBVH.hit(
            outsideMeshRay,
            1e-4f,
            std::numeric_limits<float>::infinity(),
            bvhMissRecord));

    dynamicHeightField.Update(1.1f);
    oceanSurface.Update(dynamicHeightField);
    meshBVH.Refit();

    const Bounds3f updatedLinearBounds =
        linearMesh.Bounds();
    const Bounds3f refittedMeshBounds =
        meshBVH.Bounds();

    const std::size_t updatedTriangleIndex =
        mesh->Triangles().size() / 3;
    const TriangleIndices& updatedIndices =
        mesh->Triangles()[updatedTriangleIndex];
    const Vector3f refitTestCentroid =
        (mesh->Vertices()[updatedIndices[0]].position +
         mesh->Vertices()[updatedIndices[1]].position +
         mesh->Vertices()[updatedIndices[2]].position) /
        3.0f;

    const Ray refitTestRay(
        Vector3f(
            refitTestCentroid.x,
            refitTestCentroid.y + 20.0f,
            refitTestCentroid.z),
        Vector3f(0.0f, -1.0f, 0.0f));
    HitRecord updatedLinearRecord;
    HitRecord refittedMeshRecord;

    const bool updatedLinearHit =
        linearMesh.hit(
            refitTestRay,
            1e-4f,
            std::numeric_limits<float>::infinity(),
            updatedLinearRecord);

    const bool refittedMeshHit =
        meshBVH.hit(
            refitTestRay,
            1e-4f,
            std::numeric_limits<float>::infinity(),
            refittedMeshRecord);

    ExpectTrue(
        "ocean mesh BVH refit matches updated surface",
        VectorNear(
            updatedLinearBounds.pMin,
            refittedMeshBounds.pMin,
            2e-5f) &&
        VectorNear(
            updatedLinearBounds.pMax,
            refittedMeshBounds.pMax,
            2e-5f) &&
        updatedLinearHit &&
        refittedMeshHit &&
        Near(
            updatedLinearRecord.t,
            refittedMeshRecord.t,
            2e-5f) &&
        VectorNear(
            updatedLinearRecord.point,
            refittedMeshRecord.point,
            2e-5f) &&
        VectorNear(
            updatedLinearRecord.geometricNormal,
            refittedMeshRecord.geometricNormal,
            2e-5f) &&
        VectorNear(
            updatedLinearRecord.normal,
            refittedMeshRecord.normal,
            2e-5f) &&
        updatedLinearRecord.material ==
            refittedMeshRecord.material);

    ExpectThrows(
        "triangle mesh aggregate null mesh rejected",
        [&material]
        {
            TriangleMeshAggregate aggregate(
                nullptr,
                material);
        });

    ExpectThrows(
        "triangle mesh aggregate empty mesh rejected",
        [&material]
        {
            auto emptyMesh =
                std::make_shared<TriangleMesh>(
                    std::vector<MeshVertex>(),
                    std::vector<TriangleIndices>());

            TriangleMeshAggregate aggregate(
                emptyMesh,
                material);
        });

    std::vector<MeshVertex> aggregateVertices =
    {
        {
            Vector3f(-1.0f, 0.0f, -1.0f),
            Vector3f(0.0f, 1.0f, 0.0f),
            Vector3f(0.0f, 0.0f, 0.0f)
        },
        {
            Vector3f(-1.0f, 0.0f, 1.0f),
            Vector3f(0.0f, 1.0f, 0.0f),
            Vector3f(0.0f, 1.0f, 0.0f)
        },
        {
            Vector3f(1.0f, 0.0f, 1.0f),
            Vector3f(0.0f, 1.0f, 0.0f),
            Vector3f(1.0f, 1.0f, 0.0f)
        },
        {
            Vector3f(1.0f, 0.0f, -1.0f),
            Vector3f(0.0f, 1.0f, 0.0f),
            Vector3f(1.0f, 0.0f, 0.0f)
        }
    };

    std::vector<TriangleIndices> aggregateIndices =
    {
        TriangleIndices{ 0, 1, 2 },
        TriangleIndices{ 0, 2, 3 }
    };

    auto aggregateMesh =
        std::make_shared<TriangleMesh>(
            std::move(aggregateVertices),
            std::move(aggregateIndices));

    HittableList aggregateLinearMesh;
    const auto aggregateTriangles =
        CreateMeshHittables(
            aggregateMesh,
            material);

    for (const auto& aggregateTriangle :
         aggregateTriangles)
    {
        aggregateLinearMesh.add(
            aggregateTriangle);
    }

    TriangleMeshAggregate aggregate(
        aggregateMesh,
        material,
        1);

    ExpectTrue(
        "triangle mesh aggregate preserves shared mesh",
        aggregate.Mesh() == aggregateMesh);

    const Ray aggregateRay(
        Vector3f(-0.5f, 2.0f, 0.5f),
        Vector3f(0.0f, -1.0f, 0.0f));
    HitRecord aggregateLinearRecord;
    HitRecord aggregateRecord;

    const bool aggregateLinearHit =
        aggregateLinearMesh.hit(
            aggregateRay,
            1e-4f,
            std::numeric_limits<float>::infinity(),
            aggregateLinearRecord);

    const bool aggregateHit =
        aggregate.hit(
            aggregateRay,
            1e-4f,
            std::numeric_limits<float>::infinity(),
            aggregateRecord);

    const Bounds3f aggregateLinearBounds =
        aggregateLinearMesh.Bounds();
    const Bounds3f aggregateBounds =
        aggregate.Bounds();

    ExpectTrue(
        "triangle mesh aggregate matches linear mesh",
        VectorNear(
            aggregateLinearBounds.pMin,
            aggregateBounds.pMin) &&
        VectorNear(
            aggregateLinearBounds.pMax,
            aggregateBounds.pMax) &&
        aggregateLinearHit &&
        aggregateHit &&
        Near(
            aggregateLinearRecord.t,
            aggregateRecord.t) &&
        VectorNear(
            aggregateLinearRecord.point,
            aggregateRecord.point) &&
        aggregateRecord.material == material);

    for (std::size_t vertexIndex = 0;
         vertexIndex < aggregateMesh->Vertices().size();
         ++vertexIndex)
    {
        const Vector3f oldPosition =
            aggregateMesh->Vertices()[vertexIndex].position;

        aggregateMesh->UpdateVertex(
            vertexIndex,
            Vector3f(
                oldPosition.x,
                1.0f,
                oldPosition.z),
            Vector3f(0.0f, 1.0f, 0.0f));
    }

    aggregate.Refit();

    const Bounds3f refittedAggregateBounds =
        aggregate.Bounds();
    const Bounds3f refittedLinearBounds =
        aggregateLinearMesh.Bounds();
    const Ray refittedAggregateRay(
        Vector3f(-0.5f, 3.0f, 0.5f),
        Vector3f(0.0f, -1.0f, 0.0f));
    HitRecord refittedAggregateRecord;

    ExpectTrue(
        "triangle mesh aggregate refit observes vertex updates",
        VectorNear(
            refittedAggregateBounds.pMin,
            refittedLinearBounds.pMin) &&
        VectorNear(
            refittedAggregateBounds.pMax,
            refittedLinearBounds.pMax) &&
        Near(refittedAggregateBounds.pMin.y, 1.0f) &&
        Near(refittedAggregateBounds.pMax.y, 1.0f) &&
        aggregate.hit(
            refittedAggregateRay,
            1e-4f,
            std::numeric_limits<float>::infinity(),
            refittedAggregateRecord) &&
        Near(refittedAggregateRecord.t, 2.0f));

    for (std::size_t vertexIndex = 0;
         vertexIndex < aggregateMesh->Vertices().size();
         ++vertexIndex)
    {
        const Vector3f oldPosition =
            aggregateMesh->Vertices()[vertexIndex].position;

        aggregateMesh->UpdateVertex(
            vertexIndex,
            Vector3f(
                oldPosition.x,
                -1.0f,
                oldPosition.z),
            Vector3f(0.0f, 1.0f, 0.0f));
    }

    aggregate.Rebuild();

    const Bounds3f rebuiltAggregateBounds =
        aggregate.Bounds();
    const Ray rebuiltAggregateRay(
        Vector3f(-0.5f, 1.0f, 0.5f),
        Vector3f(0.0f, -1.0f, 0.0f));
    HitRecord rebuiltAggregateRecord;

    ExpectTrue(
        "triangle mesh aggregate rebuild observes vertex updates",
        Near(rebuiltAggregateBounds.pMin.y, -1.0f) &&
        Near(rebuiltAggregateBounds.pMax.y, -1.0f) &&
        aggregate.hit(
            rebuiltAggregateRay,
            1e-4f,
            std::numeric_limits<float>::infinity(),
            rebuiltAggregateRecord) &&
        Near(rebuiltAggregateRecord.t, 2.0f));
}
