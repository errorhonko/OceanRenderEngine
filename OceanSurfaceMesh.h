#pragma once
#include "TriangleMesh.h"

#include <cstddef>
#include <memory>
#include <vector>

class OceanHeightField;

class OceanSurfaceMesh
{
public:
    OceanSurfaceMesh(
        int resolution,
        float patchLength);

    void Update(
        const OceanHeightField& heightField);

    int Resolution() const
    {
        return resolution;
    }

    float PatchLength() const
    {
        return patchLength;
    }

    const std::vector<MeshVertex>&
        Vertices() const
    {
        return mesh->Vertices();
    }

    const std::vector<TriangleIndices>&
        Triangles() const
    {
        return mesh->Triangles();
    }

    const std::shared_ptr<TriangleMesh>&
        Mesh() const
    {
        return mesh;
    }

private:
    int Wrap(int index) const;

    std::size_t VertexIndex(
        int x,
        int z) const;

    Vector3f ComputeNormal(
        const OceanHeightField& heightField,
        int x,
        int z) const;

    void BuildIndices(
        std::vector<TriangleIndices>& triangles) const;

    int resolution = 0;
    float patchLength = 0.0f;
    float spacing = 0.0f;

    std::shared_ptr<TriangleMesh> mesh;
};
