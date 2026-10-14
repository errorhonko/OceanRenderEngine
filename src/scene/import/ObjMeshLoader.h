#pragma once

#include "third_party/tinyobjloader/tiny_obj_loader.h"
#include "geometry/mesh/TriangleMesh.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

enum class ObjAxisDirection
{
    PositiveX, NegativeX,
    PositiveY, NegativeY,
    PositiveZ, NegativeZ
};

struct ObjImportOptions
{
    // Explicit number of metres per source length unit; OBJ does not declare it.
    // m: 1, cm: 0.01, mm: 0.001. Positive uniform unit conversion only.
    double metersPerUnit = 1.0;
    // Columns of the source-to-engine orthogonal axis mapping: source X/Y/Z
    // expressed in engine axes. Default is already engine Y-up, right-handed.
    // Z-up RH preserving X: {PositiveX, NegativeZ, PositiveY} => (x,z,-y).
    // Y-up LH: {PositiveX, PositiveY, NegativeZ} => (x,y,-z), with winding repair.
    std::array<ObjAxisDirection, 3> sourceAxesInEngine{
        ObjAxisDirection::PositiveX,
        ObjAxisDirection::PositiveY,
        ObjAxisDirection::PositiveZ};
    // Explicit opt-in only: skip exactly zero-area SOURCE triangles. No epsilon,
    // and no tolerance for malformed indices, nonfinite data or scale collapse.
    bool skipDegenerateTriangles = false;
};

struct ObjImportDiagnostics
{
    std::size_t importedTriangleCount = 0;
    std::size_t skippedDegenerateTriangleCount = 0;
};

struct ImportedMesh
{
    std::string name;
    std::shared_ptr<TriangleMesh> mesh;
    // One ID per triangle, indexing the source reader's material table.
    // -1 means unassigned. This does NOT assign engine/LiDAR materials.
    std::vector<int> materialIds;
};

// Engine geometry is right-handed, Y-up, in metres. Source conventions are
// declared explicitly; the default assumes the asset already uses this system.
// No inferred smoothing, scene placement, BVH or LiDAR material conversion.
class ObjMeshLoader
{
public:
    static tinyobj::ObjReader Read(const std::string& filePath)
    {
        if (filePath.empty())
        {
            throw std::invalid_argument(
                "OBJ file path must not be empty.");
        }

        tinyobj::ObjReaderConfig config;
        config.triangulate = true;
        config.vertex_color = false;
        // Default: resolve referenced MTL files relative to the OBJ directory.
        config.mtl_search_path.clear();

        tinyobj::ObjReader reader;
        if (!reader.ParseFromFile(filePath, config))
        {
            throw std::runtime_error(
                "Failed to load OBJ: " + filePath + "\n" + reader.Error());
        }

        // Preserve nonfatal warnings for the caller; do not print in the loader.
        return reader;
    }

    static std::vector<ImportedMesh> Convert(const tinyobj::ObjReader& reader,
        const ObjImportOptions& options = {}, ObjImportDiagnostics* diagnostics = nullptr)
    {
        if (!reader.Valid())
            throw std::invalid_argument("OBJ conversion requires a valid reader.");
        if (!(options.metersPerUnit > 0) || !std::isfinite(options.metersPerUnit))
            throw std::invalid_argument("OBJ metres per unit must be finite and positive.");
        const std::array<Vector3f, 3> axes{
            AxisVector(options.sourceAxesInEngine[0]),
            AxisVector(options.sourceAxesInEngine[1]),
            AxisVector(options.sourceAxesInEngine[2])};
        if (axes[0].dot(axes[1]) != 0 || axes[0].dot(axes[2]) != 0 || axes[1].dot(axes[2]) != 0)
            throw std::invalid_argument("OBJ source axes must map to three distinct engine axes.");
        const bool reverseWinding = axes[0].dot(axes[1].cross(axes[2])) < 0;

        const auto& attrib = reader.GetAttrib();
        if (attrib.vertices.size() % 3 || attrib.normals.size() % 3 ||
            attrib.texcoords.size() % 2)
            throw std::invalid_argument("OBJ attribute array has invalid dimensions.");

        ObjImportDiagnostics report; // Publish only after the whole conversion succeeds.
        std::vector<ImportedMesh> result;
        for (const auto& shape : reader.GetShapes())
        {
            const auto& source = shape.mesh;
            if (source.num_face_vertices.empty())
            {
                if (!source.indices.empty())
                    throw std::invalid_argument("OBJ shape has indices without faces.");
                continue; // Line/point-only shapes are outside this triangle baseline.
            }
            for (const auto count : source.num_face_vertices)
                if (count != 3)
                    throw std::invalid_argument("OBJ conversion requires triangulated faces.");
            if (source.indices.size() % 3 ||
                source.indices.size() / 3 != source.num_face_vertices.size())
                throw std::invalid_argument("OBJ face/index counts do not match.");
            if (!source.material_ids.empty() &&
                source.material_ids.size() != source.num_face_vertices.size())
                throw std::invalid_argument("OBJ material/face counts do not match.");

            std::vector<MeshVertex> vertices;
            std::vector<TriangleIndices> triangles;
            std::vector<int> materialIds;
            triangles.reserve(source.num_face_vertices.size());
            materialIds.reserve(source.num_face_vertices.size());
            using CornerKey = std::tuple<int, int, int>; // position, normal, UV
            std::map<CornerKey, std::uint32_t> remap;

            for (std::size_t face = 0; face < source.num_face_vertices.size(); ++face)
            {
                std::array<MeshVertex, 3> corners;
                bool useNormals = true;
                for (std::size_t corner = 0; corner < 3; ++corner)
                {
                    const auto& index = source.indices[face * 3 + corner];
                    corners[corner].position = ToEngine(
                        Attribute3(attrib.vertices, index.vertex_index), axes, options.metersPerUnit);
                    if (index.normal_index < -1 || index.texcoord_index < -1)
                        throw std::out_of_range("OBJ optional attribute index is invalid.");
                    if (index.normal_index == -1)
                        useNormals = false;
                    else
                    {
                        const auto normal = Attribute3(attrib.normals, index.normal_index);
                        const double length = std::hypot(double(normal.x), double(normal.y), double(normal.z));
                        if (length == 0)
                            useNormals = false;
                        else
                            corners[corner].normal = ToEngine(
                                {float(normal.x / length), float(normal.y / length), float(normal.z / length)},
                                axes, 1.0); // Normal direction does not receive the unit scale.
                    }
                    if (index.texcoord_index != -1)
                        corners[corner].uv = Attribute2(attrib.texcoords, index.texcoord_index);
                }
                const int materialId = source.material_ids.empty() ? -1 : source.material_ids[face];
                if (materialId < -1 || (materialId >= 0 &&
                    static_cast<std::size_t>(materialId) >= reader.GetMaterials().size()))
                    throw std::out_of_range("OBJ material index is out of range.");
                const auto p0 = Attribute3(attrib.vertices, source.indices[face * 3].vertex_index);
                const auto p1 = Attribute3(attrib.vertices, source.indices[face * 3 + 1].vertex_index);
                const auto p2 = Attribute3(attrib.vertices, source.indices[face * 3 + 2].vertex_index);
                if (options.skipDegenerateTriangles && ExactlyZeroArea(p0, p1, p2))
                {
                    ++report.skippedDegenerateTriangleCount;
                    continue;
                }
                const auto cross = (corners[1].position - corners[0].position).cross(
                    corners[2].position - corners[0].position);
                const float twiceArea = cross.norm();
                // Reject zero-area and float overflow/underflow, without a unit-dependent epsilon.
                if (!(twiceArea > 0) || !std::isfinite(twiceArea))
                    throw std::invalid_argument("OBJ triangle is degenerate or outside float geometry range.");

                TriangleIndices triangle;
                for (std::size_t corner = 0; corner < 3; ++corner)
                {
                    const auto& index = source.indices[face * 3 + corner];
                    // A partial/zero normal set falls back for the WHOLE face.
                    // Zero normals trigger MeshTriangle's per-hit geometric normal fallback,
                    // so sharing these vertices never smooths across adjacent faces.
                    if (!useNormals) corners[corner].normal = Vector3f(0, 0, 0);
                    const CornerKey key(index.vertex_index,
                        useNormals ? index.normal_index : -1, index.texcoord_index);
                    const auto found = remap.find(key);
                    if (found != remap.end()) triangle[corner] = found->second;
                    else
                    {
                        if (vertices.size() > std::numeric_limits<std::uint32_t>::max())
                            throw std::overflow_error("OBJ mesh exceeds uint32 vertex index range.");
                        const auto newIndex = static_cast<std::uint32_t>(vertices.size());
                        vertices.push_back(corners[corner]);
                        remap.emplace(key, newIndex);
                        triangle[corner] = newIndex;
                    }
                }
                // cross(Ce1, Ce2) = det(C) C cross(e1,e2). Reflections require
                // an index swap to keep geometric normals aligned with Cn.
                // UVs remain attached to their vertices; material IDs stay per face.
                if (reverseWinding) std::swap(triangle[1], triangle[2]);
                triangles.push_back(triangle);
                materialIds.push_back(materialId);
                ++report.importedTriangleCount;
            }
            if (triangles.empty()) continue; // A degenerate-only group is not a mesh.
            result.push_back({shape.name,
                std::make_shared<TriangleMesh>(std::move(vertices), std::move(triangles)),
                std::move(materialIds)});
        }
        if (result.empty())
            throw std::invalid_argument("OBJ document contains no triangle meshes.");
        if (diagnostics) *diagnostics = report;
        return result;
    }

private:
    static bool ExactlyZeroArea(const Vector3f& a, const Vector3f& b, const Vector3f& c)
    {
        const double ux = double(b.x)-a.x, uy = double(b.y)-a.y, uz = double(b.z)-a.z;
        const double vx = double(c.x)-a.x, vy = double(c.y)-a.y, vz = double(c.z)-a.z;
        return uy*vz-uz*vy == 0 && uz*vx-ux*vz == 0 && ux*vy-uy*vx == 0;
    }
    static Vector3f AxisVector(ObjAxisDirection direction)
    {
        switch (direction)
        {
        case ObjAxisDirection::PositiveX: return {1, 0, 0};
        case ObjAxisDirection::NegativeX: return {-1, 0, 0};
        case ObjAxisDirection::PositiveY: return {0, 1, 0};
        case ObjAxisDirection::NegativeY: return {0, -1, 0};
        case ObjAxisDirection::PositiveZ: return {0, 0, 1};
        case ObjAxisDirection::NegativeZ: return {0, 0, -1};
        default: throw std::invalid_argument("Unknown OBJ axis direction.");
        }
    }
    static Vector3f ToEngine(const Vector3f& value,
        const std::array<Vector3f, 3>& axes, double scale)
    {
        // Double arithmetic for unit conversion, then validate the float geometry boundary.
        const double x = scale * (double(axes[0].x) * value.x + double(axes[1].x) * value.y + double(axes[2].x) * value.z);
        const double y = scale * (double(axes[0].y) * value.x + double(axes[1].y) * value.y + double(axes[2].y) * value.z);
        const double z = scale * (double(axes[0].z) * value.x + double(axes[1].z) * value.y + double(axes[2].z) * value.z);
        const double limit = std::numeric_limits<float>::max();
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            std::abs(x) > limit || std::abs(y) > limit || std::abs(z) > limit)
            throw std::overflow_error("Converted OBJ coordinates exceed float geometry range.");
        return {float(x), float(y), float(z)};
    }
    static Vector3f Attribute3(const std::vector<tinyobj::real_t>& values, int index)
    {
        if (index < 0 || static_cast<std::size_t>(index) >= values.size() / 3)
            throw std::out_of_range("OBJ position/normal index is out of range.");
        const auto offset = static_cast<std::size_t>(index) * 3;
        const Vector3f value{float(values[offset]), float(values[offset + 1]), float(values[offset + 2])};
        if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z))
            throw std::invalid_argument("OBJ referenced attribute is not finite.");
        return value;
    }
    static Vector3f Attribute2(const std::vector<tinyobj::real_t>& values, int index)
    {
        if (index < 0 || static_cast<std::size_t>(index) >= values.size() / 2)
            throw std::out_of_range("OBJ UV index is out of range.");
        const auto offset = static_cast<std::size_t>(index) * 2;
        const Vector3f value{float(values[offset]), float(values[offset + 1]), 0};
        if (!std::isfinite(value.x) || !std::isfinite(value.y))
            throw std::invalid_argument("OBJ referenced UV is not finite.");
        return value;
    }
};
