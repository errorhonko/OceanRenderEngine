#pragma once

#include "scene/import/ObjMeshLoader.h"
#include "scene/StaticScene.h"
#include "geometry/mesh/TriangleMeshAggregate.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

struct SceneObjectPose
{
    Vector3f translationMeters{0, 0, 0};
    // Source mesh local to world, quaternion in w,x,y,z order.
    std::array<double, 4> rotationWxyz{1, 0, 0, 0};
};

// PBRT-inspired primitive/aggregate separation. Deliberate simplification:
// static rigid placement is baked into copied vertices, not TransformedPrimitive
// instancing. No nonuniform scale, animation, CSG or automatic MTL/LiDAR mapping.
class SceneBuilder
{
public:
    std::size_t ObjectCount() const { return objects.size(); }
    std::size_t AddMesh(const ImportedMesh& source, const SceneObjectPose& pose = {},
        std::shared_ptr<Material> material = nullptr)
    {
        auto object = Prepare(source, pose, std::move(material));
        const auto index = objects.size();
        objects.push_back(std::move(object));
        return index;
    }
    // One asset's groups share a rigid pose. Batch validation is atomic.
    void AddMeshes(const std::vector<ImportedMesh>& sources, const SceneObjectPose& pose = {},
        std::shared_ptr<Material> material = nullptr)
    {
        (void)Rotation(pose); // Validate even an empty batch.
        std::vector<PreparedObject> prepared;
        prepared.reserve(sources.size());
        for (const auto& source : sources)
            prepared.push_back(Prepare(source, pose, material));
        if (prepared.size() > objects.max_size() - objects.size())
            throw std::overflow_error("Scene object count exceeds container capacity.");
        objects.reserve(objects.size() + prepared.size());
        objects.insert(objects.end(), std::make_move_iterator(prepared.begin()),
            std::make_move_iterator(prepared.end()));
    }
    // Named API avoids overload ambiguity with legacy nullptr material calls.
    // Bindings are already resolved by the caller, not inferred from MTL/Kd.
    std::size_t AddMeshWithSurfaceMaterials(const ImportedMesh& source,
        const SurfaceMaterialBindings& bindings, const SceneObjectPose& pose = {})
    {
        if (!source.mesh || bindings.size() != source.mesh->Triangles().size())
            throw std::invalid_argument("Surface binding count must equal triangle count.");
        for (const auto& binding : bindings)
            if (!binding) throw std::invalid_argument("Scene surface bindings must be nonnull.");
        auto object = Prepare(source, pose, nullptr);
        object.surfaceBindings = bindings; // Snapshot vector; immutable pairs share models.
        const auto index = objects.size();
        objects.push_back(std::move(object));
        return index;
    }

    std::shared_ptr<StaticScene> Build(std::size_t meshLeafCapacity = 4,
        std::size_t sceneLeafCapacity = 4) const
    {
        return BuildScene(meshLeafCapacity, sceneLeafCapacity);
    }

    // Atomic multi-group import: validate and prepare every group before commit.
    void AddMeshesWithSurfaceMaterials(const std::vector<ImportedMesh>& sources,
        const std::vector<SurfaceMaterialBindings>& bindings, const SceneObjectPose& pose = {})
    {
        (void)Rotation(pose);
        if (bindings.size() != sources.size())
            throw std::invalid_argument("Surface binding groups must match imported mesh groups.");
        std::vector<PreparedObject> prepared; prepared.reserve(sources.size());
        for (std::size_t i = 0; i < sources.size(); ++i)
        {
            if (!sources[i].mesh || bindings[i].size() != sources[i].mesh->Triangles().size())
                throw std::invalid_argument("Surface binding count must equal triangle count.");
            for (const auto& binding : bindings[i])
                if (!binding) throw std::invalid_argument("Scene surface bindings must be nonnull.");
            auto object = Prepare(sources[i], pose, nullptr);
            object.surfaceBindings = bindings[i]; prepared.push_back(std::move(object));
        }
        if (prepared.size() > objects.max_size() - objects.size())
            throw std::overflow_error("Scene object count exceeds container capacity.");
        objects.reserve(objects.size() + prepared.size());
        objects.insert(objects.end(), std::make_move_iterator(prepared.begin()),
            std::make_move_iterator(prepared.end()));
    }

private:
    std::shared_ptr<StaticScene> BuildScene(std::size_t meshLeafCapacity,
        std::size_t sceneLeafCapacity = 4) const
    {
        if (meshLeafCapacity == 0 || sceneLeafCapacity == 0)
            throw std::invalid_argument("Scene and mesh BVH leaf capacities must be positive.");
        std::vector<std::shared_ptr<Hittable>> primitives;
        std::vector<SceneObjectInfo> info;
        primitives.reserve(objects.size());
        info.reserve(objects.size());
        for (const auto& object : objects)
        {
            auto aggregate = object.surfaceBindings.empty() ?
                std::make_shared<TriangleMeshAggregate>(object.mesh, object.material, meshLeafCapacity) :
                std::make_shared<TriangleMeshAggregate>(object.mesh, object.surfaceBindings, meshLeafCapacity);
            info.push_back({object.name, aggregate->Bounds(), object.mesh->Triangles().size()});
            primitives.push_back(std::move(aggregate));
        }
        // Private constructor: shared_ptr owns it immediately. The scene outlives
        // this builder and the source readers/meshes; geometry is not exposed mutable.
        return std::shared_ptr<StaticScene>(new StaticScene(
            std::move(primitives), std::move(info), sceneLeafCapacity));
    }
private:
    struct PreparedObject
    {
        std::string name;
        std::shared_ptr<TriangleMesh> mesh;
        std::shared_ptr<Material> material;
        SurfaceMaterialBindings surfaceBindings;
    };
    using RotationMatrix = std::array<std::array<double, 3>, 3>;
    static bool Finite(const Vector3f& v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }
    static RotationMatrix Rotation(const SceneObjectPose& pose)
    {
        if (!Finite(pose.translationMeters))
            throw std::invalid_argument("Scene translation must be finite metres.");
        auto q = pose.rotationWxyz;
        double largest = 0;
        for (const auto value : q)
        {
            if (!std::isfinite(value))
                throw std::invalid_argument("Scene quaternion must be finite.");
            largest = std::max(largest, std::abs(value));
        }
        if (largest == 0)
            throw std::invalid_argument("Scene quaternion must be nonzero.");
        double lengthSquared = 0;
        for (auto& value : q) { value /= largest; lengthSquared += value * value; }
        const double length = std::sqrt(lengthSquared);
        for (auto& value : q) value /= length;
        const double w = q[0], x = q[1], y = q[2], z = q[3];
        return {{{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
                 {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
                 {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}}};
    }
    static Vector3f Transform(const Vector3f& value, const RotationMatrix& rotation,
        const Vector3f& translation)
    {
        if (!Finite(value)) throw std::invalid_argument("Scene mesh attributes must be finite.");
        const double x = rotation[0][0] * value.x + rotation[0][1] * value.y + rotation[0][2] * value.z + translation.x;
        const double y = rotation[1][0] * value.x + rotation[1][1] * value.y + rotation[1][2] * value.z + translation.y;
        const double z = rotation[2][0] * value.x + rotation[2][1] * value.y + rotation[2][2] * value.z + translation.z;
        const double limit = std::numeric_limits<float>::max();
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            std::abs(x) > limit || std::abs(y) > limit || std::abs(z) > limit)
            throw std::overflow_error("Scene transformed coordinates exceed float range.");
        return {float(x), float(y), float(z)};
    }
    static PreparedObject Prepare(const ImportedMesh& source, const SceneObjectPose& pose,
        std::shared_ptr<Material> material)
    {
        if (!source.mesh || source.mesh->Triangles().empty())
            throw std::invalid_argument("Scene object requires a nonempty triangle mesh.");
        const auto rotation = Rotation(pose);
        auto vertices = source.mesh->Vertices(); // Snapshot, never move/modify the source mesh.
        for (auto& vertex : vertices)
        {
            vertex.position = Transform(vertex.position, rotation, pose.translationMeters);
            if (!Finite(vertex.normal) || !Finite(vertex.uv))
                throw std::invalid_argument("Scene normal/UV attributes must be finite.");
            const double length = std::hypot(double(vertex.normal.x), double(vertex.normal.y), double(vertex.normal.z));
            if (length > 0)
                vertex.normal = Transform({float(vertex.normal.x / length),
                    float(vertex.normal.y / length), float(vertex.normal.z / length)}, rotation, {0, 0, 0});
            // Exactly zero remains MeshTriangle's flat geometric normal marker.
        }
        for (const auto& indices : source.mesh->Triangles())
        {
            const float twiceArea = (vertices[indices[1]].position - vertices[indices[0]].position).cross(
                vertices[indices[2]].position - vertices[indices[0]].position).norm();
            if (!(twiceArea > 0) || !std::isfinite(twiceArea))
                throw std::invalid_argument("Scene placement collapses or overflows triangle geometry.");
        }
        return {source.name, std::make_shared<TriangleMesh>(
            std::move(vertices), source.mesh->Triangles()), std::move(material)};
    }
    std::vector<PreparedObject> objects;
};
