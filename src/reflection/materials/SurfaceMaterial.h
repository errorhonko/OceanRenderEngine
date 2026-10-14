#pragma once

#include <memory>
#include <vector>

class Material;
class LidarScatteringModel;

// PBRT-inspired primitive material binding, extended with a separate LiDAR
// model. Geometry stays independent; a LiDAR-only surface needs no fake BSDF.
// Bindings are fixed after construction. Models are shared, not copied per face.
struct SurfaceMaterial
{
    const std::shared_ptr<Material> renderMaterial;
    const std::shared_ptr<const LidarScatteringModel> lidarScattering;
};

// Resolved bindings, in TriangleMesh::Triangles() order, NOT OBJ material IDs.
// A nonnull binding with null models explicitly represents an unbound surface.
using SurfaceMaterialBindings = std::vector<std::shared_ptr<const SurfaceMaterial>>;
