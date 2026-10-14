# Static scene assembly baseline

## Geometry pipeline

`ObjMeshLoader::Read` -> `Convert(reader, importOptions)` ->
`SceneBuilder::AddMeshes(parts, pose)` -> `Build()` -> `StaticScene` (`Hittable`).

The import conversion produces right-handed Y-up metre coordinates using an
explicit positive source unit scale `s` and signed axis permutation `C`.
Placement then applies a proper rigid rotation `R` and metre translation `t`:

- Position: `p_world = R * (s * C * p_source) + t`.
- Supplied normal: normalized and transformed by `R * C`, not the length scale.
- Missing/partial/zero normal sets use the entire face's geometric-normal fallback.
- UVs remain attached to their vertices. Unit conversion is not texture scaling.
- Import reflections repair winding. Quaternion placement never mirrors geometry.

Scene placement rotates about the asset coordinate origin. No automatic centering,
ground snapping, general affine transform, nonuniform scaling or animation is done.

## Example

```cpp
#include "scene/SceneBuilder.h"

auto reader = ObjMeshLoader::Read("models/room.obj");
if (!reader.Warning().empty()) {
    // Report this to the caller before deciding to use the asset.
}
ObjImportOptions import; // Explicitly adjust if the file is not Y-up/metres.
auto parts = ObjMeshLoader::Convert(reader, import);

SceneObjectPose pose;
pose.translationMeters = {0, 0, 10};
// rotationWxyz is local-to-world, quaternion order w,x,y,z; identity by default.
SceneBuilder builder;
builder.AddMeshes(parts, pose);
auto scene = builder.Build();
// Pass *scene as the existing LidarScanDriver's world argument.
// Keep scene alive for the lifetime of the driver.
```

`AddMesh` returns an object index in insertion order. `AddMeshes` applies one
pose to all groups of an asset and validates the batch before appending anything.
Source geometry is copied on addition. Later source edits, builder additions,
reader destruction and builder destruction do not alter a previously built scene.
Engine material overrides are shared pointers, not deep appearance snapshots.

Each object has a mesh-level BVH; the scene has a top-level BVH over these objects.
`Objects()` exposes names, world bounds and triangle counts in insertion order.
Empty scenes are valid worlds with empty bounds and no hits. BVH leaf capacities
must still be positive. Static placement checks reject nonfinite attributes and
float-range overflow or triangle collapse; this does not remove existing small-
triangle/grazing numerical limitations in `MeshTriangle::hit`.

## Overlap semantics

- Overlapping AABBs are normal broad-phase candidates, not proof of collision.
- Intersecting objects are allowed. Queries return the nearest triangle surface.
- No boolean union or CSG is performed: internal faces are not removed. In
  particular, rays starting inside overlapping solids are not union-boundary queries.
- Exact coplanar equal-distance hits return one surface, but material/normal
  ownership follows existing BVH traversal and has no insertion-priority contract.
  Clean duplicate faces when ownership matters; no silent deduplication is done.
- The current opaque first-hit LiDAR estimator does not sum coincident surfaces
  or transmit one ray through them. A divergent beam can still contain multiple
  returns from different subrays sampling differently exposed targets.
- `PotentialBoundsOverlaps()` reports conservative object-index pairs, including
  touching bounds. It is an optional O(n^2) diagnostic, not used by ray traversal,
  and must not be treated as a triangle-intersection or collision solver.

## Material and feature boundaries

The optional `Material` argument binds one engine material to a whole mesh/group.
MTL material IDs are not automatically applied or converted to spectral reflectance.
The LiDAR driver still uses its explicitly supplied scattering model for the world.
Per-object/per-face LiDAR scattering selection, semantic/object hit IDs, texture
loading, transparent transport, instance sharing and a scene-file format remain
separate extensions. This static baseline copies vertices per placement rather
than sharing one mesh-level BVH across transformed instances.

## Reference and validation

The primitive/aggregate organization follows PBRT v4's architecture:
https://pbr-book.org/4ed/Primitives_and_Intersection_Acceleration/Primitive_Interface_and_Geometric_Primitives
The static baked-transform implementation is a deliberate simplification of
PBRT's transformed primitives, not a reproduction of its instance architecture.

`SceneBuilderAcceptance.cpp` checks placement, snapshot ownership, validation,
overlap/occlusion, conservative bounds diagnostics, coplanar first-hit energy,
OBJ-to-scene lifetime, and the full noise-free calibrated LiDAR scan pipeline.
