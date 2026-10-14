# tinyobjloader dependency

- Upstream: https://github.com/tinyobjloader/tinyobjloader
- Source branch: `release`
- Pinned commit: `45636bdcef1a4fec140346b90c0b50bf0bc3e23b`
- Vendored on: 2026-10-13
- License: MIT; see the unmodified `LICENSE` and notices embedded in the header.

## Downloaded files (unmodified)

- Header: https://raw.githubusercontent.com/tinyobjloader/tinyobjloader/45636bdcef1a4fec140346b90c0b50bf0bc3e23b/tiny_obj_loader.h
  - SHA-256: `cffc9c8746541bd14a3ba877cd3c52cb7ccfe3779b1d5863f7d13f05d26f1ef8`
- License: https://raw.githubusercontent.com/tinyobjloader/tinyobjloader/45636bdcef1a4fec140346b90c0b50bf0bc3e23b/LICENSE
  - SHA-256: `19e1b0242709694c6671369609544993a2dfcef8477f1aa19bd30de35ec58caa`

## Integration

Include `third_party/tinyobjloader/tiny_obj_loader.h` from project code.
Only `TinyObjLoaderImplementation.cpp` defines `TINYOBJLOADER_IMPLEMENTATION`.
The renderer and CPU test executables each compile this source exactly once;
do not also compile upstream `tiny_obj_loader.cc` or define the macro in a header.
Default float precision and built-in triangulation are retained.

The implementation defines `TINYOBJLOADER_DISABLE_FAST_FLOAT`, using the
upstream-supported standard numeric parser: the pinned embedded fast_float
produces MSVC C3615 under this project's C++20 compiler. The implementation
source uses `/utf-8` in both VS projects. No upstream source is patched.

No optional Mapbox earcut dependency is enabled. Prefer triangulated assets for the initial
baseline; built-in triangulation is not a guarantee for arbitrary polygons.

This dependency parses OBJ/MTL only. The project's `ObjMeshLoader::Read()` and
`Convert()` adapt its output to owned, named `TriangleMesh` objects, retaining
per-triangle source material IDs. `ObjImportOptions` declares the positive
metres-per-source-unit scale and a signed source-to-engine axis permutation.
The engine target is right-handed Y-up in metres. Default options assume the
asset already uses that convention; OBJ units/orientation are not guessed.
Reflection mappings also swap triangle indices to preserve the physical front.
Source IDs refer to the reader's material table. The engine now supplies placement
through `src/scene/SceneBuilder.h`, material conversion through
`src/scene/import/ObjMaterialBinding.h`, and the owning `ObjSceneAsset` adapter.
These are engine features, not modifications to tinyobjloader itself.
`TinyObjLoaderImplementation.cpp` now lives in `src/scene/import/`; the legacy
loader and rasterizer remain available under `src/legacy/`.
