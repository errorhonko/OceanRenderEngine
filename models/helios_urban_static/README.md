# HELIOS++ urban local resources

Source: https://github.com/3dgeo-heidelberg/helios
Pinned commit: `2b3585aa766213ae9a46c4790ad4f3594ffa213d`.
Fetched 2026-10-13. Originals remain unmodified.

The local set contains 27 OBJ files referenced by `data/scenes/dyn/urban_dyn_scene.xml`,
five MTL files, the original XML, and upstream root `LICENSE.md` (locally renamed
`UPSTREAM_LICENSE.md`), `COPYING`, `COPYING.LESSER`. About 54 MB total.
`FetchUrbanStatic.ps1` recreates this pinned set and checks each Git blob hash
against the upstream commit tree. Different existing files are not overwritten.

Files other than this README, downloader and .gitignore are intentionally local
and excluded from Git. Upstream code notices do not establish independent rights
for all buildings, car models, vegetation, textures or derived datasets; verify
applicable asset attribution/redistribution terms before publication. We do not
download or use the referenced raster textures in this static geometry baseline.

`HeliosUrbanStaticScene` is a fixed preset adapter, NOT a generic HELIOS XML loader.
It freezes both dynamic cars before the first motion, retains original layout,
applies scale/rotation/offset and handles the 18 `onGround=1` placements using a
fitted terrain plane. Source units assume metres, Z-up becomes engine Y-up.
The upstream internal whole-scene origin shift is deliberately not reproduced.

Three car OBJ files each contain six exactly zero-area triangles. The adapter
explicitly requests and reports their omission from generated meshes, leaving
the files unchanged. Generic OBJ conversion remains strict by default. No
arbitrary small-face epsilon, mesh simplification or independent asset centering.
MTL tables are parsed but spectral reflectance and semantic classifications are
not automatically assigned to the engine.

See `experiments/UrbanStaticImport.md` for implementation, checks and next steps.
