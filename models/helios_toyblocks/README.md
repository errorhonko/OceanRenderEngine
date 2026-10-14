# HELIOS++ Toyblocks local test assets

Upstream: https://github.com/3dgeo-heidelberg/helios
Pinned commit: `2b3585aa766213ae9a46c4790ad4f3594ffa213d`
Downloaded: 2026-10-13. Original OBJ, MTL, XML and license files are unmodified.

- `cube.*`, `sphere.*`, `cylinder.*`: `data/sceneparts/toyblocks/`.
- `groundplane.*`: `data/sceneparts/basic/groundplane/`.
- `toyblocks_scene.xml`: `data/scenes/toyblocks/toyblocks_scene.xml` (reference only).
- `UPSTREAM_LICENSE.md`, `COPYING`, `COPYING.LESSER`: upstream root license texts.

The root notices describe the HELIOS++ code licenses. Independent model licensing
has not been established from these files; do not assume the code license grants
all asset redistribution rights. Downloaded resources are excluded from Git to
avoid silently republishing them. Keep original notices when using these assets.

Run `FetchToyblocks.ps1` to recreate this local asset set at the pinned commit.
The script validates SHA-256 and refuses to overwrite a mismatched local file.
The external-data acceptance test reports SKIP if the complete OBJ/MTL set is
absent; the ordinary unit tests remain runnable without a network download.

The import experiment does not interpret HELIOS++ XML, spectral material tags,
automatic grounding, or reproduce its original survey. It converts source Z-up
geometry to engine Y-up with an explicit source-unit assumption of 1 metre/unit,
then applies a separate test-only uniform geometry resize and foot-centering.
This resize is NOT a unit conversion. MTL assignments are parsed and checked,
but the experiment uses one Lambertian LiDAR scattering model for every object.
