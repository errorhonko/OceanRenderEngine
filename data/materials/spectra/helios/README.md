# HELIOS++ starter reflectance spectra

Downloaded and verified 2026-10-19. This is the ten-file starter set distributed
with HELIOS++, not the entire ECOSTRESS library and not a complete measured BRDF
database. Existing engine source and scene bindings are unchanged.

## Source and reproducibility

- Repository: https://github.com/3dgeo-heidelberg/helios
- Pinned commit: `2b3585aa766213ae9a46c4790ad4f3594ffa213d`
- Original directory: `python/pyhelios/data/spectra/`
- Engine location: `data/materials/spectra/helios/`
- `FetchHeliosSpectra.ps1` downloads only the listed ten spectra and three upstream
  license documents. It verifies pinned Git blob hashes and refuses to overwrite
  a differing existing file. It also generates `download-manifest.json` containing
  original URLs, byte sizes, Git hashes and SHA-256 hashes.
- Original bytes are retained: headers, descriptions, sample conditions, original
  wavelength units and percent reflectance have not been rewritten or resampled.

To reproduce locally, run the fetcher in this directory. It does not load or run
any external program and does not download the whole HELIOS repository.

## Local acquisition vs redistribution

The original HELIOS `LICENSE.md` is retained as `UPSTREAM_LICENSE.md`, accompanied
by `COPYING` and `COPYING.LESSER`. These describe the source project's GPL/LGPL
licensing; they must not be assumed to grant an independent blanket license for
all underlying ASTER/JPL/JHU/other measurement data. Preserve the original headers
and consult the original data terms before publishing a bundled dataset.

The local `.gitignore` keeps downloaded data, acquisition manifests and copied
license files out of automatic Git staging. The fetcher and this README remain
trackable. No Git commit or push is performed by the fetcher.

ECOSTRESS source information: https://speclib.jpl.nasa.gov/
HELIOS material usage:
https://heliospp.readthedocs.io/en/alpha-dev/intensity_fwf.html#material-files

## Verified inventory

Bounds below are parsed bounds, not a claim of uniform wavelength sampling,
identical measurement geometry, or universally representative material behavior.
The rho(905 nm) column is dimensionless, linearly interpolated by the existing
reader, not an independently measured value at exactly 905 nm.

| File | Sample ID | Samples | Wavelength support (nm) | rho(905 nm) |
| --- | --- | ---: | --- | ---: |
| aluminum.txt | 0384UUUALM | 536 | 300-12500 | 0.5957000 |
| asphalt.txt | 0425UUUASP | 491 | 420-14000 | 0.2776305 |
| conifer.txt | conifers.doc | 550 | 302-14000 | 0.5150550 |
| copper.txt | 0682UUUCOP | 536 | 300-12500 | 0.5445250 |
| deciduous.txt | decidous.doc | 550 | 302-14000 | 0.5538775 |
| grass.txt | grass.doc | 550 | 302-14000 | 0.5085775 |
| grass_dry.txt | drygrass.doc | 2559 | 380.49-14011 | 0.59765848 |
| shingle_gray.txt | 0680UUUSTO | 551 | 300-14000 | 0.1893750 |
| shingle_red.txt | 0672UUUASP | 536 | 300-12500 | 0.0703000 |
| wood.txt | 0404UUUWOD | 536 | 300-12500 | 0.9182610 |

`asphalt` is a particular rough black asphaltic-concrete sample with exposed
aggregate; `wood` is a bare weathered pine sample, not all wood. `aluminum` is
weathered, scratched/scored bare metal. It is not the same sample or quantity as
the existing Rakic n/k table. Neither aluminum.txt nor copper.txt supplies n/k,
microfacet alpha, oxide thickness, or a full angular BRDF.

Most manmade samples declare directional (10 degree) hemispherical reflectance.
Vegetation files declare mixed bidirectional/directional-hemispherical spectra;
their descriptions document different instruments and spectral segments.
For example, dry grass's VNIR/SWIR measurement has overhead illumination and a
60 degree viewing angle. Treating that curve as Lambert rho is an explicit
modeling approximation, not an assertion that the source measured hemispherical
albedo at all wavelengths. Using a HELIOS angular model likewise introduces
an empirical angular assumption; it does not recover missing angular data.

## Existing engine integration

```cpp
ObjMaterialBindingOptions options;
options.spectraDirectory =
    "D:/sunmmer_camp/computerGraphics/OceanRenderEngine/data/materials/spectra/helios";
options.lidarAngularModel = ObjLidarAngularModel::Lambert;
// Or select ObjLidarAngularModel::HeliosEmpirical explicitly.
```

The existing MTL field `helios_spectra asphalt` now has a matching data file in
this directory. An explicit `helios_reflectance` still takes precedence, and
full SurfaceMaterial overrides remain authoritative. Setting the directory does
not fill the missing materials of an entire urban scene, enable transparency,
or change the strict missing-data policy. No OBJ/MTL file was modified.

The reader converts micrometers to nm and percent to rho exactly once. Data
loading is at setup, not in the ray loop. Existing LidarScatteringFactory accepts
the loaded curve with either LambertLidarResponse or the existing HELIOS response.
Existing optical constants continue through OpticalConstantsTableReader and the
GGX conductor configuration, separately from these reflectance curves.

## Verification scope

All 13 downloads match their pinned upstream Git blob hashes. The actual engine
reader loaded all ten spectra and retained name, sample ID and measurement
metadata. All ten support the tested 532, 905 and 1550 nm queries. Every source
sample node was checked for unchanged evaluation. Both spectral Lambert and the
diffuse branch of the HELIOS adapter were instantiated using the new factory;
their projected-response conventions agree. Out-of-band queries reject.

The acquisition check completed 7515 checks successfully. This is data/reader/
adapter verification, not validation against physical LiDAR measurements, a new
full-acceptance-suite run, or a claim of calibrated real-world material behavior.
