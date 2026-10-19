# RefractiveIndex.INFO: one pinned aluminium data file

Only `Al/Rakic.yml` is included, not the database code or entire database.
It is an unmodified UTF-8/LF snapshot of:

- Repository: https://github.com/polyanskiy/refractiveindex.info-database
- Revision: `37773dce8128ccabcd9d5108d9958d6041244661`
- Original path: `database/data/main/Al/nk/Rakic.yml`
- Pinned raw URL: https://raw.githubusercontent.com/polyanskiy/refractiveindex.info-database/37773dce8128ccabcd9d5108d9958d6041244661/database/data/main/Al/nk/Rakic.yml
- SHA-256 of the original UTF-8/LF bytes: `e93b31fe821551836e5b000949c67c633b8a1ca1f2c5549f4db0f1c03b24844d`
- Retrieved: 2026-10-14

## License and attribution

The database contents are dedicated to the public domain under **CC0 1.0**.
The original three-line CC0 notice remains in the snapshot.
Database license statement: https://refractiveindex.info/about
CC0 legal code: https://creativecommons.org/publicdomain/zero/1.0/legalcode

Data reference: A. D. Rakic, *Algorithm for the determination of intrinsic optical
constants of metal films: application to aluminum*, Applied Optics 34, 4755-4767
(1995), https://doi.org/10.1364/AO.34.004755
Database reference: M. N. Polyanskiy, *Refractiveindex.info database of optical
constants*, Scientific Data 11, 94 (2024), https://doi.org/10.1038/s41597-023-02898-2

## Scientific scope

These are intrinsic optical constants reconstructed by an iterative analysis of
optical and electron-energy-loss data for evaporated aluminium films. They are
not a measured BRDF, LiDAR return calibration, universal alloy dataset, or
roughness/oxide/coating specification. No temperature or processing conditions
absent from the source are invented by this project.

The original table has 206 rows in micrometers. The engine-readable data file
`data/materials/optical_constants/Al_Rakic1995_um.txt` preserves a contiguous
26-row subset from 0.41328 to 2.4797 micrometers, without resampling, rounding,
or changes to any numeric token. Other wavelengths are deliberately excluded
from this first baseline; the original broad table is retained for provenance.

The runtime reader only supports strict three-column whitespace text. It does
not parse this YAML file; no YAML library or database-wide importer is included.
