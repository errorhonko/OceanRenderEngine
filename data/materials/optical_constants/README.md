# Optical-constant data

`Al_Rakic1995_um.txt` is the first literature-derived aluminium baseline.
Source/license/version details are in `third_party/refractiveindex-info/README.md`.

- Format: three whitespace-separated columns, `wavelength_um n k`; `#` comments.
- Wavelength: vacuum wavelength in **micrometers**; n/k are absolute dimensionless values.
- Support: 0.41328-2.4797 um (413.28-2479.7 nm), including 905 and 1550 nm.
- Samples: 26 original contiguous rows, no interpolation or new samples inserted in this file.
- Model: intrinsic aluminium constants reconstructed from film optical/loss data, not a measured BRDF.

Call `OpticalConstantsTableReader::FromFile` with
`OpticalConstantsWavelengthUnit::Micrometers`; the reader converts wavelengths
to nm exactly once, using factor 1000. There is no unit guessing from numbers
or comments. Provide bibliographic metadata explicitly; comment headers are
retained in the file but not interpreted as runtime metadata.

Roughness is **not** in this dataset. Test alphaX=0.2, alphaY=0.3 are assumptions,
not values for real polished/roughened aluminium. No oxidation, coatings, alloy
composition, temperature variation or measured radar intensity is encoded.
Files are registered as VS data resources; they are not implicitly copied to
the executable directory. The caller must supply their actual filesystem path.
