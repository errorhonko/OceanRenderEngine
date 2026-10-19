# LiDAR material data and response configuration

2026-10-19. Small configuration refactor, not a new material database or BRDF.
PBRT reference: Material::Create and Material/GetBSDF separation (4th ed., 10.5).
HELIOS reference: https://heliospp.readthedocs.io/en/alpha-dev/intensity_fwf.html

## Configuration vs runtime

`LidarMaterialConfig` contains two explicit choices:

| Loaded source data | Compatible response |
| --- | --- |
| ConstantLidarReflectance (rho in [0,1]) | Lambert or existing HELIOS empirical |
| SpectralReflectanceCurve | Lambert or existing HELIOS empirical |
| SpectralOpticalConstants (n/k, not rho) | Existing rough GGX conductor |

`LidarScatteringFactory::Create` validates compatibility and delegates to the
existing model constructors. No new Fresnel, microfacet, angular, energy or PDF
formula is introduced. `alphaX/alphaY` are distribution parameters, not artist
roughness; n/k data does not determine them. No automatic defaults fill missing
optical constants, roughness or out-of-band measurements.

```cpp
auto curve = SpectralReflectanceCurve::FromAsterFile(actualSpectrumPath);
auto lambert = LidarScatteringFactory::Create({curve, LambertLidarResponse{}});
auto empirical = LidarScatteringFactory::Create({curve,
    HeliosEmpiricalAngularResponse({.2,.2,.2}, {.6,.6,.6}, 4)});
// Kd/Ks/Ns above are illustrative assumptions, not NIR calibration data.

auto nk = OpticalConstantsTableReader::FromFile(actualOpticalConstantsPath,
    OpticalConstantsWavelengthUnit::Micrometers, opticalMetadata);
auto metal = LidarScatteringFactory::Create({nk,
    GGXConductorLidarResponse{1.0, .2f, .3f}});
// The two alphas above are explicit assumptions, not measured Al roughness.
```

The config owns loaded data; existing readers retain units/provenance and the
models retain their existing owned snapshots. Loading is outside the ray loop.
A serialized catalog, sample selection, data downloads and license manifest
are deferred. No real ECOSTRESS production spectra are added by this refactor.

## Actual call chain

```
MTL percent / spectrum name -> ObjMaterialBinding parses source data
                              + options.lidarAngularModel / Kd / Ks / Ns
                              -> LidarMaterialConfig -> LidarScatteringFactory
                              -> concrete existing model -> SurfaceMaterial
                              -> BindFaces -> mesh hit -> HitRecord
                              -> SurfaceLidarScattering
                              -> EvaluateProjectedResponse
                              -> LidarReturnEstimator -> waveform -> receiver

n/k reader + GGX parameters -> same factory -> options.overrides -> same binding
```

Defaults, percent-over-spectrum precedence, authoritative full overrides,
explicit fallbacks, path validation, missing-file errors and transparency policy
remain unchanged. No new MTL field is invented for n/k. Lambert spectral models
remain shared within one converted table. HELIOS responses remain per-entry,
since the same spectrum can have different Kd/Ks/Ns.

HELIOS is a monostatic empirical projected response, not a general pure BRDF.
Its existing 1/pi convention and no-double-cosine rule are unchanged; it does
not reproduce upstream HELIOS raw intensity scale. Optical constants cannot be
combined with Lambert/HELIOS, nor reflectance with the GGX conductor adapter.

Acceptance: direct-vs-factory equivalence across spectra, wavelengths and angles;
existing data provenance and errors; incompatible combinations; scalar validation;
per-entry empirical behavior and Lambert sharing; existing OBJ override bridge.
Full pre-existing OBJ/HELIOS/energy-chain tests must also pass after the change.
