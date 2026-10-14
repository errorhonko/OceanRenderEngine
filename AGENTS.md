# OceanRenderEngine Project Guidance

## Project goal

- Build a C++20 CPU renderer as an autumn-recruitment portfolio project.
- Evolve the renderer toward physically based ocean-surface and laser-LiDAR return simulation.
- Favor physically correct, testable light-transport models over purely visual effects.

## Primary technical reference

- Use *Physically Based Rendering: From Theory to Implementation, Fourth Edition* as the primary reference for renderer architecture, radiometry, sampling, BSDFs, lights, and integrators: https://pbr-book.org/4ed/contents
- Use the official `mmp/pbrt-v4` repository when exact implementation details are needed: https://github.com/mmp/pbrt-v4
- Prefer the official book and repository over secondary tutorials.
- State which PBRT concept or source section is being adapted when proposing a renderer design.
- Do not copy PBRT mechanically. Map PBRT abstractions onto this smaller engine and clearly identify deliberate simplifications.
- For Monte Carlo code, explain the estimator, sampling distribution, PDF measure, cosine term, and conditions required for an unbiased result before proposing implementation details.

## Collaboration style

- Teach first. The developer wants to write the implementation personally.
- Do not edit project source files unless the developer explicitly asks for an edit.
- Provide diagnosis, relevant formulas, PBRT-to-project type mappings, and small before/after snippets.
- Work in one small, testable checkpoint at a time; wait for the developer's implementation and then review it.
- Distinguish compile-only fixes from physically correct fixes.
- After each light-transport milestone, propose a deterministic acceptance test before adding more features.

## Project conventions

- Use C++20 and the existing PBRT-inspired naming style.
- Shading-local coordinates use positive Z as the surface normal.
- Keep random sampling reproducible through explicit sampler seeds.
- Treat `Spectrum`, BSDF values, PDFs, and path throughput as physical quantities; do not silently replace them with display-space RGB operations.
- Keep `SimplePathIntegrator` pedagogical: implement direct-light sampling and BSDF sampling first. Defer MIS, Russian roulette, and regularization to the later full `PathIntegrator`.

## Module layout and lookup

- Engine source now lives under `src/`; see `experiments/ProjectStructure.md` for the exact migration map.
- Include headers by module-qualified paths relative to `src/`, e.g. `reflection/bxdfs/ConductorBxDF.h`.
- Keep main project filters aligned with physical directories; place related .h/.cpp together.
- Shared geometry belongs to `geometry/`, not ocean; digital estimation belongs to `lidar/signal_processing/`, not analog receiver.
- Before adding a model, inspect existing BxDF/Fresnel/microfacet implementations and adapters for reuse.
- `reflection/lidar/` contains BRDF adapters, not receiver processing. RGB and laser-wavelength inputs remain distinct.
- Preserve existing `gpu/`, tests/fixtures, experiments and third-party data; do not register inactive .cpp files as compilation units accidentally.
- The checkpoint below is historical; use current acceptance logs and experiment documents for current capability/counts.

## Verified build baseline

- Pin WindowsTargetPlatformVersion to `10.0.26100.0` in all six CPU/GPU/experiment projects; do not use floating `10.0` for new projects.
- Verified 2026-10-14: actual x64 Debug/Release solution rebuilds with default manifest generation; CPU acceptance has 2300 passes per configuration.
- Hardware GPU acceptance passes on AMD Radeon RX 7900 XTX for affine compute and DXR 1.1 triangle/persistent-scene queries. This is not a complete GPU renderer.
- When testing, run the executable emitted in the build log, not an assumed old output path. Solution builds put CPU executables in repository `x64/<configuration>/`; standalone project outputs can differ.
- See `experiments/Sdk26100Verification.md` for scope, logs and the prior SDK 28000 missing-DLL issue.

## Historical renderer checkpoint

- `SimplePathIntegrator::Li()` now supports environment emission, point and distant lights, sphere and triangle diffuse area lights, next-event estimation, BSDF continuation sampling, emissive-surface hits, and the no-double-counting rule for sampled direct lighting.
- Surface interactions distinguish geometric and shading normals. Secondary and shadow rays use the geometric normal for robust origin offsets, while BSDF evaluation uses the shading normal.
- The deterministic acceptance suite currently contains 38 passing checks, including direct-light visibility, inverse-square falloff, environment sampling, area-light emission/sampling, triangle differential geometry, normal separation, and ray-offset behavior.
- The next milestone is a separate PBRT-inspired full `PathIntegrator`: add light-direction PDFs and multiple importance sampling first, then add Russian roulette and the remaining production-path controls. Keep `SimplePathIntegrator` unchanged as the pedagogical reference implementation.
