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

## Current development checkpoint

- `SimplePathIntegrator::Li()` now supports environment emission, point and distant lights, sphere and triangle diffuse area lights, next-event estimation, BSDF continuation sampling, emissive-surface hits, and the no-double-counting rule for sampled direct lighting.
- Surface interactions distinguish geometric and shading normals. Secondary and shadow rays use the geometric normal for robust origin offsets, while BSDF evaluation uses the shading normal.
- The deterministic acceptance suite currently contains 38 passing checks, including direct-light visibility, inverse-square falloff, environment sampling, area-light emission/sampling, triangle differential geometry, normal separation, and ray-offset behavior.
- The next milestone is a separate PBRT-inspired full `PathIntegrator`: add light-direction PDFs and multiple importance sampling first, then add Russian roulette and the remaining production-path controls. Keep `SimplePathIntegrator` unchanged as the pedagogical reference implementation.
