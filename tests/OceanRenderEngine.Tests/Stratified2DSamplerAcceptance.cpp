#include "core/sampling/Stratified2DSampler.h"
#include "lidar/emitter/beam/GaussianLidarBeamProfile.h"
#include "lidar/simulation/LidarIntegrator.h"
#include "reflection/lidar/LambertianLidarScattering.h"
#include "core/ray/HittableList.h"
#include "geometry/primitives/Sphere.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace
{
void Check(const char* name, bool ok)
{
    if (!ok) throw std::runtime_error(std::string("[FAIL] ") + name);
    std::cout << "[PASS] " << name << '\n';
}

template <class Exception, class Function>
bool Throws(Function&& function)
{
    try { function(); }
    catch (const Exception&) { return true; }
    return false;
}

bool Inside(float u, std::size_t cell, std::size_t count)
{
    return std::isfinite(u) && double(u) >= double(cell) / double(count) &&
        double(u) < double(cell + 1) / double(count);
}

void CheckGrid(std::size_t nx, std::size_t ny)
{
    Stratified2DSampler sampler(nx, ny, 42);
    std::vector<int> visits(nx * ny, 0);
    bool supported = true, ordered = true, progress = true;
    for (std::size_t i = 0; i < nx * ny; ++i)
    {
        const auto u = sampler.Get2D();
        supported = supported && u.x >= 0 && u.x < 1 && u.y >= 0 && u.y < 1;
        ordered = ordered && Inside(u.x, i % nx, nx) && Inside(u.y, i / nx, ny);
        const auto x = std::size_t(double(u.x) * double(nx));
        const auto y = std::size_t(double(u.y) * double(ny));
        if (x < nx && y < ny) ++visits[y * nx + x]; else supported = false;
        progress = progress && sampler.SamplesGenerated() == i + 1 &&
            sampler.RemainingSamples() == nx * ny - i - 1;
    }
    bool once = true;
    for (const int count : visits) once = once && count == 1;
    const std::string label = "Stratified grid " + std::to_string(nx) + "x" + std::to_string(ny);
    Check((label + " samples stay in half open unit square").c_str(), supported);
    Check((label + " every cell visited once in row major order").c_str(), ordered && once);
    Check((label + " progress and exhaustion match batch size").c_str(), progress && sampler.Exhausted());
}
}

void RunStratified2DSamplerAcceptanceTests()
{
    static_assert(std::is_base_of_v<Sampler, Stratified2DSampler>);
    CheckGrid(8, 8);
    CheckGrid(7, 5);
    CheckGrid(1, 1);
    CheckGrid(1, 16);
    CheckGrid(16, 1);
    CheckGrid(1, 65536); // Float boundaries near 1, with a complete fine grid.

    Check("Stratified zero X rejected", Throws<std::invalid_argument>([] { Stratified2DSampler s(0, 8); }));
    Check("Stratified zero Y rejected", Throws<std::invalid_argument>([] { Stratified2DSampler s(8, 0); }));
    Check("Stratified batch multiplication overflow rejected", Throws<std::length_error>([] {
        Stratified2DSampler s(std::numeric_limits<std::size_t>::max(), 2);
    }));
    Check("Stratified X beyond float grid precision rejected", Throws<std::invalid_argument>([] {
        Stratified2DSampler s(Stratified2DSampler::MaxStrataPerAxis + 1, 1);
    }));
    Check("Stratified Y beyond float grid precision rejected", Throws<std::invalid_argument>([] {
        Stratified2DSampler s(1, Stratified2DSampler::MaxStrataPerAxis + 1);
    }));
    Stratified2DSampler sampler(8, 8, 42), replay(8, 8, 42), other(8, 8, 43);
    Check("Stratified getters report initial batch configuration", sampler.XStrata() == 8 &&
        sampler.YStrata() == 8 && sampler.SamplesPerBatch() == 64 &&
        sampler.SamplesGenerated() == 0 && sampler.RemainingSamples() == 64 && !sampler.Exhausted());
    Sampler& base = sampler;
    bool identical = true, different = false, jittered = false;
    for (std::size_t i = 0; i < 64; ++i)
    {
        const auto a = base.Get2D(), b = replay.Get2D(), c = other.Get2D();
        identical = identical && a.x == b.x && a.y == b.y;
        different = different || a.x != c.x || a.y != c.y;
        jittered = jittered || double(a.x) != (double(i % 8) + .5) / 8;
    }
    Check("Stratified polymorphic fixed seed replay exact", identical);
    Check("Stratified another seed changes within cell jitter", different);
    Check("Stratified samples are jittered rather than fixed centers", jittered);
    Check("Stratified exhausted batch throws without automatic wrapping", Throws<std::out_of_range>([&] { base.Get2D(); }));
    Check("Stratified repeated exhaustion leaves counters unchanged",
        Throws<std::out_of_range>([&] { base.Get2D(); }) && sampler.SamplesGenerated() == 64 && sampler.RemainingSamples() == 0);

    Stratified2DSampler oneD(2, 3, 42), oneDReplay(2, 3, 42);
    bool oneDValid = true, oneDReplayExact = true;
    for (int i = 0; i < 1000; ++i)
    {
        const float a = oneD.Get1D(), b = oneDReplay.Get1D();
        oneDValid = oneDValid && std::isfinite(a) && a >= 0 && a < 1;
        oneDReplayExact = oneDReplayExact && a == b;
    }
    Check("Stratified Get1D stays in half open unit interval", oneDValid);
    Check("Stratified Get1D replays with identical call sequence", oneDReplayExact);
    const auto first = oneD.Get2D(), firstReplay = oneDReplay.Get2D();
    Check("Stratified Get1D does not consume 2D cells", oneD.SamplesGenerated() == 1 &&
        Inside(first.x, 0, 2) && Inside(first.y, 0, 3));
    Check("Stratified mixed 1D and 2D calls replay exactly", first.x == firstReplay.x && first.y == firstReplay.y);
    sampler.Get1D();
    Check("Stratified Get1D remains independent after 2D exhaustion", sampler.Exhausted() && sampler.SamplesGenerated() == 64);

    // Boundary stratum on the finest supported axis, without allocating or drawing a whole huge batch.
    Stratified2DSampler finest(Stratified2DSampler::MaxStrataPerAxis, 1, 42);
    bool fineSupported = true;
    for (std::size_t i = 0; i < 64; ++i)
    {
        const auto u = finest.Get2D();
        fineSupported = fineSupported && Inside(u.x, i, finest.XStrata()) && Inside(u.y, 0, 1);
    }
    Check("Stratified finest supported axis retains cell membership", fineSupported);

    // Analytic integral over u-space, averaging complete batches over independent seeds.
    double meanX = 0, meanY = 0, meanProduct = 0;
    constexpr int batches = 512;
    for (int seed = 0; seed < batches; ++seed)
    {
        Stratified2DSampler batch(8, 8, std::uint32_t(seed));
        for (int i = 0; i < 64; ++i)
        {
            const auto u = batch.Get2D();
            meanX += double(u.x) / (batches * 64);
            meanY += double(u.y) / (batches * 64);
            meanProduct += double(u.x) * double(u.y) / (batches * 64);
        }
    }
    Check("Stratified complete batch mean matches unit square analytic integral", std::abs(meanX - .5) < .002 && std::abs(meanY - .5) < .002);
    Check("Stratified complete batch product matches analytic integral", std::abs(meanProduct - .25) < .002);

    // Real Gaussian emitter and existing integrator, without changing the scan driver's default.
    const auto beam = std::make_shared<GaussianLidarBeamProfile>(.001, .003);
    const LaserEmitter emitter({0, 15, 0}, {0, -1, 0}, 532, 1e-3f, beam);
    Stratified2DSampler energySampler(8, 8, 42);
    double totalEnergy = 0;
    bool validEmission = true;
    for (int i = 0; i < 64; ++i)
    {
        const auto e = emitter.SampleRay(energySampler.Get2D());
        validEmission = validEmission && !e.deltaDirection && std::isfinite(e.directionPdf) && e.directionPdf > 0;
        totalEnergy += e.energyWeightJ / 64;
    }
    Check("Stratified batch plugs into actual Gaussian emission", validEmission && energySampler.Exhausted());
    Check("Stratified Gaussian energy weights preserve pulse energy with one 1 over N", std::abs(totalEnergy / emitter.PulseEnergyJ() - 1) < 1e-12);
    HittableList world(std::make_shared<Sphere>(Vector3f(0, -1, 0), 1, nullptr));
    const LidarReceiver receiver({0, 15, 0}, {0, -1, 0}, .25f, .01f, .8f);
    const LambertianLidarScattering scattering(.5);
    const LidarIntegrator integrator(world, emitter, receiver, 64, scattering);
    Stratified2DSampler pulseSampler(8, 8, 42), pulseReplay(8, 8, 42);
    const auto pulse = integrator.SimulatePulse(pulseSampler);
    const auto repeated = integrator.SimulatePulse(pulseReplay);
    Check("Stratified existing LiDAR integrator consumes complete batch", pulseSampler.Exhausted() &&
        pulse.emittedRayCount == 64 && pulse.surfaceHitCount == 64 && pulse.returns.size() == 64 &&
        std::isfinite(pulse.TotalReceivedEnergyJ()) && pulse.TotalReceivedEnergyJ() > 0);
    Check("Stratified real LiDAR pulse fixed seed energy replays exactly", pulse.TotalReceivedEnergyJ() == repeated.TotalReceivedEnergyJ());
}
