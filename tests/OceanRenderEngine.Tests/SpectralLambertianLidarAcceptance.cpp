#include "reflection/lidar/SpectralLambertianLidarScattering.h" // Must be self-contained.
#include "reflection/lidar/LambertianLidarScattering.h"
#include "core/ray/HittableList.h"
#include "core/sampling/IndependentSampler.h"
#include "lidar/simulation/LidarIntegrator.h"
#include "lidar/simulation/LidarWaveform.h"
#include "geometry/primitives/Sphere.h"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <type_traits>
#include <utility>

namespace
{
void Check(const char* name, bool condition)
{
    if (!condition) { std::cerr << "[FAIL] " << name << '\n'; throw std::runtime_error(name); }
    std::cout << "[PASS] " << name << '\n';
}
bool Near(double a, double b)
{
    return std::isfinite(a) && std::isfinite(b) &&
        std::abs(a - b) <= 1e-12 + 1e-10 * std::max(std::abs(a), std::abs(b));
}
bool EnergyNear(double a, double b)
{
    return std::isfinite(a) && std::isfinite(b) &&
        std::abs(a - b) <= 1e-22 + 1e-6 * std::max(std::abs(a), std::abs(b));
}
template<class Exception, class Function> bool Rejects(Function function)
{
    try { function(); } catch (const Exception&) { return true; } return false;
}
LidarPulseResult Pulse(const LidarScatteringModel& model, float wavelength, std::size_t count = 1)
{
    HittableList world(std::make_shared<Sphere>(Vector3f(0, -1, 0), 1, nullptr));
    const Vector3f sensor(0, 15, 0), down(0, -1, 0);
    const LaserEmitter emitter(sensor, down, wavelength, 1e-3f, 0);
    const LidarReceiver receiver(sensor, down, .25f, .01f, .8f);
    const LidarIntegrator integrator(world, emitter, receiver, count, model);
    IndependentSampler sampler(42);
    return integrator.SimulatePulse(sampler);
}
}

void RunSpectralLambertianLidarAcceptanceTests()
{
    static_assert(std::is_base_of_v<LidarScatteringModel, SpectralLambertianLidarScattering>);
    const SpectralReflectanceCurve curve({{400, .2}, {600, .6}, {1550, .8}});
    const SpectralLambertianLidarScattering model(curve);
    const LidarScatteringModel& interface = model;
    const Vector3f up(0, 1, 0);
    HitRecord hit{}; hit.normal = hit.geometricNormal = up;
    const double pi = std::numbers::pi_v<double>;
    auto eval = [&](float wavelength) { return interface.Evaluate(hit, up, up, wavelength); };
    Check("spectral Lambert lower endpoint rho over pi", Near(eval(400), .2 / pi));
    Check("spectral Lambert interior knot rho over pi", Near(eval(600), .6 / pi));
    Check("spectral Lambert upper endpoint rho over pi", Near(eval(1550), .8 / pi));
    Check("spectral Lambert interpolated rho over pi", Near(eval(500), .4 / pi));
    Check("spectral Lambert wavelength changes BRDF", eval(905) > eval(532));
    Check("spectral Lambert reflectance accessor not BRDF", Near(model.ReflectanceAtWavelength(500), .4));
    const Vector3f wi(.2f, 1, .3f), wo(-.4f, 1, .1f);
    Check("spectral Lambert no extra angle cosine", Near(interface.Evaluate(hit, wi, wo, 500), eval(500)));
    Check("spectral Lambert reciprocity", Near(interface.Evaluate(hit, wi, wo, 500), interface.Evaluate(hit, wo, wi, 500)));
    Check("spectral Lambert owns curve copy", model.ReflectanceCurve().Samples().data() != curve.Samples().data());
    Check("spectral Lambert rejects zero wavelength", Rejects<std::invalid_argument>([&] { eval(0); }));
    Check("spectral Lambert rejects negative wavelength", Rejects<std::invalid_argument>([&] { eval(-1); }));
    Check("spectral Lambert rejects NaN wavelength", Rejects<std::invalid_argument>([&] { eval(std::numeric_limits<float>::quiet_NaN()); }));
    Check("spectral Lambert rejects infinite wavelength", Rejects<std::invalid_argument>([&] { eval(std::numeric_limits<float>::infinity()); }));
    Check("spectral Lambert rejects lower out of band", Rejects<std::out_of_range>([&] { eval(399); }));
    Check("spectral Lambert rejects upper out of band", Rejects<std::out_of_range>([&] { eval(1551); }));
    SpectralReflectanceCurve movable = curve;
    const SpectralLambertianLidarScattering moved(std::move(movable));
    Check("spectral Lambert accepts moved curve", Near(moved.ReflectanceAtWavelength(500), .4));
    // A moved-from vector's contents are unspecified; test either legal state
    // with one deterministic check, rather than relying on it always being empty.
    Check("spectral Lambert handles moved-from input explicitly",
        movable.SampleCount() < 2 ? Rejects<std::invalid_argument>([&] {
            SpectralLambertianLidarScattering bad(movable);
        }) : Near(SpectralLambertianLidarScattering(movable).ReflectanceAtWavelength(500), .4));
    const SpectralLambertianLidarScattering constant(SpectralReflectanceCurve({{400, .5}, {1550, .5}}));
    const LambertianLidarScattering fixed(.5);
    Check("spectral Lambert constant curve matches fixed Lambert", Near(constant.Evaluate(hit, up, up, 905), fixed.Evaluate(hit, up, up, 905)));

    const auto path = std::filesystem::path(__FILE__).parent_path() / "fixtures" / "spectra" / "synthetic_reflectance.txt";
    const SpectralLambertianLidarScattering measured(SpectralReflectanceCurve::FromAsterFile(path));
    Check("spectral Lambert ASTER file units reach BRDF", Near(measured.Evaluate(hit, up, up, 532), .4 / pi));
    Check("spectral Lambert retains measurement metadata", measured.ReflectanceCurve().Metadata().sampleNumber == "TEST-001");
    const auto green = Pulse(measured, 532), infrared = Pulse(measured, 1550);
    Check("spectral Lambert integrator produces surface returns", green.surfaceHitCount == 1 && green.returns.size() == 1 && infrared.returns.size() == 1);
    // E_rx = E_emit * (rho/pi) * cosReturn * (A/R^2) * efficiency.
    // At normal incidence cosReturn=1; no extra cosIncident is introduced.
    const double expected = double(1e-3f) * (.4 / pi) * double(.01f) / 225.0 * double(.8f);
    Check("spectral Lambert analytic received energy", EnergyNear(green.TotalReceivedEnergyJ(), expected));
    Check("spectral Lambert wavelength energy ratio follows rho", Near(infrared.TotalReceivedEnergyJ() / green.TotalReceivedEnergyJ(), 2));
    Check("spectral Lambert wavelength preserved in return", green.returns.at(0).wavelengthNm == 532 && infrared.returns.at(0).wavelengthNm == 1550);
    Check("spectral Lambert geometry time unchanged by wavelength", green.returns.at(0).arrivalTimeSeconds == infrared.returns.at(0).arrivalTimeSeconds && Near(green.returns.at(0).arrivalTimeSeconds, 30.0 / 299792458.0));
    const auto many = Pulse(measured, 532, 16);
    Check("spectral Lambert MC sample count normalization", many.returns.size() == 16 && EnergyNear(many.TotalReceivedEnergyJ(), green.TotalReceivedEnergyJ()));
    const auto repeat = Pulse(measured, 532, 16);
    Check("spectral Lambert seeded pulse repeat", repeat.TotalReceivedEnergyJ() == many.TotalReceivedEnergyJ());
    LidarWaveform waveform({0, 1e-9, 256});
    Check("spectral Lambert waveform preserves total energy", EnergyNear(waveform.AccumulatePulse(many), many.TotalReceivedEnergyJ()));
    Check("spectral Lambert constant curve preserves existing chain", EnergyNear(Pulse(constant, 905).TotalReceivedEnergyJ(), Pulse(fixed, 905).TotalReceivedEnergyJ()));
    const SpectralLambertianLidarScattering black(SpectralReflectanceCurve({{400, 0}, {1550, 0}}));
    const auto dark = Pulse(black, 532);
    Check("spectral Lambert black surface hit without return", dark.surfaceHitCount == 1 && dark.returns.empty());
    Check("spectral Lambert out of band propagates through integrator", Rejects<std::out_of_range>([&] { Pulse(measured, 1600); }));
}
