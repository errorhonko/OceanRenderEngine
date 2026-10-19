#include "core/spectrum/OpticalConstantsTableReader.h" // Self-contained first include.
#include "reflection/lidar/RoughConductorLidarScattering.h"
#include "reflection/lidar/SurfaceLidarScattering.h"
#include "scene/import/ObjSceneAsset.h"
#include "lidar/simulation/LidarIntegrator.h"
#include "lidar/simulation/LidarWaveform.h"
#include "core/sampling/IndependentSampler.h"
#include <iostream>
#include <numbers>

namespace {
void Check(const char* name, bool condition)
{
    if (!condition) { std::cerr << "[FAIL] " << name << '\n'; throw std::runtime_error(name); }
    std::cout << "[PASS] " << name << '\n';
}
bool Near(double a, double b, double relative = 1e-10)
{
    return std::isfinite(a) && std::isfinite(b) && std::abs(a-b) <= 1e-20 + relative*std::max(std::abs(a),std::abs(b));
}
template<class Exception, class Function> bool Rejects(Function function)
{
    try { function(); } catch (const Exception&) { return true; } return false;
}
SpectralOpticalConstants Parse(const std::string& text,
    OpticalConstantsWavelengthUnit unit = OpticalConstantsWavelengthUnit::Micrometers)
{
    std::istringstream input(text);
    return OpticalConstantsTableReader::FromStream(input, unit, {}, "synthetic-stream");
}
std::filesystem::path RepositoryRoot()
{
    const std::string_view utf8(__FILE__);
    return std::filesystem::path(std::u8string(utf8.begin(), utf8.end()))
        .parent_path().parent_path().parent_path();
}
// Independent values from the pinned source's two neighboring rows, not reader output.
OpticalConstants ReferenceAl(float wavelength)
{
    if (wavelength == 905)
    {
        const double t = (905.0 - 885.61) / (911.66 - 885.61);
        return {2.2802 + t*(1.9739-2.2802), 8.1134 + t*(8.3058-8.1134)};
    }
    const double t = (1550.0 - 1549.8) / (1771.2 - 1549.8);
    return {1.5782 + t*(1.9205-1.5782), 15.656 + t*(17.991-15.656)};
}
double NormalF(OpticalConstants nk)
{
    return ((nk.n-1)*(nk.n-1)+nk.k*nk.k) / ((nk.n+1)*(nk.n+1)+nk.k*nk.k);
}
}

void RunOpticalConstantsTableReaderAcceptanceTests()
{
    const auto nm = Parse("400 1.2 2\n1600 1.8 4\n", OpticalConstantsWavelengthUnit::Nanometers);
    const auto um = Parse("0.4 1.2 2\n1.6 1.8 4\n");
    Check("optical reader explicit nm and um agree", nm.MinimumWavelengthNm() == 400 && um.MinimumWavelengthNm() == 400 && Near(nm.Evaluate(905).n, um.Evaluate(905).n));
    Check("optical reader retains dimensionless n k without scaling", um.Samples()[0].n == 1.2 && um.Samples()[0].k == 2);
    Check("optical reader preserves k values above one", um.Evaluate(1600).k == 4);
    Check("optical reader does not guess wavelength units", Parse("0.4 1.2 2\n1.6 1.8 4\n", OpticalConstantsWavelengthUnit::Nanometers).MaximumWavelengthNm() == 1.6);
    Check("optical reader supports comments BOM and CRLF", Parse("\xEF\xBB\xBF# test\r\n\r\n  +0.4\t+1.2 2e0 # row\r\n1.6 1.8 4\r\n").SampleCount() == 2);
    Check("optical reader defaults provenance to source name", um.Metadata().source == "synthetic-stream");
    std::istringstream named("0.4 1 2\n1.6 2 3\n");
    const auto annotated = OpticalConstantsTableReader::FromStream(named, OpticalConstantsWavelengthUnit::Micrometers, {"sample", "doi:example", "known conditions"});
    Check("optical reader preserves supplied metadata", annotated.Metadata().name == "sample" && annotated.Metadata().source == "doi:example" && annotated.Metadata().sampleConditions == "known conditions");
    for (const std::string text : {"", "# comment only\n", "0.4 1 2\n", "0.4 1\n1.6 2 3\n", "0.4 1 2 9\n1.6 2 3\n", "0.4 1 2\ncorrupt row\n", "wavelength n k\n0.4 1 2\n1.6 2 3\n", "0.4,1,2\n1.6,2,3\n", "DATA:\n - type: tabulated nk\n", "0.4 1oops 2\n1.6 2 3\n", "0.4 +-0 2\n1.6 2 3\n", "0.4 nan 2\n1.6 2 3\n", "0.4 1 inf\n1.6 2 3\n", "inf 1 2\n1.6 2 3\n", "0.4 1e999 2\n1.6 2 3\n", "1e308 1 2\n1.6 2 3\n", "0 1 2\n1.6 2 3\n", "-0.4 1 2\n1.6 2 3\n", "0.4 -1 2\n1.6 2 3\n", "0.4 1 -2\n1.6 2 3\n", "0.4 0 0\n1.6 2 3\n", "0.4 1 2\n0.4 2 3\n", "1.6 1 2\n0.4 2 3\n"})
        Check("optical reader rejects malformed or physically invalid table", Rejects<std::invalid_argument>([&] { (void)Parse(text); }));
    Check("optical reader rejects unknown unit enum", Rejects<std::invalid_argument>([] { (void)Parse("0.4 1 2\n1.6 2 3\n", static_cast<OpticalConstantsWavelengthUnit>(99)); }));
    std::istringstream broken("0.4 1 2\n1.6 2 3\n"); broken.setstate(std::ios::badbit);
    Check("optical reader rejects unreadable stream", Rejects<std::runtime_error>([&] { (void)OpticalConstantsTableReader::FromStream(broken, OpticalConstantsWavelengthUnit::Micrometers); }));
    Check("optical reader rejects empty file path", Rejects<std::invalid_argument>([] { (void)OpticalConstantsTableReader::FromFile({}, OpticalConstantsWavelengthUnit::Micrometers); }));
    Check("optical reader rejects missing file", Rejects<std::runtime_error>([] { (void)OpticalConstantsTableReader::FromFile(RepositoryRoot()/"data/materials/optical_constants/does-not-exist.txt", OpticalConstantsWavelengthUnit::Micrometers); }));
    Check("optical reader supplies line context for bad numeric tokens", [] {
        try { (void)Parse("# comment\n0.4 garbage 2\n1.6 2 3\n"); }
        catch (const std::invalid_argument& error) { return std::string(error.what()).find("synthetic-stream:2:") != std::string::npos; }
        return false;
    }());
    const auto file = RepositoryRoot()/"data/materials/optical_constants/Al_Rakic1995_um.txt";
    const auto al = OpticalConstantsTableReader::FromFile(file, OpticalConstantsWavelengthUnit::Micrometers,
        {"Al Rakic 1995", "https://doi.org/10.1364/AO.34.004755", "Intrinsic constants reconstructed from evaporated-Al film data; roughness unspecified"});
    Check("optical reader real Al subset retains 26 source rows", al.SampleCount() == 26);
    Check("optical reader real Al subset retains finite support", Near(al.MinimumWavelengthNm(), 413.28) && Near(al.MaximumWavelengthNm(), 2479.7));
    Check("optical reader real Al exact source knot preserved", al.Samples()[13].n == 2.2802 && al.Samples()[13].k == 8.1134);
    Check("optical reader real Al provenance retained", al.Metadata().source == "https://doi.org/10.1364/AO.34.004755");
    Check("optical reader wrong nm declaration does not silently become um", Rejects<std::out_of_range>([&] { (void)OpticalConstantsTableReader::FromFile(file, OpticalConstantsWavelengthUnit::Nanometers).Evaluate(905); }));
    Check("optical reader real Al below supported band rejected", Rejects<std::out_of_range>([&] { (void)al.Evaluate(400); }));
    Check("optical reader real Al above supported band rejected", Rejects<std::out_of_range>([&] { (void)al.Evaluate(2500); }));
    const RoughConductorLidarScattering model(1, al, .2f, .3f); // ASSUMED roughness, not data-derived.
    HitRecord hit{}; hit.geometricNormal = hit.normal = {0,1,0}; hit.dpdu = {1,0,0};
    for (float wavelength : {905.f, 1550.f})
    {
        const auto expected = ReferenceAl(wavelength), actual = al.Evaluate(wavelength);
        Check("optical reader real Al n matches independent source interpolation", Near(actual.n, expected.n));
        Check("optical reader real Al k matches independent source interpolation", Near(actual.k, expected.k));
        Check("optical reader real Al Fresnel matches analytic normal incidence", Near(FrConductor(1.0, 1.0, actual.n, actual.k), NormalF(expected)));
        const double brdf = NormalF(expected)/(4*std::numbers::pi_v<double>*double(.2f)*double(.3f));
        Check("optical reader real Al BRDF matches analytic normal incidence", Near(model.Evaluate(hit, {0,1,0}, {0,1,0}, wavelength), brdf, 1e-6));
        const RoughConductorLidarScattering fixed(1, expected.n, expected.k, .2f, .3f, wavelength);
        bool matches = true;
        for (double angle : {0.,5.,15.,30.,60.,80.})
        {
            const double theta = angle*std::numbers::pi_v<double>/180;
            const Vector3f w(float(std::sin(theta)),float(std::cos(theta)),0);
            matches = matches && Near(model.Evaluate(hit,w,w,wavelength), fixed.Evaluate(hit,w,w,wavelength),1e-6);
        }
        Check("optical reader real Al angular sweep matches fixed reference", matches);
    }
    auto shared = std::make_shared<RoughConductorLidarScattering>(model);
    const auto fixture = RepositoryRoot()/"tests/OceanRenderEngine.Tests/fixtures/obj_materials";
    ObjMaterialBindingOptions options; options.spectraDirectory = fixture;
    options.overrides["low"] = std::make_shared<const SurfaceMaterial>(SurfaceMaterial{nullptr,shared});
    const auto asset = ObjSceneAsset::Read(fixture/"materials.obj",{},options);
    SceneBuilder builder; asset.AddToScene(builder); const auto scene = builder.Build();
    const SurfaceLidarScattering dispatch;
    for (float wavelength : {905.f,1550.f})
    {
        auto pulse = [&](std::size_t count) {
            const Vector3f sensor(-.5f,15,.5f), down(0,-1,0);
            const LaserEmitter emitter(sensor,down,wavelength,1e-3f,0);
            const LidarReceiver receiver(sensor,down,.25f,.01f,.8f);
            IndependentSampler sampler(42);
            return LidarIntegrator(*scene,emitter,receiver,count,dispatch).SimulatePulse(sampler);
        };
        const auto single = pulse(1), many = pulse(16);
        const double brdf = NormalF(ReferenceAl(wavelength))/(4*std::numbers::pi_v<double>*double(.2f)*double(.3f));
        const double energy = double(1e-3f)*brdf*double(.01f)/225*double(.8f);
        Check("optical reader real Al chain produces bound returns", single.returns.size() == 1 && many.returns.size() == 16);
        Check("optical reader real Al chain matches analytic received energy", Near(single.TotalReceivedEnergyJ(),energy,1e-6));
        Check("optical reader real Al chain retains MC normalization", Near(many.TotalReceivedEnergyJ(),single.TotalReceivedEnergyJ(),1e-6));
        Check("optical reader real Al chain retains time and wavelength", Near(single.returns.at(0).arrivalTimeSeconds,30.0/299792458.0) && single.returns.at(0).wavelengthNm == wavelength);
        LidarWaveform waveform({0,1e-9,256});
        Check("optical reader real Al chain conserves waveform energy", Near(waveform.AccumulatePulse(many), many.TotalReceivedEnergyJ(),1e-6));
    }
}
