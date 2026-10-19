#include "core/spectrum/SpectralOpticalConstants.h" // Must compile as the first include.
#include "reflection/fresnel/FresnelUtils.h"
#include <iostream>
#include <limits>
#include <type_traits>

namespace {
void Check(const char* name, bool condition)
{
    if (!condition) { std::cerr << "[FAIL] " << name << '\n'; throw std::runtime_error(name); }
    std::cout << "[PASS] " << name << '\n';
}
bool Near(double a, double b) { return std::abs(a - b) <= 1e-12; }
template<class Exception, class Function> bool Rejects(Function function)
{
    try { function(); } catch (const Exception&) { return true; } return false;
}
}

void RunSpectralOpticalConstantsAcceptanceTests()
{
    static_assert(!std::is_default_constructible_v<SpectralOpticalConstants>);
    static_assert(std::is_same_v<decltype(std::declval<const SpectralOpticalConstants&>().Evaluate(905.0)), OpticalConstants>);
    static_assert(std::is_same_v<decltype(std::declval<const SpectralOpticalConstants&>().Samples()), const std::vector<OpticalConstantsSample>&>);
    // Synthetic data only: these numbers are NOT measured metal optical constants.
    std::vector<OpticalConstantsSample> input{{400, 0.2, 2.0}, {600, 0.6, 4.0}, {1600, 1.6, 8.0}};
    SpectralOpticalConstantsMetadata metadata{"synthetic", "acceptance test", "not measured data"};
    const SpectralOpticalConstants table(input, metadata);
    input[0] = {1, 9, 9}; metadata.name = "changed";
    Check("optical constants own paired samples", table.SampleCount() == 3 && table.Samples()[0].wavelengthNm == 400);
    Check("optical constants retain provenance independently", table.Metadata().name == "synthetic" && table.Metadata().source == "acceptance test" && table.Metadata().sampleConditions == "not measured data");
    Check("optical constants retain closed wavelength support", table.MinimumWavelengthNm() == 400 && table.MaximumWavelengthNm() == 1600);
    const auto first = table.Evaluate(400), last = table.Evaluate(1600), knot = table.Evaluate(600);
    Check("optical constants first endpoint is exact", first.n == 0.2 && first.k == 2.0);
    Check("optical constants last endpoint is exact", last.n == 1.6 && last.k == 8.0);
    Check("optical constants internal knot is exact", knot.n == 0.6 && knot.k == 4.0);
    const auto middle = table.Evaluate(500);
    Check("optical constants interpolate n and k together", Near(middle.n, 0.4) && Near(middle.k, 3.0));
    const auto nir = table.Evaluate(905);
    Check("optical constants interpolate nonuniform near infrared knots", Near(nir.n, 0.905) && Near(nir.k, 5.22));
    const auto swir = table.Evaluate(1550);
    Check("optical constants support 1550 nm without visible-only limits", Near(swir.n, 1.55) && Near(swir.k, 7.8));
    Check("optical constants n and k may exceed one", last.n > 1.0 && last.k > 1.0);
    Check("optical constants support includes endpoints", table.Contains(400) && table.Contains(1600) && table.Contains(905));
    Check("optical constants do not extrapolate", !table.Contains(399) && !table.Contains(1601));
    Check("optical constants reject lower out of range query", Rejects<std::out_of_range>([&] { (void)table.Evaluate(399); }));
    Check("optical constants reject upper out of range query", Rejects<std::out_of_range>([&] { (void)table.Evaluate(1601); }));
    Check("optical constants reject a query immediately below support", Rejects<std::out_of_range>([&] { (void)table.Evaluate(std::nextafter(400.0, 0.0)); }));
    Check("optical constants reject a query immediately above support", Rejects<std::out_of_range>([&] { (void)table.Evaluate(std::nextafter(1600.0, 2000.0)); }));
    const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
    for (double wavelength : {0.0, -1.0, nan, inf, -inf})
    {
        Check("optical constants reject invalid query wavelength", Rejects<std::invalid_argument>([&] { (void)table.Evaluate(wavelength); }));
        Check("optical constants Contains safely rejects invalid query", !table.Contains(wavelength));
    }
    Check("optical constants reject empty tables", Rejects<std::invalid_argument>([] { (void)SpectralOpticalConstants({}); }));
    Check("optical constants reject a single sample", Rejects<std::invalid_argument>([] { (void)SpectralOpticalConstants({{500, 1, 0}}); }));
    Check("optical constants reject duplicate wavelengths", Rejects<std::invalid_argument>([] { (void)SpectralOpticalConstants({{500, 1, 0}, {500, 2, 1}}); }));
    Check("optical constants reject descending wavelengths", Rejects<std::invalid_argument>([] { (void)SpectralOpticalConstants({{600, 1, 0}, {500, 2, 1}}); }));
    for (double wavelength : {0.0, -1.0, nan, inf, -inf})
        Check("optical constants reject invalid sample wavelength", Rejects<std::invalid_argument>([&] { (void)SpectralOpticalConstants({{wavelength, 1, 0}, {1600, 2, 1}}); }));
    for (double value : {-1.0, nan, inf, -inf})
    {
        Check("optical constants reject negative or nonfinite n", Rejects<std::invalid_argument>([&] { (void)SpectralOpticalConstants({{400, value, 1}, {1600, 2, 1}}); }));
        Check("optical constants reject negative or nonfinite k", Rejects<std::invalid_argument>([&] { (void)SpectralOpticalConstants({{400, 1, value}, {1600, 2, 1}}); }));
    }
    Check("optical constants reject jointly zero n and k", Rejects<std::invalid_argument>([] { (void)SpectralOpticalConstants({{400, 0, 0}, {1600, 2, 1}}); }));
    const SpectralOpticalConstants dielectric({{400, 1.2, 0}, {1600, 1.8, 0}});
    Check("optical constants allow nonabsorbing dielectric limit", Near(dielectric.Evaluate(1000).n, 1.5) && dielectric.Evaluate(1000).k == 0);
    const SpectralOpticalConstants purelyImaginary({{400, 0, 1}, {1600, 0, 3}});
    Check("optical constants allow zero n with positive k", purelyImaginary.Evaluate(1000).n == 0 && purelyImaginary.Evaluate(1000).k == 2);
    const double huge = std::numeric_limits<double>::max();
    const SpectralOpticalConstants large({{400, huge / 2, huge / 2}, {1600, huge, huge}});
    const auto largeMiddle = large.Evaluate(1000);
    Check("optical constants interpolation avoids overflowing endpoint sums", std::isfinite(largeMiddle.n) && largeMiddle.n > huge / 2 && largeMiddle.n == largeMiddle.k);
    const SpectralOpticalConstants largeDomain({{1, 1, 1}, {huge, 3, 5}});
    const auto wide = largeDomain.Evaluate(huge / 2);
    Check("optical constants support finite large wavelength intervals", Near(wide.n, 2) && Near(wide.k, 3));
    const double tiny = std::numeric_limits<double>::denorm_min();
    const SpectralOpticalConstants underflow({{400, tiny, 0}, {1600, 0, tiny}});
    Check("optical constants report jointly zero interpolation underflow", Rejects<std::underflow_error>([&] { (void)underflow.Evaluate(1000); }));
    const auto copy = table;
    Check("optical constants copying preserves data and provenance", copy.Evaluate(905).n == nir.n && copy.Metadata().name == table.Metadata().name);
    SpectralOpticalConstants movable = table;
    const SpectralOpticalConstants moved(std::move(movable));
    Check("optical constants moving preserves destination", moved.Evaluate(905).k == nir.k && moved.Metadata().name == "synthetic");
    // The moved-from vector may be empty or retain its valid table. Support both.
    Check("optical constants moved-from object remains safely queryable or guarded",
        movable.SampleCount() >= 2 ? Near(movable.Evaluate(905).n, nir.n) :
        (!movable.Contains(905) && Rejects<std::logic_error>([&] { (void)movable.Evaluate(905); }) &&
         Rejects<std::logic_error>([&] { (void)movable.MinimumWavelengthNm(); }) &&
         Rejects<std::logic_error>([&] { (void)movable.MaximumWavelengthNm(); })));
    // Only a scalar Fresnel smoke test: the LiDAR adapter has not been modified.
    const double analytic = ((nir.n - 1) * (nir.n - 1) + nir.k * nir.k) /
        ((nir.n + 1) * (nir.n + 1) + nir.k * nir.k);
    Check("optical constants feed existing normal-incidence conductor Fresnel", Near(FrConductor(1.0, 1.0, nir.n, nir.k), analytic));
    Check("optical constants dielectric limit matches existing Fresnel", Near(FrConductor(0.7, 1.0, dielectric.Evaluate(1000).n, 0.0), FrDielectric(0.7, 1.0, 1.5)));
}
