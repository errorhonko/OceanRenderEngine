#include "core/spectrum/SpectrumInterpolation.h" // Self-contained first include.
#include "core/spectrum/SampledSpectrum.h"
#include "core/spectrum/SpectralReflectanceCurve.h"
#include "core/spectrum/SpectralOpticalConstants.h"
#include <array>
#include <iostream>
#include <limits>

float SpectrumInterpolationOtherTranslationUnit();
namespace {
void Check(const char* name, bool condition)
{
    if (!condition) { std::cerr << "[FAIL] " << name << '\n'; throw std::runtime_error(name); }
    std::cout << "[PASS] " << name << '\n';
}
bool Near(double a, double b, double tolerance = 1e-12) { return std::abs(a - b) <= tolerance; }
template<class Exception, class Function> bool Rejects(Function function)
{
    try { function(); } catch (const Exception&) { return true; } return false;
}
}
void RunSpectrumInterpolationAcceptanceTests()
{
    using SpectrumInterpolation::FindInterval;
    using SpectrumInterpolation::Weight;
    const std::array<double, 3> coordinates{400, 600, 1600};
    const auto span = std::span(coordinates);
    const auto identity = [](double value) { return value; };
    const auto first = FindInterval(span, 400, identity), knot = FindInterval(span, 600, identity), last = FindInterval(span, 1600, identity);
    Check("spectrum interpolation preserves first knot", first.lower == 0 && first.upper == 0 && first.weight == 0);
    Check("spectrum interpolation preserves internal knot", knot.lower == 1 && knot.upper == 1 && knot.weight == 0);
    Check("spectrum interpolation preserves last knot", last.lower == 2 && last.upper == 2 && last.weight == 0);
    const auto mid = FindInterval(span, 500, identity), nir = FindInterval(span, 905, identity);
    Check("spectrum interpolation finds first interior interval", mid.lower == 0 && mid.upper == 1 && mid.weight == 0.5);
    Check("spectrum interpolation finds nonuniform interior interval", nir.lower == 1 && nir.upper == 2 && Near(nir.weight, 0.305));
    Check("spectrum interpolation evaluates shared double values", Near(nir.Evaluate(0.6, 1.6), 0.905));
    Check("spectrum interpolation evaluates shared float values", Near(mid.Evaluate(2.0f, 4.0f), 3.0));
    Check("spectrum interpolation supports decreasing values", Near(mid.Evaluate(4.0, 2.0), 3.0));
    Check("spectrum interpolation rejects below support", Rejects<std::out_of_range>([&] { (void)FindInterval(span, 399, identity); }));
    Check("spectrum interpolation rejects above support", Rejects<std::out_of_range>([&] { (void)FindInterval(span, 1601, identity); }));
    Check("spectrum interpolation rejects empty span", Rejects<std::invalid_argument>([&] { (void)FindInterval(std::span<const double>{}, 500, identity); }));
    Check("spectrum interpolation rejects singleton span", Rejects<std::invalid_argument>([&] { (void)FindInterval(span.first<1>(), 400, identity); }));
    const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
    for (double value : {nan, inf, -inf})
        Check("spectrum interpolation rejects nonfinite query", Rejects<std::invalid_argument>([&] { (void)FindInterval(span, value, identity); }));
    Check("spectrum interpolation weight retains endpoints", Weight(400.0, 400.0, 600.0) == 0 && Weight(600.0, 400.0, 600.0) == 1);
    Check("spectrum interpolation weight rejects collapsed interval", Rejects<std::invalid_argument>([] { (void)Weight(400.0, 400.0, 400.0); }));
    Check("spectrum interpolation weight rejects reversed interval", Rejects<std::invalid_argument>([] { (void)Weight(500.0, 600.0, 400.0); }));
    Check("spectrum interpolation weight rejects extrapolation", Rejects<std::out_of_range>([] { (void)Weight(399.0, 400.0, 600.0); }));
    Check("spectrum interpolation weight rejects nonfinite interval", Rejects<std::invalid_argument>([&] { (void)Weight(500.0, 400.0, inf); }));
    const double huge = std::numeric_limits<double>::max();
    Check("spectrum interpolation reports unrepresentable interval width", Rejects<std::overflow_error>([&] { (void)Weight(0.0, -huge, huge); }));
    Check("spectrum interpolation legacy helper retains left extension", Interpolate(300, 400, 600, 2, 4) == 2);
    Check("spectrum interpolation legacy helper retains right extension", Interpolate(700, 400, 600, 2, 4) == 4);
    Check("spectrum interpolation legacy helper retains exact endpoints", Interpolate(400, 400, 600, 2, 4) == 2 && Interpolate(600, 400, 600, 2, 4) == 4);
    Check("spectrum interpolation legacy helper uses common calculation", Interpolate(500, 400, 600, 2, 4) == mid.Evaluate(2.0f, 4.0f));
    Check("spectrum interpolation legacy helper rejects degenerate interval", Rejects<std::invalid_argument>([] { (void)Interpolate(500, 500, 500, 2, 4); }));
    Check("spectrum interpolation legacy helper rejects NaN", Rejects<std::invalid_argument>([] { (void)Interpolate(std::numeric_limits<float>::quiet_NaN(), 400, 600, 2, 4); }));
    Check("spectrum interpolation legacy helper rejects nonfinite values", Rejects<std::invalid_argument>([] { (void)Interpolate(500, 400, 600, 2, std::numeric_limits<float>::infinity()); }));
    const SpectralReflectanceCurve reflectance({{400, 0.2}, {600, 0.6}, {1600, 0.9}});
    const SpectralOpticalConstants optical({{400, 0.2, 2}, {600, 0.6, 4}, {1600, 0.9, 8}});
    for (double wavelength : {400.0, 450.0, 500.0, 600.0, 905.0, 1000.0, 1550.0, 1600.0})
    {
        Check("spectrum interpolation containers agree on identical scalar samples", reflectance.Evaluate(wavelength) == optical.Evaluate(wavelength).n);
        const double expected = wavelength <= 600 ? 0.2 + (wavelength - 400) * 0.002 : 0.6 + (wavelength - 600) * 0.0003;
        Check("spectrum interpolation containers match independent analytic segments", Near(reflectance.Evaluate(wavelength), expected));
    }
    Check("spectrum interpolation strict containers do not inherit clamping", Rejects<std::out_of_range>([&] { (void)reflectance.Evaluate(300); }) && Rejects<std::out_of_range>([&] { (void)optical.Evaluate(300); }));
    Check("spectrum interpolation does not merge physical validation rules", Rejects<std::invalid_argument>([] { (void)SpectralReflectanceCurve({{400, 2}, {600, 3}}); }) && SpectralOpticalConstants({{400, 2, 3}, {600, 3, 4}}).Evaluate(500).n == 2.5);
    const float lambda[]{400, 600, 700}, values[]{2, 4, 5};
    const SampledSpectrum legacy;
    Check("spectrum interpolation legacy average remains an integral", Near(legacy.AverageSpectrumSamples(lambda, values, 3, 400, 600), 3, 1e-6));
    Check("spectrum interpolation legacy average spans multiple segments", Near(legacy.AverageSpectrumSamples(lambda, values, 3, 500, 650), 3.75, 1e-6));
    Check("spectrum interpolation legacy average retains boundary extension", Near(legacy.AverageSpectrumSamples(lambda, values, 3, 300, 500), 2.25, 1e-6));
    Check("spectrum interpolation legacy average wholly below support", legacy.AverageSpectrumSamples(lambda, values, 3, 200, 300) == 2);
    Check("spectrum interpolation legacy average wholly above support", legacy.AverageSpectrumSamples(lambda, values, 3, 750, 850) == 5);
    const float constant[]{7, 7, 7};
    const auto resampled = SampledSpectrum::FromSampled(lambda, constant, 3);
    bool constantPreserved = true;
    for (float value : resampled.c) constantPreserved = constantPreserved && Near(value, 7, 2e-6);
    Check("spectrum interpolation legacy resampling preserves constant spectra", constantPreserved);
    float x, y, z;
    CIE_XYZ(550, x, y, z);
    Check("spectrum interpolation legacy header links from two translation units", Near(SpectrumInterpolationOtherTranslationUnit(), 3.0f + y, 1e-6));
}
