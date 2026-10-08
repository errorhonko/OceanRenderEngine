#include "FresnelDielectric.h"
#include "FresnelUtils.h"

#include <cmath>
#include <concepts>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
void ExpectTrue(
    const std::string& testName,
    bool condition)
{
    if (!condition)
    {
        std::cerr << "[FAIL] " << testName << '\n';
        throw std::runtime_error(testName + " failed");
    }

    std::cout << "[PASS] " << testName << '\n';
}

bool Near(
    double actual,
    double expected,
    double tolerance = 1.0e-12)
{
    return std::fabs(actual - expected) <= tolerance;
}
}

void RunFresnelUtilsAcceptanceTests()
{
    static_assert(
        std::same_as<
            decltype(FrDielectric(1.0f, 1.0f, 1.5f)),
            float>);

    static_assert(
        std::same_as<
            decltype(FrDielectric(1.0, 1.0, 1.5)),
            double>);

    constexpr double etaAir = 1.0;
    constexpr double etaGlass = 1.5;

    const double normalAmplitude =
        (etaAir - etaGlass) /
        (etaAir + etaGlass);

    const double expectedNormalReflectance =
        normalAmplitude * normalAmplitude;

    ExpectTrue(
        "dielectric Fresnel normal incidence",
        Near(
            FrDielectric(
                1.0,
                etaAir,
                etaGlass),
            expectedNormalReflectance));

    ExpectTrue(
        "dielectric Fresnel clamps incident cosine",
        Near(
            FrDielectric(
                2.0,
                etaAir,
                etaGlass),
            expectedNormalReflectance));

    ExpectTrue(
        "dielectric Fresnel grazing incidence",
        Near(
            FrDielectric(
                0.0,
                etaAir,
                etaGlass),
            1.0));

    ExpectTrue(
        "dielectric Fresnel total internal reflection",
        Near(
            FrDielectric(
                0.5,
                etaGlass,
                etaAir),
            1.0));

    constexpr double cosine = 0.8;

    ExpectTrue(
        "dielectric Fresnel swaps media for exiting direction",
        Near(
            FrDielectric(
                cosine,
                etaAir,
                etaGlass),
            FrDielectric(
                -cosine,
                etaGlass,
                etaAir)));

    const float floatReflectance =
        FrDielectric(
            0.8f,
            1.0f,
            1.5f);

    const double doubleReflectance =
        FrDielectric(
            0.8,
            1.0,
            1.5);

    ExpectTrue(
        "dielectric Fresnel float and double agree",
        Near(
            static_cast<double>(floatReflectance),
            doubleReflectance,
            1.0e-6));

    FresnelDielectric spectrumFresnel(
        1.0f,
        1.5f);

    const Spectrum spectrumReflectance =
        spectrumFresnel.Evaluate(1.0f);

    float rgb[3]{};
    spectrumReflectance.ToRGB(rgb);

    ExpectTrue(
        "dielectric Fresnel spectrum wrapper",
        Near(rgb[0], expectedNormalReflectance, 1.0e-6) &&
        Near(rgb[1], expectedNormalReflectance, 1.0e-6) &&
        Near(rgb[2], expectedNormalReflectance, 1.0e-6));
}
