#include "ElfouhailySpectrum.h"

#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>

namespace
{
void ExpectFloatNear(
    const std::string& testName,
    float actual,
    float expected,
    float epsilon = 1e-6f)
{
    if (!std::isfinite(actual) ||
        std::fabs(actual - expected) > epsilon)
    {
        throw std::runtime_error(
            testName + " failed, actual = " +
            std::to_string(actual) +
            ", expected = " +
            std::to_string(expected));
    }

    std::cout << "[PASS] " << testName << '\n';
}

void ExpectTrue(
    const std::string& testName,
    bool condition)
{
    if (!condition)
        throw std::runtime_error(testName + " failed");

    std::cout << "[PASS] " << testName << '\n';
}

double IntegrateSpreading(
    const ElfouhailySpectrum& spectrum,
    float k)
{
    constexpr int sampleCount = 16384;
    constexpr double twoPi =
        2.0 * std::numbers::pi_v<double>;
    const double step = twoPi / sampleCount;
    double integral = 0.0;

    for (int i = 0; i < sampleCount; ++i)
    {
        const float phi =
            static_cast<float>((i + 0.5) * step);
        integral += spectrum.Spreading(k, phi) * step;
    }

    return integral;
}
}

void RunElfouhailySpectrumAcceptanceTests()
{
    ElfouhailyConfig config;
    ElfouhailySpectrum spectrum(config);

    const float cp =
        config.windSpeed10m /
        config.inverseWaveAge;
    const float kp =
        config.gravity / (cp * cp);
    const float km =
        std::sqrt(
            config.waterDensity *
            config.gravity /
            config.surfaceTension);
    const float pi =
        std::numbers::pi_v<float>;

    ExpectFloatNear(
        "Elfouhaily phase speed zero wavenumber",
        spectrum.PhaseSpeed(0.0f),
        0.0f);
    ExpectFloatNear(
        "Elfouhaily long-wave zero wavenumber",
        spectrum.LongWaveCurvature(0.0f),
        0.0f);
    ExpectFloatNear(
        "Elfouhaily short-wave zero wavenumber",
        spectrum.ShortWaveCurvature(0.0f),
        0.0f);
    ExpectFloatNear(
        "Elfouhaily spreading zero wavenumber",
        spectrum.Spreading(0.0f, 0.0f),
        0.0f);
    ExpectFloatNear(
        "Elfouhaily Cartesian zero wave vector",
        spectrum.CartesianSpectrum(0.0f, 0.0f),
        0.0f);

    ExpectFloatNear(
        "Elfouhaily long-wave peak reference",
        spectrum.LongWaveCurvature(kp),
        0.0013391885f,
        2.0e-7f);

    ElfouhailyConfig youngSeaConfig = config;
    youngSeaConfig.inverseWaveAge = 2.0f;
    ElfouhailySpectrum youngSea(youngSeaConfig);
    const float youngCp =
        youngSeaConfig.windSpeed10m /
        youngSeaConfig.inverseWaveAge;
    const float youngKp =
        youngSeaConfig.gravity /
        (youngCp * youngCp);
    ExpectFloatNear(
        "Elfouhaily young-sea gamma branch",
        youngSea.LongWaveCurvature(youngKp),
        0.0071216845f,
        2.0e-6f);

    ExpectFloatNear(
        "Elfouhaily short-wave high-friction branch",
        spectrum.ShortWaveCurvature(km),
        0.012392798f,
        2.0e-6f);

    ElfouhailyConfig lowFrictionConfig = config;
    lowFrictionConfig.frictionVelocity = 0.15f;
    ElfouhailySpectrum lowFriction(lowFrictionConfig);
    ExpectFloatNear(
        "Elfouhaily short-wave low-friction branch",
        lowFriction.ShortWaveCurvature(km),
        0.0028165861f,
        2.0e-6f);

    ExpectFloatNear(
        "Elfouhaily spreading dominant-direction reference",
        spectrum.Spreading(kp, 0.0f),
        0.3182344f,
        2.0e-6f);
    ExpectFloatNear(
        "Elfouhaily spreading crosswind reference",
        spectrum.Spreading(km, pi * 0.5f),
        0.1005586f,
        2.0e-6f);

    const float testPhi = 0.37f;
    ExpectFloatNear(
        "Elfouhaily spreading centrosymmetry",
        spectrum.Spreading(kp, testPhi),
        spectrum.Spreading(kp, testPhi + pi),
        2.0e-6f);
    ExpectTrue(
        "Elfouhaily windward exceeds crosswind",
        spectrum.Spreading(kp, 0.0f) >
        spectrum.Spreading(kp, pi * 0.5f));

    ExpectFloatNear(
        "Elfouhaily spreading normalization at gravity peak",
        static_cast<float>(IntegrateSpreading(spectrum, kp)),
        1.0f,
        3.0e-6f);
    ExpectFloatNear(
        "Elfouhaily spreading normalization at capillary peak",
        static_cast<float>(IntegrateSpreading(spectrum, km)),
        1.0f,
        3.0e-6f);

    const float cartesian =
        spectrum.CartesianSpectrum(kp, 0.0f);
    const float polarWithoutJacobian =
        spectrum.Omnidirectional(kp) *
        spectrum.Spreading(kp, 0.0f);
    ExpectFloatNear(
        "Elfouhaily Cartesian polar Jacobian",
        cartesian * kp,
        polarWithoutJacobian,
        std::fabs(polarWithoutJacobian) * 2.0e-5f +
        1.0e-8f);

    ElfouhailyConfig rotatedConfig = config;
    rotatedConfig.windDirection = pi * 0.5f;
    ElfouhailySpectrum rotated(rotatedConfig);
    ExpectTrue(
        "Elfouhaily wind direction rotates Cartesian spectrum",
        rotated.CartesianSpectrum(0.0f, kp) >
        rotated.CartesianSpectrum(kp, 0.0f));

    bool finiteAndNonnegative = true;
    for (int i = 0;
         i <= 140 && finiteAndNonnegative;
         ++i)
    {
        const float exponent =
            -3.0f + i * (7.0f / 140.0f);
        const float k =
            std::pow(10.0f, exponent);

        const float omnidirectional =
            spectrum.Omnidirectional(k);
        if (!std::isfinite(omnidirectional) ||
            omnidirectional < 0.0f)
        {
            finiteAndNonnegative = false;
            break;
        }

        for (int j = 0; j < 32; ++j)
        {
            const float phi =
                2.0f * pi * j / 32.0f;
            const float directional =
                spectrum.Spreading(k, phi);
            const float cart =
                spectrum.CartesianSpectrum(
                    k * std::cos(phi),
                    k * std::sin(phi));

            if (!std::isfinite(directional) ||
                directional < 0.0f ||
                !std::isfinite(cart) ||
                cart < 0.0f)
            {
                finiteAndNonnegative = false;
                break;
            }
        }
    }
    ExpectTrue(
        "Elfouhaily spectrum finite and nonnegative scan",
        finiteAndNonnegative);

    ElfouhailyConfig invalidWaveAgeConfig = config;
    invalidWaveAgeConfig.inverseWaveAge = 0.5f;
    ElfouhailySpectrum invalidWaveAge(invalidWaveAgeConfig);
    ExpectFloatNear(
        "Elfouhaily invalid wave age rejects long-wave spectrum",
        invalidWaveAge.LongWaveCurvature(kp),
        0.0f);
    ExpectFloatNear(
        "Elfouhaily invalid wave age rejects spreading",
        invalidWaveAge.Spreading(kp, 0.0f),
        0.0f);

    ElfouhailyConfig zeroFrictionConfig = config;
    zeroFrictionConfig.frictionVelocity = 0.0f;
    ElfouhailySpectrum zeroFriction(zeroFrictionConfig);
    ExpectFloatNear(
        "Elfouhaily zero friction rejects short-wave spectrum",
        zeroFriction.ShortWaveCurvature(km),
        0.0f);
    ExpectFloatNear(
        "Elfouhaily zero friction rejects spreading",
        zeroFriction.Spreading(km, 0.0f),
        0.0f);
}
