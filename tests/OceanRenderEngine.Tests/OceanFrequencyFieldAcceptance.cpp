#include "OceanFrequencyField.h"

#include <cmath>
#include <complex>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

namespace
{
void ExpectTrue(
    const std::string& testName,
    bool condition)
{
    if (!condition)
        throw std::runtime_error(testName + " failed");

    std::cout << "[PASS] " << testName << '\n';
}

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

void ExpectComplexNear(
    const std::string& testName,
    const std::complex<float>& actual,
    const std::complex<float>& expected,
    float epsilon = 1e-5f)
{
    if (!std::isfinite(actual.real()) ||
        !std::isfinite(actual.imag()) ||
        std::abs(actual - expected) > epsilon)
    {
        throw std::runtime_error(testName + " failed");
    }

    std::cout << "[PASS] " << testName << '\n';
}

template <typename Function>
void ExpectThrows(
    const std::string& testName,
    Function&& function)
{
    bool threw = false;

    try
    {
        function();
    }
    catch (const std::exception&)
    {
        threw = true;
    }

    ExpectTrue(testName, threw);
}
}

void RunOceanFrequencyFieldAcceptanceTests()
{
    OceanFrequencyConfig config;
    config.resolution = 8;
    config.patchLength =
        2.0f * std::numbers::pi_v<float>;
    config.seed = 42;

    const auto constantSpectrum =
        [](float, float)
        {
            return 2.0f;
        };

    const auto dispersion =
        [](float k)
        {
            return 2.0f * k;
        };

    OceanFrequencyField field(
        config,
        constantSpectrum,
        dispersion);

    ExpectTrue(
        "ocean frequency field element count",
        field.InitialAmplitudes().size() == 64);

    ExpectFloatNear(
        "ocean frequency delta k",
        field.DeltaK(),
        1.0f);

    ExpectFloatNear(
        "ocean frequency positive layout",
        field.WaveNumber(3),
        3.0f);

    ExpectFloatNear(
        "ocean frequency negative layout",
        field.WaveNumber(7),
        -1.0f);

    ExpectComplexNear(
        "ocean frequency zero initial amplitude",
        field.H0(0, 0),
        std::complex<float>(0.0f, 0.0f));

    ExpectComplexNear(
        "ocean frequency zero evolved amplitude",
        field.H(0, 0, 1.25f),
        std::complex<float>(0.0f, 0.0f));

    OceanFrequencyField sameSeedField(
        config,
        constantSpectrum,
        dispersion);

    bool sameSeedMatches = true;
    for (int z = 0; z < config.resolution; ++z)
    {
        for (int x = 0; x < config.resolution; ++x)
        {
            sameSeedMatches =
                sameSeedMatches &&
                field.H0(x, z) == sameSeedField.H0(x, z);
        }
    }

    ExpectTrue(
        "ocean frequency same seed reproducibility",
        sameSeedMatches);

    OceanFrequencyConfig differentSeedConfig = config;
    differentSeedConfig.seed = 43;
    OceanFrequencyField differentSeedField(
        differentSeedConfig,
        constantSpectrum,
        dispersion);

    bool differentSeedChangesField = false;
    for (int z = 0; z < config.resolution; ++z)
    {
        for (int x = 0; x < config.resolution; ++x)
        {
            if (field.H0(x, z) != differentSeedField.H0(x, z))
                differentSeedChangesField = true;
        }
    }

    ExpectTrue(
        "ocean frequency different seed changes field",
        differentSeedChangesField);

    const float testTimes[] = { 0.0f, 0.37f, 2.5f };
    bool conjugateSymmetry = true;

    for (float time : testTimes)
    {
        for (int z = 0; z < config.resolution; ++z)
        {
            for (int x = 0; x < config.resolution; ++x)
            {
                const int mirrorX =
                    (config.resolution - x) % config.resolution;
                const int mirrorZ =
                    (config.resolution - z) % config.resolution;

                conjugateSymmetry =
                    conjugateSymmetry &&
                    std::abs(
                        field.H(mirrorX, mirrorZ, time) -
                        std::conj(field.H(x, z, time))) < 1e-5f;
            }
        }
    }

    ExpectTrue(
        "ocean frequency evolved conjugate symmetry",
        conjugateSymmetry);

    ExpectTrue(
        "ocean frequency evolves with time",
        std::abs(field.H(1, 2, 0.0f) -
                 field.H(1, 2, 0.37f)) > 1e-6f);

    constexpr float batchTime = 0.75f;
    std::vector<std::complex<float>> currentSpectrum;
    field.BuildSpectrumAtTime(
        batchTime,
        currentSpectrum);

    ExpectTrue(
        "ocean frequency batch spectrum size",
        currentSpectrum.size() ==
            field.InitialAmplitudes().size());

    bool batchMatchesIndividual = true;
    bool batchConjugateSymmetry = true;

    for (int z = 0; z < config.resolution; ++z)
    {
        for (int x = 0; x < config.resolution; ++x)
        {
            const std::size_t index =
                static_cast<std::size_t>(z) *
                static_cast<std::size_t>(config.resolution) +
                static_cast<std::size_t>(x);

            const int mirrorX =
                (config.resolution - x) % config.resolution;
            const int mirrorZ =
                (config.resolution - z) % config.resolution;
            const std::size_t mirrorIndex =
                static_cast<std::size_t>(mirrorZ) *
                static_cast<std::size_t>(config.resolution) +
                static_cast<std::size_t>(mirrorX);

            batchMatchesIndividual =
                batchMatchesIndividual &&
                std::abs(
                    currentSpectrum[index] -
                    field.H(x, z, batchTime)) < 1e-6f;

            batchConjugateSymmetry =
                batchConjugateSymmetry &&
                std::abs(
                    currentSpectrum[mirrorIndex] -
                    std::conj(currentSpectrum[index])) < 1e-5f;
        }
    }

    ExpectTrue(
        "ocean frequency batch matches individual modes",
        batchMatchesIndividual);

    ExpectTrue(
        "ocean frequency batch conjugate symmetry",
        batchConjugateSymmetry);

    ExpectThrows(
        "ocean frequency batch invalid time rejected",
        [&field, &currentSpectrum]
        {
            field.BuildSpectrumAtTime(
                std::numeric_limits<float>::infinity(),
                currentSpectrum);
        });

    OceanFrequencyField zeroSpectrumField(
        config,
        [](float, float) { return 0.0f; },
        dispersion);

    bool zeroSpectrumIsZero = true;
    for (const std::complex<float>& value :
         zeroSpectrumField.InitialAmplitudes())
    {
        zeroSpectrumIsZero =
            zeroSpectrumIsZero && value == std::complex<float>(0.0f, 0.0f);
    }

    ExpectTrue(
        "ocean frequency zero spectrum produces zero field",
        zeroSpectrumIsZero);

    ExpectThrows(
        "ocean frequency invalid spectrum rejected",
        [&config, &dispersion]
        {
            OceanFrequencyField invalid(
                config,
                [](float, float) { return -1.0f; },
                dispersion);
        });

    ExpectThrows(
        "ocean frequency invalid dispersion rejected",
        [&config, &constantSpectrum]
        {
            OceanFrequencyField invalid(
                config,
                constantSpectrum,
                [](float) { return -1.0f; });
        });

    ExpectThrows(
        "ocean frequency index bounds",
        [&field]
        {
            static_cast<void>(field.H0(-1, 0));
        });

    ExpectThrows(
        "ocean frequency invalid time rejected",
        [&field]
        {
            static_cast<void>(field.H(
                0,
                0,
                std::numeric_limits<float>::quiet_NaN()));
        });
}
