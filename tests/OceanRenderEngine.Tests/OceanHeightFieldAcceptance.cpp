#include "OceanHeightField.h"

#include "OceanFFT.h"
#include "OceanFrequencyField.h"

#include <cmath>
#include <complex>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

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

void RunOceanHeightFieldAcceptanceTests()
{
    OceanFrequencyConfig config;
    config.resolution = 8;
    config.patchLength =
        2.0f * std::numbers::pi_v<float>;
    config.seed = 42;

    const auto spectrum =
        [](float, float)
        {
            return 0.1f;
        };

    const auto dispersion =
        [](float k)
        {
            return 1.0f + k;
        };

    OceanFrequencyField frequencyField(
        config,
        spectrum,
        dispersion);
    OceanHeightField heightField(
        frequencyField);

    constexpr float testTime = 0.75f;
    heightField.Update(testTime);

    ExpectTrue(
        "ocean height field resolution",
        heightField.Resolution() == config.resolution);

    ExpectTrue(
        "ocean height field element count",
        heightField.Heights().size() == 64);

    bool allFinite = true;
    double heightSum = 0.0;
    for (float height : heightField.Heights())
    {
        allFinite = allFinite && std::isfinite(height);
        heightSum += height;
    }

    ExpectTrue(
        "ocean height field finite values",
        allFinite);

    const double meanHeight =
        heightSum /
        static_cast<double>(heightField.Heights().size());
    ExpectTrue(
        "ocean height field zero mean",
        std::fabs(meanHeight) < 1e-5);

    ExpectTrue(
        "ocean height field imaginary residual",
        std::isfinite(heightField.MaxImaginaryResidual()) &&
        heightField.MaxImaginaryResidual() < 1e-4f);

    ExpectTrue(
        "ocean height accessor layout",
        heightField.Height(3, 2) ==
            heightField.Heights()[2 * config.resolution + 3]);

    OceanFrequencyField sameFrequencyField(
        config,
        spectrum,
        dispersion);
    OceanHeightField sameHeightField(
        sameFrequencyField);
    sameHeightField.Update(testTime);

    bool sameSeedAndTimeMatch = true;
    for (std::size_t i = 0;
         i < heightField.Heights().size();
         ++i)
    {
        sameSeedAndTimeMatch =
            sameSeedAndTimeMatch &&
            heightField.Heights()[i] ==
                sameHeightField.Heights()[i];
    }

    ExpectTrue(
        "ocean height field deterministic",
        sameSeedAndTimeMatch);

    std::vector<float> firstFrame =
        heightField.Heights();
    heightField.Update(testTime + 0.37f);

    bool changesWithTime = false;
    for (std::size_t i = 0;
         i < firstFrame.size();
         ++i)
    {
        if (std::fabs(
                firstFrame[i] -
                heightField.Heights()[i]) > 1e-6f)
        {
            changesWithTime = true;
        }
    }

    ExpectTrue(
        "ocean height field evolves with time",
        changesWithTime);

    std::vector<std::complex<float>> manualBuffer;
    frequencyField.BuildSpectrumAtTime(
        testTime,
        manualBuffer);
    OceanFFT::Inverse2D(
        manualBuffer,
        config.resolution);
    const float physicalScale =
        static_cast<float>(config.resolution) *
        static_cast<float>(config.resolution);

    heightField.Update(testTime);
    bool matchesManualPipeline = true;
    for (std::size_t i = 0;
         i < manualBuffer.size();
         ++i)
    {
        matchesManualPipeline =
            matchesManualPipeline &&
            std::fabs(
                heightField.Heights()[i] -
                manualBuffer[i].real() * physicalScale) < 1e-6f;
    }

    ExpectTrue(
        "ocean height field matches frequency FFT pipeline",
        matchesManualPipeline);

    OceanFrequencyField zeroFrequencyField(
        config,
        [](float, float) { return 0.0f; },
        dispersion);
    OceanHeightField zeroHeightField(
        zeroFrequencyField);
    zeroHeightField.Update(1.25f);

    bool zeroSpectrumProducesZeroHeight = true;
    for (float height : zeroHeightField.Heights())
    {
        zeroSpectrumProducesZeroHeight =
            zeroSpectrumProducesZeroHeight &&
            height == 0.0f;
    }

    ExpectTrue(
        "ocean height field zero spectrum",
        zeroSpectrumProducesZeroHeight);

    ExpectThrows(
        "ocean height field negative index rejected",
        [&heightField]
        {
            static_cast<void>(heightField.Height(-1, 0));
        });

    ExpectThrows(
        "ocean height field upper index rejected",
        [&heightField, &config]
        {
            static_cast<void>(
                heightField.Height(config.resolution, 0));
        });

    ExpectThrows(
        "ocean height field invalid time rejected",
        [&heightField]
        {
            heightField.Update(
                std::numeric_limits<float>::quiet_NaN());
        });

    ExpectThrows(
        "ocean height field non-power-of-two rejected",
        [&spectrum, &dispersion]
        {
            OceanFrequencyConfig invalidConfig;
            invalidConfig.resolution = 6;
            invalidConfig.patchLength = 10.0f;
            OceanFrequencyField invalidFrequencyField(
                invalidConfig,
                spectrum,
                dispersion);
            OceanHeightField invalidHeightField(
                invalidFrequencyField);
            invalidHeightField.Update(0.0f);
        });
}
