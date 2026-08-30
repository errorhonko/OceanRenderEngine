#include "OceanFFT.h"

#include <cmath>
#include <complex>
#include <iostream>
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

bool IsRealFieldNear(
    const std::vector<std::complex<float>>& values,
    float epsilon = 1e-6f)
{
    for (const std::complex<float>& value : values)
    {
        if (!std::isfinite(value.real()) ||
            !std::isfinite(value.imag()) ||
            std::fabs(value.imag()) > epsilon)
        {
            return false;
        }
    }

    return true;
}
}

void RunOceanFFTAcceptanceTests()
{
    constexpr int resolution = 4;
    constexpr std::size_t elementCount =
        static_cast<std::size_t>(resolution) *
        static_cast<std::size_t>(resolution);
    constexpr float physicalScale =
        static_cast<float>(elementCount);
    const float twoPi =
        2.0f * std::numbers::pi_v<float>;

    std::vector<std::complex<float>> dc(
        elementCount,
        std::complex<float>(0.0f, 0.0f));
    dc[0] = std::complex<float>(1.0f, 0.0f);
    OceanFFT::Inverse2D(dc, resolution);

    bool dcIsConstant = true;
    for (const std::complex<float>& value : dc)
    {
        dcIsConstant =
            dcIsConstant &&
            std::fabs(value.real() * physicalScale - 1.0f) < 1e-6f &&
            std::fabs(value.imag()) < 1e-6f;
    }

    ExpectTrue(
        "ocean FFT DC produces constant height",
        dcIsConstant);

    std::vector<std::complex<float>> xCosine(
        elementCount,
        std::complex<float>(0.0f, 0.0f));
    xCosine[1] = std::complex<float>(0.5f, 0.0f);
    xCosine[resolution - 1] =
        std::complex<float>(0.5f, 0.0f);
    OceanFFT::Inverse2D(xCosine, resolution);

    bool xCosineMatches = true;
    for (int z = 0; z < resolution; ++z)
    {
        for (int x = 0; x < resolution; ++x)
        {
            const std::size_t index =
                static_cast<std::size_t>(z) * resolution + x;
            const float expected =
                std::cos(twoPi * x / resolution);
            const float actual =
                xCosine[index].real() * physicalScale;
            xCosineMatches =
                xCosineMatches &&
                std::fabs(actual - expected) < 1e-5f;
        }
    }

    ExpectTrue(
        "ocean FFT X cosine reconstruction",
        xCosineMatches);

    std::vector<std::complex<float>> zCosine(
        elementCount,
        std::complex<float>(0.0f, 0.0f));
    zCosine[resolution] =
        std::complex<float>(0.5f, 0.0f);
    zCosine[(resolution - 1) * resolution] =
        std::complex<float>(0.5f, 0.0f);
    OceanFFT::Inverse2D(zCosine, resolution);

    bool zCosineMatches = true;
    for (int z = 0; z < resolution; ++z)
    {
        for (int x = 0; x < resolution; ++x)
        {
            const std::size_t index =
                static_cast<std::size_t>(z) * resolution + x;
            const float expected =
                std::cos(twoPi * z / resolution);
            const float actual =
                zCosine[index].real() * physicalScale;
            zCosineMatches =
                zCosineMatches &&
                std::fabs(actual - expected) < 1e-5f;
        }
    }

    ExpectTrue(
        "ocean FFT Z cosine reconstruction",
        zCosineMatches);

    std::vector<std::complex<float>> diagonalCosine(
        elementCount,
        std::complex<float>(0.0f, 0.0f));
    diagonalCosine[resolution + 1] =
        std::complex<float>(0.5f, 0.0f);
    diagonalCosine[
        (resolution - 1) * resolution +
        (resolution - 1)] =
        std::complex<float>(0.5f, 0.0f);
    OceanFFT::Inverse2D(
        diagonalCosine,
        resolution);

    bool diagonalCosineMatches = true;
    for (int z = 0; z < resolution; ++z)
    {
        for (int x = 0; x < resolution; ++x)
        {
            const std::size_t index =
                static_cast<std::size_t>(z) * resolution + x;
            const float expected =
                std::cos(
                    twoPi * (x + z) /
                    resolution);
            const float actual =
                diagonalCosine[index].real() *
                physicalScale;
            diagonalCosineMatches =
                diagonalCosineMatches &&
                std::fabs(actual - expected) < 1e-5f;
        }
    }

    ExpectTrue(
        "ocean FFT diagonal cosine reconstruction",
        diagonalCosineMatches);

    ExpectTrue(
        "ocean FFT conjugate spectrum produces real field",
        IsRealFieldNear(xCosine) &&
        IsRealFieldNear(zCosine) &&
        IsRealFieldNear(diagonalCosine));

    ExpectThrows(
        "ocean FFT rejects non-power-of-two resolution",
        []
        {
            std::vector<std::complex<float>> values(9);
            OceanFFT::Inverse2D(values, 3);
        });

    ExpectThrows(
        "ocean FFT rejects mismatched input size",
        []
        {
            std::vector<std::complex<float>> values(15);
            OceanFFT::Inverse2D(values, 4);
        });

    ExpectThrows(
        "ocean FFT rejects zero resolution",
        []
        {
            std::vector<std::complex<float>> values;
            OceanFFT::Inverse2D(values, 0);
        });
}
