#include "GaussianPulseProfile.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>

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

bool Near(
    double actual,
    double expected,
    double relativeTolerance = 1.0e-12,
    double absoluteTolerance = 1.0e-15)
{
    const double error = std::fabs(actual - expected);
    const double scale = std::fmax(
        std::fabs(actual),
        std::fabs(expected));

    return error <= std::fmax(
        absoluteTolerance,
        relativeTolerance * scale);
}
}

void RunLidarPulseProfileAcceptanceTests()
{
    static_assert(
        std::is_abstract_v<LidarPulseProfile>);

    constexpr double fwhmSeconds = 4.0e-9;
    GaussianPulseProfile gaussian(fwhmSeconds);

    const double expectedSigma =
        fwhmSeconds /
        (2.0 * std::sqrt(
            2.0 * std::log(2.0)));

    ExpectTrue(
        "Gaussian pulse exposes FWHM and sigma",
        Near(gaussian.FwhmSeconds(),
             fwhmSeconds) &&
        Near(gaussian.StandardDeviationSeconds(),
             expectedSigma));

    const LidarPulseProfile& profile = gaussian;
    const double sigma =
        gaussian.StandardDeviationSeconds();

    ExpectTrue(
        "Gaussian pulse dispatches through profile interface",
        Near(profile.FractionBetween(
                 -sigma,
                 sigma),
             std::erf(1.0 / std::sqrt(2.0))));

    const double leftFraction =
        profile.FractionBetween(
            -2.0 * sigma,
            -sigma);

    const double rightFraction =
        profile.FractionBetween(
            sigma,
            2.0 * sigma);

    ExpectTrue(
        "Gaussian pulse is symmetric",
        Near(leftFraction,
             rightFraction));

    const double negativeHalf =
        profile.FractionBetween(
            -10.0 * sigma,
            0.0);

    const double positiveHalf =
        profile.FractionBetween(
            0.0,
            10.0 * sigma);

    ExpectTrue(
        "Gaussian pulse splits equally at its center",
        Near(negativeHalf, 0.5) &&
        Near(positiveHalf, 0.5));

    ExpectTrue(
        "Gaussian pulse integrates to unit energy",
        Near(profile.FractionBetween(
                 -10.0 * sigma,
                 10.0 * sigma),
             1.0));

    const double leftPart =
        profile.FractionBetween(
            -sigma,
            0.0);

    const double rightPart =
        profile.FractionBetween(
            0.0,
            sigma);

    const double wholePart =
        profile.FractionBetween(
            -sigma,
            sigma);

    ExpectTrue(
        "Gaussian pulse interval fractions are additive",
        Near(leftPart + rightPart,
             wholePart));

    ExpectTrue(
        "Gaussian pulse zero-width interval has zero energy",
        Near(profile.FractionBetween(
                 0.25 * sigma,
                 0.25 * sigma),
             0.0));

    ExpectThrows(
        "Gaussian pulse rejects zero FWHM",
        []
        {
            GaussianPulseProfile invalid(0.0);
        });

    ExpectThrows(
        "Gaussian pulse rejects negative FWHM",
        []
        {
            GaussianPulseProfile invalid(-1.0e-9);
        });

    ExpectThrows(
        "Gaussian pulse rejects nonfinite FWHM",
        []
        {
            GaussianPulseProfile invalid(
                std::numeric_limits<double>::infinity());
        });

    ExpectThrows(
        "Gaussian pulse rejects nonfinite interval",
        [&]
        {
            profile.FractionBetween(
                std::numeric_limits<double>::quiet_NaN(),
                0.0);
        });

    ExpectThrows(
        "Gaussian pulse rejects reversed interval",
        [&]
        {
            profile.FractionBetween(
                sigma,
                -sigma);
        });
}
