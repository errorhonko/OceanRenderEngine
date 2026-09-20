#pragma once

#include "LidarPulseProfile.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

class GaussianPulseProfile final
    : public LidarPulseProfile
{
public:
    explicit GaussianPulseProfile(
        double fwhmSeconds)
        : fwhmSeconds(fwhmSeconds)
    {
        if (!std::isfinite(fwhmSeconds) ||
            fwhmSeconds <= 0.0)
        {
            throw std::invalid_argument(
                "Gaussian pulse FWHM must be finite and positive.");
        }

        sigmaSeconds =
            fwhmSeconds /
            (2.0 * std::sqrt(
                2.0 * std::log(2.0)));
    }

    double FractionBetween(
        double relativeStartSeconds,
        double relativeEndSeconds) const override
    {
        if (!std::isfinite(relativeStartSeconds) ||
            !std::isfinite(relativeEndSeconds))
        {
            throw std::invalid_argument(
                "Pulse interval must be finite.");
        }

        if (relativeEndSeconds <
            relativeStartSeconds)
        {
            throw std::invalid_argument(
                "Pulse interval end must not precede start.");
        }

        const double denominator =
            std::sqrt(2.0) * sigmaSeconds;

        const double z0 =
            relativeStartSeconds /
            denominator;

        const double z1 =
            relativeEndSeconds /
            denominator;

        const double fraction =
            0.5 *
            (std::erf(z1) -
                std::erf(z0));

        return std::clamp(
            fraction, 0.0, 1.0);
    }

    double FwhmSeconds() const
    {
        return fwhmSeconds;
    }

    double StandardDeviationSeconds() const
    {
        return sigmaSeconds;
    }

private:
    double fwhmSeconds = 0.0;
    double sigmaSeconds = 0.0;
};
