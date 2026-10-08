#pragma once

#include <cmath>
#include <stdexcept>

// 理想、无内部倍增的线性光电探测器。
// 固定波长和量子效率；不含噪声、带宽、放大器及饱和。
class LinearPhotodetector
{
public:
    LinearPhotodetector(
        float wavelengthNm,
        double quantumEfficiency)
    {
        if (!std::isfinite(wavelengthNm) ||
            wavelengthNm <= 0.0f)
        {
            throw std::invalid_argument(
                "Wavelength must be finite and positive.");
        }

        if (!std::isfinite(quantumEfficiency) ||
            quantumEfficiency < 0.0 ||
            quantumEfficiency > 1.0)
        {
            throw std::invalid_argument(
                "Quantum efficiency must be in [0, 1].");
        }

        constexpr double electronChargeC =
            1.602176634e-19;

        constexpr double planckConstantJs =
            6.62607015e-34;

        constexpr double speedOfLight =
            299792458.0;

        const double wavelengthMeters =
            double(wavelengthNm) * 1.0e-9;

        responsivityAmpsPerWatt =
            quantumEfficiency *
            electronChargeC *
            wavelengthMeters /
            (planckConstantJs * speedOfLight);
    }

    double ResponsivityAmpsPerWatt() const
    {
        return responsivityAmpsPerWatt;
    }

    double EnergyToChargeC(double energyJ) const
    {
        if (!std::isfinite(energyJ) || energyJ < 0.0)
        {
            throw std::invalid_argument(
                "Optical energy must be finite "
                "and nonnegative.");
        }

        const double chargeC =
            responsivityAmpsPerWatt * energyJ;

        if (!std::isfinite(chargeC))
        {
            throw std::overflow_error(
                "Photodetector charge overflow.");
        }

        return chargeC;
    }

    double EnergyToAverageCurrentA(
        double energyJ,
        double binWidthSeconds) const
    {
        if (!std::isfinite(binWidthSeconds) ||
            binWidthSeconds <= 0.0)
        {
            throw std::invalid_argument(
                "Bin width must be finite and positive.");
        }

        const double currentA =
            EnergyToChargeC(energyJ) / binWidthSeconds;

        if (!std::isfinite(currentA))
        {
            throw std::overflow_error(
                "Photodetector current overflow.");
        }

        return currentA;
    }

private:
    double responsivityAmpsPerWatt = 0.0;
};