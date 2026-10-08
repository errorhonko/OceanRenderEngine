#include "OceanSlopeVariance.h"
#include "ElfouhailySpectrum.h"
#include "OceanFrequencyField.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

namespace
{
void Expect(const std::string& name, bool ok)
{
    if (!ok)
        throw std::runtime_error(name + " failed");
    std::cout << "[PASS] " << name << '\n';
}

bool Near(double a, double b, double relative = 1e-6)
{
    return std::fabs(a - b) <= 1e-12 + relative * std::fabs(b);
}

template <typename Function>
void ExpectThrows(const std::string& name, Function function)
{
    bool threw = false;
    try { function(); }
    catch (const std::exception&) { threw = true; }
    Expect(name, threw);
}
}

void RunOceanSlopeVarianceAcceptanceTests()
{
    constexpr double pi = std::numbers::pi_v<double>;
    const OceanCartesianSpectrum constant = [](double, double) { return 0.25; };
    const auto coarse = IntegrateSlopeVariance(constant, 1.0, 4.0, 16, 32);
    const auto fine = IntegrateSlopeVariance(constant, 1.0, 4.0, 1024, 64);
    const double analytic = pi * 0.25 * (std::pow(4.0, 4) - 1.0) / 4.0;
    Expect("slope variance constant PSD analytic annulus",
        Near(fine.varianceX, analytic, 2e-6) &&
        Near(fine.varianceZ, analytic, 2e-6) &&
        Near(fine.covarianceXZ, 0.0));
    Expect("slope variance quadrature converges",
        std::fabs(fine.varianceX - analytic) <
        std::fabs(coarse.varianceX - analytic) / 100.0);

    // Psi = (1 + 0.4 cos(2 theta) + 0.2 sin(2 theta)) / k^4.
    // Its radial integral is log(kMax/kMin); angular moments are analytic.
    const OceanCartesianSpectrum directional = [](double x, double z)
    {
        const double k2 = x * x + z * z;
        return (1.0 + 0.4 * (x * x - z * z) / k2 +
            0.4 * x * z / k2) / (k2 * k2);
    };
    const auto anisotropic = IntegrateSlopeVariance(directional, 1.0, 16.0, 64, 64);
    const double scale = pi * std::log(16.0);
    Expect("slope variance directional moments and covariance",
        Near(anisotropic.varianceX, 1.2 * scale) &&
        Near(anisotropic.varianceZ, 0.8 * scale) &&
        Near(anisotropic.covarianceXZ, 0.1 * scale));
    Expect("slope variance exposes RMS not mean square",
        Near(anisotropic.RmsX(), std::sqrt(1.2 * scale)) &&
        Near(anisotropic.RmsZ(), std::sqrt(0.8 * scale)));
    const auto lower = IntegrateSlopeVariance(directional, 1.0, 4.0);
    const auto upper = IntegrateSlopeVariance(directional, 4.0, 16.0);
    Expect("slope variance resolved and unresolved bands add",
        Near(lower.varianceX + upper.varianceX, anisotropic.varianceX) &&
        Near(lower.varianceZ + upper.varianceZ, anisotropic.varianceZ) &&
        Near(lower.covarianceXZ + upper.covarianceXZ, anisotropic.covarianceXZ));

    OceanFrequencyConfig config;
    config.resolution = 8;
    config.patchLength = 2.0f * std::numbers::pi_v<float>;
    OceanFrequencyField frequency(config,
        [](float, float) { return 0.0f; }, [](float) { return 0.0f; });
    OceanHeightField fullCutoff(frequency);
    OceanHeightField halfCutoff(frequency, 0.5f);
    const auto fullTail = IntegrateUnresolvedSlopeVariance(directional, fullCutoff, 16.0);
    const auto halfTail = IntegrateUnresolvedSlopeVariance(directional, halfCutoff, 16.0);
    Expect("slope variance shares circular cutoff with height field",
        Near(fullTail.varianceX, 1.2 * pi * std::log(16.0 / fullCutoff.CutoffWaveNumber())) &&
        Near(halfTail.varianceX, 1.2 * pi * std::log(16.0 / halfCutoff.CutoffWaveNumber())) &&
        halfTail.TotalVariance() > fullTail.TotalVariance());

    ElfouhailyConfig seaConfig;
    ElfouhailySpectrum sea(seaConfig);
    const OceanCartesianSpectrum seaPsd = [&sea](double x, double z)
    {
        return sea.CartesianSpectrum(static_cast<float>(x), static_cast<float>(z));
    };
    // 10000 rad/m is only this finite-band test's explicit upper bound.
    const auto seaTail = IntegrateUnresolvedSlopeVariance(seaPsd, fullCutoff, 10000.0);
    const auto seaFine = IntegrateUnresolvedSlopeVariance(seaPsd, fullCutoff, 10000.0, 1024, 256);
    Expect("Elfouhaily unresolved slope variance finite and converged",
        seaFine.varianceX > 0.0 && seaFine.varianceZ > 0.0 &&
        Near(seaTail.varianceX, seaFine.varianceX, 5e-4) &&
        Near(seaTail.varianceZ, seaFine.varianceZ, 5e-4));
    seaConfig.windDirection = std::numbers::pi_v<float> / 4.0f;
    ElfouhailySpectrum rotatedSea(seaConfig);
    const auto rotated = IntegrateUnresolvedSlopeVariance(
        [&rotatedSea](double x, double z)
        {
            return rotatedSea.CartesianSpectrum(static_cast<float>(x), static_cast<float>(z));
        }, fullCutoff, 10000.0);
    Expect("Elfouhaily wind rotation rotates slope covariance",
        Near(rotated.varianceX, seaTail.TotalVariance() / 2.0, 1e-5) &&
        Near(rotated.varianceZ, seaTail.TotalVariance() / 2.0, 1e-5) &&
        Near(rotated.covarianceXZ, (seaTail.varianceX - seaTail.varianceZ) / 2.0, 1e-5));

    const auto zero = IntegrateSlopeVariance([](double, double) { return 0.0; }, 1.0, 10.0);
    const auto empty = IntegrateSlopeVariance(constant, 4.0, 4.0);
    Expect("slope variance zero PSD and empty band",
        zero.TotalVariance() == 0.0 && zero.covarianceXZ == 0.0 &&
        empty.TotalVariance() == 0.0);
    ExpectThrows("slope variance rejects zero lower bound",
        [&] { IntegrateSlopeVariance(constant, 0.0, 4.0); });
    ExpectThrows("slope variance rejects reversed bounds",
        [&] { IntegrateSlopeVariance(constant, 4.0, 1.0); });
    ExpectThrows("slope variance rejects nonfinite bound",
        [&] { IntegrateSlopeVariance(constant, 1.0, std::numeric_limits<double>::infinity()); });
    ExpectThrows("slope variance rejects invalid sample counts",
        [&] { IntegrateSlopeVariance(constant, 1.0, 4.0, 0); });
    ExpectThrows("slope variance rejects missing PSD",
        [] { IntegrateSlopeVariance({}, 1.0, 4.0); });
    ExpectThrows("slope variance rejects negative PSD",
        [] { IntegrateSlopeVariance([](double, double) { return -1.0; }, 1.0, 4.0); });
    ExpectThrows("slope variance rejects nonfinite PSD",
        [] { IntegrateSlopeVariance([](double, double) {
            return std::numeric_limits<double>::quiet_NaN(); }, 1.0, 4.0); });
}
