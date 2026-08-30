#include "ElfouhailySpectrum.h"

ElfouhailySpectrum::ElfouhailySpectrum(const ElfouhailyConfig& config)
    : config(config)
{
}

float ElfouhailySpectrum::PhaseSpeed(float k) const
{
    if (k <= 0.0f)
        return 0.0f;

    const float g = config.gravity;
    const float sigma = config.surfaceTension;
    const float rho = config.waterDensity;

    return std::sqrt(g / k + sigma * k / rho);
}

float ElfouhailySpectrum::LongWaveCurvature(float k) const
{
    if (k <= 0.0f)
        return 0.0f;

    const float omega = config.inverseWaveAge;
    const float u10 = config.windSpeed10m;

    // Elfouhaily 模型使用的适用范围
    if (u10 <= 0.0f || omega < 0.84f || omega > 5.0f)
        return 0.0f;

    // omega = U10 / cp
    const float cp = u10 / omega;

    // 重力波谱峰对应的波数
    const float kp =
        config.gravity / (cp * cp);

    // 谱峰幅度参数
    const float alphaP =
        0.006f * std::sqrt(omega);

    // Pierson-Moskowitz 低波数截止项
    const float kpOverK = kp / k;
    const float Lpm =
        std::exp(-1.25f * kpOverK * kpOverK);

    // JONSWAP 谱峰宽度
    const float omega3 =
        omega * omega * omega;

    const float peakWidth =
        0.08f * (1.0f + 4.0f / omega3);

    const float sqrtRatio =
        std::sqrt(k / kp);

    const float peakExponent =
        std::exp(
            -((sqrtRatio - 1.0f) *
                (sqrtRatio - 1.0f)) /
            (2.0f * peakWidth * peakWidth));

    const float gamma =
        omega <= 1.0f
        ? 1.7f
        : 1.7f + 6.0f * std::log(omega);

    const float Jp =
        std::pow(gamma, peakExponent);

    // 有限波龄修正
    const float waveAgeCorrection =
        std::exp(
            -(omega / std::sqrt(10.0f)) *
            (sqrtRatio - 1.0f));

    const float Fp =
        Lpm * Jp * waveAgeCorrection;

    const float c = PhaseSpeed(k);

    if (c <= 0.0f)
        return 0.0f;

    return 0.5f * alphaP * (cp / c) * Fp;
}

float ElfouhailySpectrum::ShortWaveCurvature(float k) const
{
     if (k <= 0.0f)
        return 0.0f;

    const float g = config.gravity;
    const float rho = config.waterDensity;
    const float surfaceTension =
        config.surfaceTension;
    const float uStar =
        config.frictionVelocity;

    if (g <= 0.0f ||
        rho <= 0.0f ||
        surfaceTension <= 0.0f ||
        uStar <= 0.0f)
    {
        return 0.0f;
    }

    // 重力波与毛细波贡献相等的位置
    const float km =
        std::sqrt(rho * g / surfaceTension);

    const float cm = PhaseSpeed(km);
    const float c = PhaseSpeed(k);

    if (cm <= 0.0f || c <= 0.0f)
        return 0.0f;

    const float frictionRatio =
        uStar / cm;

    float alphaM = 0.0f;

    if (uStar <= cm)
    {
        alphaM =
            0.01f *
            (1.0f + std::log(frictionRatio));
    }
    else
    {
        alphaM =
            0.01f *
            (1.0f +
             3.0f * std::log(frictionRatio));
    }

    // 超出经验模型适用范围时，不能产生负功率谱
    if (alphaM <= 0.0f ||
        !std::isfinite(alphaM))
    {
        return 0.0f;
    }

    const float normalizedK =
        k / km - 1.0f;

    const float Fm =
        std::exp(
            -0.25f *
            normalizedK *
            normalizedK);

    return
        0.5f *
        alphaM *
        (cm / c) *
        Fm;
}

float ElfouhailySpectrum::Omnidirectional(float k) const
{
    if (k <= 0.0f)
        return 0.0f;

    return (
        LongWaveCurvature(k) +
        ShortWaveCurvature(k)
        ) / (k * k * k);
}

float ElfouhailySpectrum::CartesianSpectrum(float kx, float kz) const
{
    float k = std::sqrt(kx * kx + kz * kz);

    if (k <= 1e-6f)
        return 0.0f;

    float phi =
        std::atan2(kz, kx) -
        config.windDirection;

    return Omnidirectional(k) *
        Spreading(k, phi) / k;
}

float ElfouhailySpectrum::Spreading(
    float k,
    float phi) const
{
    if (k <= 0.0f)
        return 0.0f;

    const float g = config.gravity;
    const float rho = config.waterDensity;
    const float surfaceTension =
        config.surfaceTension;
    const float u10 =
        config.windSpeed10m;
    const float uStar =
        config.frictionVelocity;
    const float omega =
        config.inverseWaveAge;

    if (g <= 0.0f ||
        rho <= 0.0f ||
        surfaceTension <= 0.0f ||
        u10 <= 0.0f ||
        uStar <= 0.0f ||
        omega < 0.84f ||
        omega > 5.0f)
    {
        return 0.0f;
    }

    // Omega = U10 / cp
    const float cp =
        u10 / omega;

    // 重力-毛细波最小相速度对应的波数
    const float km =
        std::sqrt(
            rho * g / surfaceTension);

    const float c =
        PhaseSpeed(k);

    const float cm =
        PhaseSpeed(km);

    if (c <= 0.0f ||
        cm <= 0.0f)
    {
        return 0.0f;
    }

    const float a0 =
        std::log(2.0f) / 4.0f;

    const float aP = 4.0f;

    const float aM =
        0.13f * uStar / cm;

    const float longWaveRatio =
        c / cp;

    const float shortWaveRatio =
        cm / c;

    const float delta =
        std::tanh(
            a0 +
            aP * std::pow(longWaveRatio, 2.5f) +
            aM * std::pow(shortWaveRatio, 2.5f));

    const float inverseTwoPi =
        1.0f /
        (2.0f * std::numbers::pi_v<float>);

    return inverseTwoPi *
        (1.0f +
            delta * std::cos(2.0f * phi));
}