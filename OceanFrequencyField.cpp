#include "OceanFrequencyField.h"

#include <cmath>
#include <numbers>
#include <random>
#include <stdexcept>

OceanFrequencyField::OceanFrequencyField(
    const OceanFrequencyConfig& config,
    const SpectrumFunction& spectrum,
    const DispersionFunction& dispersion)
    : config(config)
{
    if (config.resolution < 2)
    {
        throw std::invalid_argument(
            "Ocean resolution must be at least 2.");
    }

    if (config.patchLength <= 0.0f)
    {
        throw std::invalid_argument(
            "Ocean patch length must be positive.");
    }

    if (!spectrum)
    {
        throw std::invalid_argument(
            "Ocean spectrum function is empty.");
    }

    if (!dispersion)
    {
        throw std::invalid_argument(
            "Ocean dispersion function is empty.");
    }

    deltaK =
        2.0f * std::numbers::pi_v<float> /
        config.patchLength;

    const std::size_t elementCount =
        static_cast<std::size_t>(
            config.resolution) *
        static_cast<std::size_t>(
            config.resolution);

    h0.resize(elementCount);
    angularFrequencies.resize(elementCount);


    Initialize(spectrum, dispersion);
}

std::size_t OceanFrequencyField::Index(
    int x,
    int z) const
{
    return static_cast<std::size_t>(z) *
        static_cast<std::size_t>(config.resolution) +
        static_cast<std::size_t>(x);
}

float OceanFrequencyField::WaveNumber(
    int index) const
{
    const int n = config.resolution;

    const int signedIndex =
        index <= n / 2
        ? index
        : index - n;

    return static_cast<float>(signedIndex) *
        deltaK;
}

const std::complex<float>&
OceanFrequencyField::H0(
    int x,
    int z) const
{
    if (x < 0 ||
        x >= config.resolution ||
        z < 0 ||
        z >= config.resolution)
    {
        throw std::out_of_range(
            "Ocean frequency index is out of range.");
    }
    return h0[Index(x, z)];
}

std::complex<float> OceanFrequencyField::H(int x, int z, float time) const
{
    if (!std::isfinite(time))
    {
        throw std::invalid_argument(
            "Ocean time must be finite.");
    }

    // H0() 同时完成 x、z 的边界检查。
    const std::complex<float>& h0K =
        H0(x, z);

    const int mirrorX =
        MirrorIndex(x);

    const int mirrorZ =
        MirrorIndex(z);

    const std::complex<float>& h0MinusK =
        H0(mirrorX, mirrorZ);

    const float omega =
        angularFrequencies[Index(x, z)];

    const float phase =
        omega * time;

    const std::complex<float> positivePhase =
        std::polar(
            1.0f,
            phase);

    const std::complex<float> negativePhase =
        std::conj(positivePhase);

    return
        h0K * positivePhase +
        std::conj(h0MinusK) *
        negativePhase;
}

void OceanFrequencyField::BuildSpectrumAtTime(float time, std::vector<std::complex<float>>& output) const
{
    if (!std::isfinite(time))
    {
        throw std::invalid_argument(
            "Ocean time must be finite.");
    }

    output.resize(h0.size());

    for (int z = 0;
        z < config.resolution;
        ++z)
    {
        for (int x = 0;
            x < config.resolution;
            ++x)
        {
            output[Index(x, z)] =
                H(x, z, time);
        }
    }
}

void OceanFrequencyField::Initialize(
    const SpectrumFunction& spectrum,
    const DispersionFunction& dispersion)
{
    std::mt19937 generator(config.seed);

    std::normal_distribution<float> gaussian(
        0.0f,
        1.0f);

    const float deltaArea = deltaK * deltaK;

    for (int z = 0;
         z < config.resolution;
         ++z)
    {
        const float kz = WaveNumber(z);

        for (int x = 0;
             x < config.resolution;
             ++x)
        {
            const float kx = WaveNumber(x);

            const std::size_t index =
                Index(x, z);

            const float k =
                std::sqrt(
                    kx * kx +
                    kz * kz);
            // 零频率代表整个海面的平均高度。
            // 设为 0，避免海面整体上下平移。
            if (k == 0.0f)
            {
                angularFrequencies[index] =
                    0.0f;

                h0[index] =
                    std::complex<float>(
                        0.0f,
                        0.0f);

                continue;
            }
            // 色散关系决定该波数的传播速度。
            const float omega =
                dispersion(k);

            if (!std::isfinite(omega) ||
                omega < 0.0f)
            {
                throw std::runtime_error(
                    "Ocean dispersion returned "
                    "an invalid value.");
            }

            angularFrequencies[index] =
                omega;

            const float psi = spectrum(kx, kz);

            if (!std::isfinite(psi) || psi < 0.0f)
            {
                throw std::runtime_error(
                    "Ocean spectrum returned an invalid value.");
            }

            const float scale =
                std::sqrt(
                    0.25f * psi * deltaArea);

            const float realPart =
                gaussian(generator) * scale;

            const float imaginaryPart =
                gaussian(generator) * scale;

            h0[index] =
                std::complex<float>(
                    realPart,
                    imaginaryPart);
        }
    }
}

int OceanFrequencyField::MirrorIndex(int index) const
{
    return
        (config.resolution - index) %
        config.resolution;
}
