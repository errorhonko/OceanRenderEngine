#include "OceanFFT.h"

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

void OceanFFT::Transform1D(
    std::vector<std::complex<float>>& values,
    bool inverse)
{
    const std::size_t n =
        values.size();

    if (n == 0 ||
        (n & (n - 1)) != 0)
    {
        throw std::invalid_argument(
            "FFT size must be a power of two.");
    }

    // 位反转排列
    for (std::size_t i = 1, j = 0;
        i < n;
        ++i)
    {
        std::size_t bit = n >> 1;

        while (j & bit)
        {
            j ^= bit;
            bit >>= 1;
        }

        j ^= bit;

        if (i < j)
        {
            std::swap(
                values[i],
                values[j]);
        }
    }

    // 蝶形运算
    for (std::size_t length = 2;
        length <= n;
        length <<= 1)
    {
        const float sign =
            inverse ? 1.0f : -1.0f;

        const float angle =
            sign *
            2.0f *
            std::numbers::pi_v<float> /
            static_cast<float>(length);

        const std::complex<float> root(
            std::cos(angle),
            std::sin(angle));

        for (std::size_t begin = 0;
            begin < n;
            begin += length)
        {
            std::complex<float> factor(
                1.0f,
                0.0f);

            const std::size_t halfLength =
                length / 2;

            for (std::size_t offset = 0;
                offset < halfLength;
                ++offset)
            {
                const std::complex<float> even =
                    values[begin + offset];

                const std::complex<float> odd =
                    values[
                        begin +
                            offset +
                            halfLength] *
                    factor;

                        values[begin + offset] =
                            even + odd;

                        values[
                            begin +
                                offset +
                                halfLength] =
                            even - odd;

                            factor *= root;
            }
        }
    }

    // 标准 IFFT 归一化
    if (inverse)
    {
        const float inverseN =
            1.0f /
            static_cast<float>(n);

        for (std::complex<float>& value :
            values)
        {
            value *= inverseN;
        }
    }
}

void OceanFFT::Inverse2D(
    std::vector<std::complex<float>>& values,
    int resolution)
{
    if (resolution <= 0 ||
        (resolution &
            (resolution - 1)) != 0)
    {
        throw std::invalid_argument(
            "FFT resolution must be "
            "a positive power of two.");
    }

    const std::size_t n =
        static_cast<std::size_t>(
            resolution);

    if (values.size() != n * n)
    {
        throw std::invalid_argument(
            "FFT input size does not match "
            "the resolution.");
    }

    std::vector<std::complex<float>> line(n);

    // 对所有行做一维 IFFT
    for (std::size_t z = 0;
        z < n;
        ++z)
    {
        for (std::size_t x = 0;
            x < n;
            ++x)
        {
            line[x] =
                values[z * n + x];
        }

        Transform1D(line, true);

        for (std::size_t x = 0;
            x < n;
            ++x)
        {
            values[z * n + x] =
                line[x];
        }
    }

    // 对所有列做一维 IFFT
    for (std::size_t x = 0;
        x < n;
        ++x)
    {
        for (std::size_t z = 0;
            z < n;
            ++z)
        {
            line[z] =
                values[z * n + x];
        }

        Transform1D(line, true);

        for (std::size_t z = 0;
            z < n;
            ++z)
        {
            values[z * n + x] =
                line[z];
        }
    }
}