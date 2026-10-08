#include "OceanHeightField.h"
#include "OceanFFT.h"
#include "OceanFrequencyField.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

OceanHeightField::OceanHeightField(
    const OceanFrequencyField& frequencyField,
    float cutoffFraction)
    : frequencyField(frequencyField),
    resolution(frequencyField.Resolution())
{
    if (resolution <= 0 ||
        (resolution &
            (resolution - 1)) != 0)
    {
        throw std::invalid_argument(
            "Ocean height field resolution "
            "must be a positive power of two.");
    }

    if (!std::isfinite(cutoffFraction) ||
        cutoffFraction <= 0.0f ||
        cutoffFraction > 1.0f)
    {
        throw std::invalid_argument(
            "Ocean height field cutoff fraction "
            "must be in (0, 1].");
    }

    const float nyquistWaveNumber =
        frequencyField.NyquistWaveNumber();
    if (!std::isfinite(nyquistWaveNumber) ||
        nyquistWaveNumber <= 0.0f)
    {
        throw std::invalid_argument(
            "Ocean height field Nyquist wave number "
            "must be finite and positive.");
    }
    cutoffWaveNumber =
        cutoffFraction * nyquistWaveNumber;

    const std::size_t n =
        static_cast<std::size_t>(
            resolution);

    const std::size_t elementCount =
        n * n;

    frequencyBuffer.resize(elementCount);

    heights.resize(
        elementCount,
        0.0f);

    slopeXBuffer.resize(elementCount);
    slopeZBuffer.resize(elementCount);

    slopesX.resize(elementCount, 0.0f);
    slopesZ.resize(elementCount, 0.0f);
}

std::size_t OceanHeightField::Index(
    int x,
    int z) const
{
    return
        static_cast<std::size_t>(z) *
        static_cast<std::size_t>(
            resolution) +
        static_cast<std::size_t>(x);
}

void OceanHeightField::Update(
    float time)
{
    if (!std::isfinite(time))
    {
        throw std::invalid_argument(
            "Ocean height field time "
            "must be finite.");
    }

    // 生成当前时间的完整频域数组 H(k,t)。
    frequencyField.BuildSpectrumAtTime(
        time,
        frequencyBuffer);

    // Keep the same radial cutoff in every direction.
    // The boundary and outer modes belong to the unresolved band.
    const float cutoffSquared =
        cutoffWaveNumber * cutoffWaveNumber;
    for (int z = 0; z < resolution; ++z)
    {
        const float kz = frequencyField.WaveNumber(z);
        for (int x = 0; x < resolution; ++x)
        {
            const float kx = frequencyField.WaveNumber(x);
            if (kx * kx + kz * kz >= cutoffSquared)
                frequencyBuffer[Index(x, z)] = {0.0f, 0.0f};
        }
    }

    for (int z = 0; z < resolution; ++z)
    {
        // 奈奎斯特模式的采样点坡度存在歧义，暂设为零。
        const float kz =
            z == resolution / 2
            ? 0.0f
            : frequencyField.WaveNumber(z);

        for (int x = 0; x < resolution; ++x)
        {
            const float kx =
                x == resolution / 2
                ? 0.0f
                : frequencyField.WaveNumber(x);

            const std::size_t index = Index(x, z);
            const std::complex<float> h =
                frequencyBuffer[index];

            slopeXBuffer[index] =
                std::complex<float>(0.0f, kx) * h;

            slopeZBuffer[index] =
                std::complex<float>(0.0f, kz) * h;
        }
    }
  

    // 原地转换为空间域。
    OceanFFT::Inverse2D(frequencyBuffer, resolution);
    OceanFFT::Inverse2D(slopeXBuffer, resolution);
    OceanFFT::Inverse2D(slopeZBuffer, resolution);

    const float physicalScale =
        static_cast<float>(resolution) *
        static_cast<float>(resolution);

    maxImaginaryResidual = 0.0f;

    for (std::size_t index = 0;
        index < frequencyBuffer.size();
        ++index)
    {
        const std::complex<float>& value =
            frequencyBuffer[index];

        const float imaginaryResidual =
            std::fabs(value.imag()) *
            physicalScale;

        maxImaginaryResidual =
            std::max(
                maxImaginaryResidual,
                imaginaryResidual);

        // 标准二维 IFFT 除以了 N²，
        // 这里乘回物理傅里叶级数尺度。
        heights[index] =
            frequencyBuffer[index].real() *
            physicalScale;

        slopesX[index] =
            slopeXBuffer[index].real() *
            physicalScale;

        slopesZ[index] =
            slopeZBuffer[index].real() *
            physicalScale;
    }
}

float OceanHeightField::Height(
    int x,
    int z) const
{
    if (x < 0 ||
        x >= resolution ||
        z < 0 ||
        z >= resolution)
    {
        throw std::out_of_range(
            "Ocean height index "
            "is out of range.");
    }

    return heights[Index(x, z)];
}

float OceanHeightField::SlopeX(int x, int z) const
{
    if (x < 0 || x >= resolution ||
        z < 0 || z >= resolution)
    {
        throw std::out_of_range(
            "Ocean slope index is out of range.");
    }

    return slopesX[Index(x, z)];
}

float OceanHeightField::SlopeZ(int x, int z) const
{
    if (x < 0 || x >= resolution ||
        z < 0 || z >= resolution)
    {
        throw std::out_of_range(
            "Ocean slope index is out of range.");
    }

    return slopesZ[Index(x, z)];
}
