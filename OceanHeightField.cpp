#include "OceanHeightField.h"
#include "OceanFFT.h"
#include "OceanFrequencyField.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

OceanHeightField::OceanHeightField(
    const OceanFrequencyField& frequencyField)
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

    const std::size_t n =
        static_cast<std::size_t>(
            resolution);

    const std::size_t elementCount =
        n * n;

    frequencyBuffer.resize(elementCount);

    heights.resize(
        elementCount,
        0.0f);
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

    // 原地转换为空间域。
    OceanFFT::Inverse2D(
        frequencyBuffer,
        resolution);

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
            value.real() *
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