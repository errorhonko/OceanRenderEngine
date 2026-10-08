#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <utility>

template <std::floating_point Float>
inline Float FrDielectric(
    Float cosThetaI,
    Float etaI,
    Float etaT)
{
    cosThetaI = std::clamp(
        cosThetaI,
        Float(-1),
        Float(1));

    const bool entering =
        cosThetaI > Float(0);

    if (!entering)
    {
        std::swap(etaI, etaT);
        cosThetaI = std::abs(cosThetaI);
    }

    const Float sinThetaI =
        std::sqrt(std::max(
            Float(0),
            Float(1) -
            cosThetaI * cosThetaI));

    const Float sinThetaT =
        etaI / etaT * sinThetaI;

    // 从高折射率介质进入低折射率介质时可能全反射。
    if (sinThetaT >= Float(1))
        return Float(1);

    const Float cosThetaT =
        std::sqrt(std::max(
            Float(0),
            Float(1) -
            sinThetaT * sinThetaT));

    const Float rParallel =
        (etaT * cosThetaI -
            etaI * cosThetaT) /
        (etaT * cosThetaI +
            etaI * cosThetaT);

    const Float rPerpendicular =
        (etaI * cosThetaI -
            etaT * cosThetaT) /
        (etaI * cosThetaI +
            etaT * cosThetaT);

    return
        (rParallel * rParallel +
            rPerpendicular * rPerpendicular) /
        Float(2);
}