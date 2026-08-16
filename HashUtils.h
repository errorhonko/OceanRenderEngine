#pragma once
#include "Vector3f.h"
#include <cstdint>
#include <cstring>

namespace HashUtils
{
    inline std::uint32_t FloatToBits(float value)
    {
        if (value == 0.0f)
            value = 0.0f;

        std::uint32_t bits;
        std::memcpy(&bits, &value, sizeof(float));
        return bits;
    }

    inline std::uint64_t MixBits(std::uint64_t value)
    {
        value ^= value >> 30;
        value *= 0xbf58476d1ce4e5b9ULL;

        value ^= value >> 27;
        value *= 0x94d049bb133111ebULL;

        value ^= value >> 31;

        return value;
    }
    inline std::uint64_t HashCombine(
        std::uint64_t hash,
        std::uint64_t value)
    {
        constexpr std::uint64_t goldenRatio =
            0x9e3779b97f4a7c15ULL;

        return MixBits(
            hash ^ MixBits(value + goldenRatio)
        );
    }

    inline float HashToFloat(std::uint64_t hash)
    {
        std::uint32_t value =
            static_cast<std::uint32_t>(hash >> 40);

        return value * (1.0f / 16777216.0f);
    }

    inline float HashFloat(
        const Vector3f& p,
        const Vector3f& direction,
        std::uint64_t seed = 0)
    {
        std::uint64_t hash =
            0x243f6a8885a308d3ULL;

        hash = HashCombine(hash, FloatToBits(p.x));
        hash = HashCombine(hash, FloatToBits(p.y));
        hash = HashCombine(hash, FloatToBits(p.z));

        hash = HashCombine(hash, FloatToBits(direction.x));
        hash = HashCombine(hash, FloatToBits(direction.y));
        hash = HashCombine(hash, FloatToBits(direction.z));

        hash = HashCombine(hash, seed);

        return HashToFloat(hash);
    }
}