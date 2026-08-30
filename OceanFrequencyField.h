#pragma once
#include <complex>
#include <cstdint>
#include <functional>
#include <vector>
#include <cstddef>
struct OceanFrequencyConfig
{
    int resolution = 256;
    float patchLength = 100.0f;
    std::uint32_t seed = 42;
};



class OceanFrequencyField
{
public:
    using SpectrumFunction =
        std::function<float(float kx, float kz)>;
    
    using DispersionFunction =
        std::function<float(float k)>;

    OceanFrequencyField(
        const OceanFrequencyConfig& config,
        const SpectrumFunction& spectrum,
        const DispersionFunction& dispersion);

    int Resolution() const
    {
        return config.resolution;
    }

    float PatchLength() const
    {
        return config.patchLength;
    }

    float DeltaK() const
    {
        return deltaK;
    }

  

    // FFT 数组下标对应的实际波数
    float WaveNumber(int index) const;

    const std::complex<float>& H0(
        int x,
        int z) const;

    std::complex<float> H(
        int x,
        int z,
        float time) const;
    const std::vector<std::complex<float>>&
        InitialAmplitudes() const
    {
        return h0;
    }

    void BuildSpectrumAtTime(
        float time,
        std::vector<std::complex<float>>& output) const;
private:

    void Initialize(
        const SpectrumFunction& spectrum,
        const DispersionFunction& dispersion);

    int MirrorIndex(int index) const;

    std::vector<float> angularFrequencies;

    std::size_t Index(int x, int z) const;


    OceanFrequencyConfig config;
    float deltaK = 0.0f;

    std::vector<std::complex<float>> h0;
};