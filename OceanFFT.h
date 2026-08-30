#pragma once
#include <complex>
#include <vector>

class OceanFFT
{
public:
    static void Inverse2D(
        std::vector<std::complex<float>>& values,
        int resolution);

private:
    static void Transform1D(
        std::vector<std::complex<float>>& values,
        bool inverse);
};