#pragma once
#include <cmath>
#include <cstdint>
#include <numbers>
struct ElfouhailyConfig
{
    int resolution = 256;
    float patchLength = 100.0f;       // m
    float windSpeed10m = 10.0f;       // U10, m/s
    float windDirection = 0.0f;       // 相对 X 轴，rad

    // U10 / cp。完全发展海面通常先取 0.84。
    float inverseWaveAge = 0.84f;

    float gravity = 9.81f;
    float waterDensity = 1000.0f;
    float surfaceTension = 0.074f;

    std::uint32_t seed = 42;

    float frictionVelocity = 0.38f;// u*, m/s

};
class ElfouhailySpectrum
{
public:
    explicit ElfouhailySpectrum(
        const ElfouhailyConfig& config);

    float PhaseSpeed(float k) const;

    float LongWaveCurvature(float k) const;   // Bl
    float ShortWaveCurvature(float k) const;  // Bh

    float Omnidirectional(float k) const;     // S(k)
    float Spreading(float k, float phi) const;// Phi(k, phi)

    float CartesianSpectrum(
        float kx,
        float kz) const;                      // Psi(kx, kz)

private:
    ElfouhailyConfig config;
};

