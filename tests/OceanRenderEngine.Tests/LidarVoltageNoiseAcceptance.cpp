#include "LidarVoltageNoise.h"
#include "LidarRangeCalibration.h"
#include "FirstOrderLidarResponse.h"
#include "GaussianPulseProfile.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void Check(const char* name, bool ok) {
    if (!ok) throw std::runtime_error(std::string("[FAIL] ") + name);
    std::cout << "[PASS] " << name << '\n';
}
template<class E, class F> bool Throws(F f) {
    try { f(); } catch (const E&) { return true; }
    return false;
}
}

void RunLidarVoltageNoiseAcceptanceTests() {
    constexpr double sigma = 1e-3;
    const LidarVoltageNoise noise(sigma), zeroNoise(0);
    LidarVoltageWaveform input({10e-9, 2e-9, 5}, {-.5, 0, .25, .5, 1});
    const auto before = input.AverageVoltageBinsV();
    const auto first = noise.Apply(input, 42);
    const auto repeat = noise.Apply(input, 42);
    const auto different = noise.Apply(input, 43);
    Check("voltage noise stores sigma in volts", noise.SigmaVoltageV() == sigma);
    Check("voltage noise zero sigma exact identity", zeroNoise.Apply(input, 42).AverageVoltageBinsV() == before);
    Check("voltage noise zero sigma independent of seed", zeroNoise.Apply(input, 0).AverageVoltageBinsV() == zeroNoise.Apply(input, 98765).AverageVoltageBinsV());
    Check("voltage noise fixed seed repeats exactly", first.AverageVoltageBinsV() == repeat.AverageVoltageBinsV());
    Check("voltage noise changed seed changes realization", first.AverageVoltageBinsV() != different.AverageVoltageBinsV());
    Check("voltage noise apply restarts local random state", first.AverageVoltageBinsV() == noise.Apply(input, 42).AverageVoltageBinsV());
    Check("voltage noise input remains unchanged", input.AverageVoltageBinsV() == before);
    Check("voltage noise time configuration retained", first.Config().startTimeSeconds == 10e-9 && first.Config().binWidthSeconds == 2e-9 && first.Config().binCount == 5);
    bool times = true;
    for (std::size_t i = 0; i < before.size(); ++i) times = times && first.BinCenterTimeSeconds(i) == input.BinCenterTimeSeconds(i);
    Check("voltage noise every bin center retained", times);
    input = LidarVoltageWaveform({0, 1, 1}, {0});
    Check("voltage noise output owns independent snapshot", first.AverageVoltageBinsV().size() == 5 && first.AverageVoltageBinsV() == repeat.AverageVoltageBinsV());
    const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
    Check("voltage noise rejects negative sigma", Throws<std::invalid_argument>([]{ LidarVoltageNoise bad(-1); }));
    Check("voltage noise rejects NaN sigma", Throws<std::invalid_argument>([&]{ LidarVoltageNoise bad(nan); }));
    Check("voltage noise rejects infinite sigma", Throws<std::invalid_argument>([&]{ LidarVoltageNoise bad(inf); }));
    const auto single = noise.Apply(LidarVoltageWaveform({-1, .5, 1}, {0}), 0);
    Check("voltage noise single bin and zero seed valid", single.AverageVoltageBinsV().size() == 1 && std::isfinite(single.AverageVoltageBinsV()[0]) && single.BinCenterTimeSeconds(0) == -.75);
    const auto highSeed = noise.Apply(LidarVoltageWaveform({0, 1, 5}, {0,0,0,0,0}), std::uint64_t{1} << 40);
    const auto lowSeed = noise.Apply(LidarVoltageWaveform({0, 1, 5}, {0,0,0,0,0}), 0);
    Check("voltage noise retains high seed bits", highSeed.AverageVoltageBinsV() != lowSeed.AverageVoltageBinsV());

    // Fixed seed and 200000 bins: statistical tolerances, not exact moment claims.
    constexpr std::size_t count = 200000;
    const LidarVoltageWaveform zeros({0, 1e-9, count}, std::vector<double>(count, 0));
    const auto sampled = noise.Apply(zeros, 20261006);
    const auto& samples = sampled.AverageVoltageBinsV();
    long double sum = 0, sumSquares = 0, adjacent = 0;
    std::size_t positive = 0, negative = 0;
    bool finite = true;
    for (std::size_t i = 0; i < count; ++i) {
        const long double v = samples[i];
        finite = finite && std::isfinite(samples[i]);
        sum += v;
        sumSquares += v * v;
        positive += v > 0;
        negative += v < 0;
        if (i) adjacent += v * samples[i-1];
    }
    const long double mean = sum / count;
    const long double variance = (sumSquares - sum*sum/count) / (count-1);
    const long double lagOne = (adjacent/(count-1) - mean*mean) / variance;
    Check("voltage noise large sample finite", finite);
    Check("voltage noise empirical mean within six standard errors", std::abs(mean) < 6*sigma/std::sqrt(double(count)));
    Check("voltage noise empirical variance within statistical tolerance", std::abs(variance/(sigma*sigma)-1) < 6*std::sqrt(2.0/(count-1)));
    Check("voltage noise retains both negative and positive voltages", positive > 0 && negative > 0);
    Check("voltage noise sign fractions approximately balanced", std::abs(double(positive)/count-.5) < .01 && std::abs(double(negative)/count-.5) < .01);
    Check("voltage noise adjacent bins have small sample correlation", std::abs(lagOne) < .02);
    const auto doubled = LidarVoltageNoise(2*sigma).Apply(zeros, 20261006);
    bool scales = true;
    for (std::size_t i = 0; i < count; ++i) scales = scales && doubled.AverageVoltageBinsV()[i] == 2*samples[i];
    Check("voltage noise sigma scales same realization", scales);
    const auto shifted = noise.Apply(LidarVoltageWaveform({0, 1e-9, count}, std::vector<double>(count, .25)), 20261006);
    bool adds = true;
    for (std::size_t i = 0; i < count; ++i) adds = adds && shifted.AverageVoltageBinsV()[i] == .25 + samples[i];
    Check("voltage noise adds rather than replaces input", adds);
    const double maximum = std::numeric_limits<double>::max();
    Check("voltage noise detects noise multiplication overflow", Throws<std::overflow_error>([&]{ LidarVoltageNoise(maximum).Apply(LidarVoltageWaveform({0, 1, 128}, std::vector<double>(128, 0)), 42); }));
    const LidarVoltageWaveform huge({0, 1, 128}, std::vector<double>(128, maximum));
    Check("voltage noise detects voltage addition overflow", Throws<std::overflow_error>([&]{ LidarVoltageNoise(maximum/4).Apply(huge, 42); }));
    Check("voltage noise overflow leaves input unchanged", std::all_of(huge.AverageVoltageBinsV().begin(), huge.AverageVoltageBinsV().end(), [=](double v){ return v == maximum; }));
    // Negative analog noise is preserved; clipping is an ADC decision.
    const auto unipolarDigits = LidarDigitalWaveform::FromVoltageWaveform(sampled, IdealUniformAdc(12, 0, 1));
    const auto bipolarDigits = LidarDigitalWaveform::FromVoltageWaveform(sampled, IdealUniformAdc(12, -1, 1));
    Check("voltage noise unipolar ADC reports negative input clipping", unipolarDigits.HasOutOfRangeSamples());
    Check("voltage noise bipolar ADC accommodates this realization", !bipolarDigits.HasOutOfRangeSamples());
    bool adcMatches = true;
    const IdealUniformAdc adc(12, -1, 1);
    for (std::size_t i = 0; i < count; ++i) {
        const auto expected = adc.Quantize(samples[i]);
        const auto& actual = bipolarDigits.Samples()[i];
        adcMatches = adcMatches && actual.code == expected.code && actual.belowRange == expected.belowRange && actual.aboveRange == expected.aboveRange;
    }
    Check("voltage noise ADC chain every sample analytic", adcMatches);

    LidarWaveform optical({0, .25e-9, 1024});
    optical.AccumulateReturn({1e-12, 100.2e-9, 532}, GaussianPulseProfile(4e-9));
    const auto idealCurrent = LidarAnalogWaveform::FromOpticalWaveform(optical, LinearPhotodetector(532, .6));
    const auto received = FirstOrderLidarResponse(2e-9).Apply(idealCurrent);
    const auto voltage = LidarVoltageWaveform::FromCurrentWaveform(received, IdealTransimpedanceAmplifier(10e3));
    const auto noisy = noise.Apply(voltage, 42);
    const auto digits = LidarDigitalWaveform::FromVoltageWaveform(noisy, adc);
    const LidarDigitalRangeEstimator estimator(adc.Quantize(0).code, 4);
    const LidarReturnWindow window{90e-9, 125e-9};
    const auto peak = estimator.Estimate(digits, 0, window);
    const auto centroid = estimator.EstimateCentroid(digits, 0, window);
    Check("voltage noise full signal chain peak detected", peak && std::isfinite(peak->rangeMeters));
    Check("voltage noise full signal chain centroid detected", centroid && std::isfinite(centroid->rangeMeters));
    Check("voltage noise full chain stays within bipolar ADC range", !digits.HasOutOfRangeSamples());
    Check("voltage noise full chain estimates stay in selected window", peak && centroid && peak->arrivalTimeSeconds >= 90e-9 && peak->arrivalTimeSeconds < 125e-9 && centroid->arrivalTimeSeconds >= 90e-9 && centroid->arrivalTimeSeconds < 125e-9);
    const auto cleanDigits = LidarDigitalWaveform::FromVoltageWaveform(voltage, adc);
    const auto cleanReference = estimator.EstimateCentroid(cleanDigits, 0, window);
    Check("voltage noise clean calibration reference detected", cleanReference.has_value());
    if (!cleanReference) return;
    const auto calibration = LidarRangeCalibration::FromReferenceMeasurement(*cleanReference, .5*299792458.0*100.2e-9);
    const auto corrected = centroid ? calibration.Apply(*centroid) : std::nullopt;
    Check("voltage noise chain applies frozen clean calibration", corrected && std::isfinite(corrected->correctedRangeMeters));
    std::cout << "[INFO] voltage noise N=" << count << " mean V=" << double(mean)
        << " sigma V=" << std::sqrt(double(variance)) << " lag1=" << double(lagOne) << '\n';
}
