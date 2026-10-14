#include "lidar/signal_processing/LidarDigitalMultiEchoEstimator.h"
#include "lidar/signal_processing/LidarRangeCalibration.h"
#include "lidar/receiver/analog/FirstOrderLidarResponse.h"
#include "lidar/emitter/pulse/GaussianPulseProfile.h"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void Check(const char* name, bool ok)
{
    if (!ok) throw std::runtime_error(std::string("[FAIL] ") + name);
    std::cout << "[PASS] " << name << '\n';
}
bool Near(double a, double b, double tolerance = 1e-18) { return std::abs(a - b) <= tolerance; }
template<class E, class F> bool Throws(F f)
{
    try { f(); } catch (const E&) { return true; } return false;
}
LidarDigitalWaveform Codes(const std::vector<std::uint32_t>& codes, double start = 0, double dt = 1e-9)
{
    const IdealUniformAdc adc(12, 0, 4096);
    std::vector<double> v;
    for (const auto code : codes) v.push_back(adc.CodeCenterVoltageV(code));
    return LidarDigitalWaveform::FromVoltageWaveform(LidarVoltageWaveform({start, dt, codes.size()}, v), adc);
}
}

void RunLidarDigitalMultiEchoAcceptanceTests()
{
    const LidarDigitalMultiEchoEstimator estimator({{0}, {100, 100}});
    const auto single = Codes({0, 2, 8, 2, 0});
    const auto one = estimator.Estimate(single, 0, {0, 5e-9}, 0, 0);
    Check("multi echo single peak local centroid analytic", one.size() == 1 && Near(one[0].measured.arrivalTimeSeconds, 2.5e-9));
    Check("multi echo raw range is c over two times centroid delay", one.size() == 1 && Near(one[0].measured.rangeMeters, .5 * 299792458.0 * 2.5e-9, 1e-12));
    const auto prior = LidarDigitalRangeEstimator().EstimateCentroid(single, 0, LidarReturnWindow{0, 5e-9});
    Check("multi echo isolated waveform agrees with original centroid", prior && one.size() == 1 && Near(one[0].measured.arrivalTimeSeconds, prior->arrivalTimeSeconds));
    const auto doubleWave = Codes({0, 2, 8, 2, 0, 1, 6, 3, 0});
    const auto two = estimator.Estimate(doubleWave, 0, {0, 9e-9}, 0, 0);
    Check("multi echo two detected peaks remain distinct measurements", two.size() == 2 && two[0].measured.peakBinIndex == 2 && two[1].measured.peakBinIndex == 6);
    Check("multi echo unique valley produces disjoint windows", two.size() == 2 && two[0].firstBinIndex == 0 && two[0].lastBinIndex == 4 && two[1].firstBinIndex == 5 && two[1].lastBinIndex == 8);
    Check("multi echo both centroids analytic without cross peak weights", two.size() == 2 && Near(two[0].measured.arrivalTimeSeconds, 2.5e-9) && Near(two[1].measured.arrivalTimeSeconds, 6.7e-9));
    Check("multi echo identity retained rather than repeated strongest peak", two.size() == 2 && two[0].measured.peakCode == 8 && two[1].measured.peakCode == 6);
    const auto combined = LidarDigitalRangeEstimator().EstimateCentroid(doubleWave, 0);
    Check("multi echo local centroids do not collapse into global centroid", combined && two.size() == 2 && combined->arrivalTimeSeconds > two[0].measured.arrivalTimeSeconds && combined->arrivalTimeSeconds < two[1].measured.arrivalTimeSeconds);
    const auto flat = estimator.Estimate(Codes({0, 9, 0, 0, 0, 0, 8, 0}), 0, {0, 8e-9}, 0, 0);
    Check("multi echo flat valley uses middle bin and earlier tie", flat.size() == 2 && flat[0].lastBinIndex == 3 && flat[1].firstBinIndex == 4);
    const auto minima = estimator.Estimate(Codes({0, 9, 0, 2, 0, 4, 8, 0}), 0, {0, 8e-9}, 0, 3);
    Check("multi echo separated equal valleys use exact half bin midpoint", minima.size() == 2 && minima[0].lastBinIndex == 4 && minima[1].firstBinIndex == 5);
    const auto triple = estimator.Estimate(Codes({0, 8, 0, 4, 0, 9, 0}), 0, {0, 7e-9}, 0, 0);
    Check("multi echo three windows ordered and pairwise disjoint", triple.size() == 3 && triple[0].lastBinIndex < triple[1].firstBinIndex && triple[1].lastBinIndex < triple[2].firstBinIndex && triple[0].measured.arrivalTimeSeconds < triple[1].measured.arrivalTimeSeconds && triple[1].measured.arrivalTimeSeconds < triple[2].measured.arrivalTimeSeconds);
    const LidarDigitalMultiEchoEstimator narrow({{0}, {0, 0}});
    const auto plateau = narrow.Estimate(Codes({0, 9, 9, 0}), 0, {0, 4e-9}, 0, 0);
    Check("multi echo zero radius still includes complete plateau", plateau.size() == 1 && plateau[0].firstBinIndex == 1 && plateau[0].lastBinIndex == 2 && Near(plateau[0].measured.arrivalTimeSeconds, 2e-9));
    const auto odd = narrow.Estimate(Codes({0, 9, 9, 9, 0}), 0, {0, 5e-9}, 0, 0);
    Check("multi echo odd plateau symmetric centroid", odd.size() == 1 && Near(odd[0].measured.arrivalTimeSeconds, 2.5e-9));
    const auto zeroRadius = narrow.Estimate(single, 0, {0, 5e-9}, 0, 0);
    Check("multi echo zero radius single bin uses peak center", zeroRadius.size() == 1 && zeroRadius[0].firstBinIndex == 2 && zeroRadius[0].lastBinIndex == 2);
    const auto asymmetric = LidarDigitalMultiEchoEstimator({{0}, {1, 0}}).Estimate(single, 0, {0, 5e-9}, 0, 0);
    Check("multi echo asymmetric neighborhood limits included tails", asymmetric.size() == 1 && asymmetric[0].firstBinIndex == 1 && asymmetric[0].lastBinIndex == 2 && Near(asymmetric[0].measured.arrivalTimeSeconds, 2.3e-9));
    const auto fractional = estimator.Estimate(Codes({100, 102, 108, 102, 100}), 0, {0, 5e-9}, 100.25, 7);
    Check("multi echo fractional baseline and threshold retained", fractional.size() == 1 && fractional[0].measured.peakAboveBaselineCodes == 7.75 && Near(fractional[0].measured.arrivalTimeSeconds, 2.5e-9));
    const auto scaled = estimator.Estimate(Codes({0, 4, 16, 4, 0, 2, 12, 6, 0}), 0, {0, 9e-9}, 0, 0);
    Check("multi echo amplitude scaling preserves both centroid times", scaled.size() == 2 && Near(scaled[0].measured.arrivalTimeSeconds, two[0].measured.arrivalTimeSeconds) && Near(scaled[1].measured.arrivalTimeSeconds, two[1].measured.arrivalTimeSeconds));
    const auto shifted = estimator.Estimate(Codes({0, 2, 8, 2, 0, 1, 6, 3, 0}, 1000), 1000, {1000, 1000 + 9e-9}, 0, 0);
    Check("multi echo relative local centroid stable under clock shift", shifted.size() == 2 && Near(shifted[0].measured.timeOfFlightSeconds, two[0].measured.timeOfFlightSeconds, 2e-13) && Near(shifted[1].measured.timeOfFlightSeconds, two[1].measured.timeOfFlightSeconds, 2e-13));
    Check("multi echo no candidate returns empty", estimator.Estimate(Codes({0, 0, 0}), 0, {0, 3e-9}, 0, 0).empty());
    Check("multi echo threshold equality rejects weaker echo", estimator.Estimate(doubleWave, 0, {0, 9e-9}, 0, 6).size() == 1);
    Check("multi echo minimum separation reused from detector", LidarDigitalMultiEchoEstimator({{10e-9}, {1, 1}}).Estimate(doubleWave, 0, {0, 9e-9}, 0, 0).size() == 1);
    const auto gated = estimator.Estimate(doubleWave, 4e-9, {4e-9, 9e-9}, 0, 0);
    Check("multi echo windows respect search and emission boundaries", gated.size() == 1 && gated[0].firstBinIndex == 4 && gated[0].lastBinIndex == 8 && Near(gated[0].measured.timeOfFlightSeconds, 2.7e-9));
    const auto calibration = LidarRangeCalibration(1e-9);
    const auto corrected = calibration.Apply(two[1].measured);
    Check("multi echo existing calibration applies externally to each raw estimate", corrected && Near(corrected->correctedTimeOfFlightSeconds, 5.7e-9) && Near(corrected->correctedRangeMeters, .5 * 299792458.0 * 5.7e-9, 1e-12));
    Check("multi echo invalid correction not clamped to zero", !LidarRangeCalibration(10e-9).Apply(two[0].measured));
    const IdealUniformAdc adc(12, 0, 4096);
    const auto clipped = LidarDigitalWaveform::FromVoltageWaveform(LidarVoltageWaveform({0, 1e-9, 8}, {.5, 4095.5, 5000, .5, .5, 9.5, .5, .5}), adc);
    const auto clipping = estimator.Estimate(clipped, 0, {0, 8e-9}, 0, 0);
    Check("multi echo plateau clipping preserved beyond representative bin", clipping.size() == 2 && clipping[0].peak.plateauOutOfRange && !clipping[0].measured.peakOutOfRange && clipping[0].localWindowHasOutOfRangeSamples);
    Check("multi echo global search clipping not hidden in clean local echo", clipping.size() == 2 && clipping[1].measured.searchHasOutOfRangeSamples && !clipping[1].localWindowHasOutOfRangeSamples && !clipping[1].peak.plateauOutOfRange);
    Check("multi echo all bounds safe with maximum neighborhood sizes", LidarDigitalMultiEchoEstimator({{0}, {std::numeric_limits<std::size_t>::max(), std::numeric_limits<std::size_t>::max()}}).Estimate(doubleWave, 0, {0, 9e-9}, 0, 0).size() == 2);
    Check("multi echo negative separation rejected", Throws<std::invalid_argument>([] { LidarDigitalMultiEchoEstimator e({{-1}, {0, 0}}); }));
    Check("multi echo invalid baseline reuses detector validation", Throws<std::invalid_argument>([&] { estimator.Estimate(single, 0, {0, 5e-9}, -1, 0); }));
    Check("multi echo invalid time reuses detector validation", Throws<std::invalid_argument>([&] { estimator.Estimate(single, -1, {0, 5e-9}, 0, 0); }));
    Check("multi echo invalid search reuses detector validation", Throws<std::invalid_argument>([&] { estimator.Estimate(single, 0, {5e-9, 0}, 0, 0); }));
    const auto repeated = estimator.Estimate(doubleWave, 0, {0, 9e-9}, 0, 0);
    Check("multi echo repeated estimation deterministic", repeated.size() == 2 && repeated[0].measured.arrivalTimeSeconds == two[0].measured.arrivalTimeSeconds && repeated[1].measured.arrivalTimeSeconds == two[1].measured.arrivalTimeSeconds);

    LidarWaveform optical({0, .25e-9, 1024});
    const GaussianPulseProfile profile(4e-9);
    optical.AccumulateReturn({1e-12, 70e-9, 532}, profile);
    optical.AccumulateReturn({.6e-12, 100e-9, 532}, profile);
    const auto current = LidarAnalogWaveform::FromOpticalWaveform(optical, LinearPhotodetector(532, .6));
    const auto response = FirstOrderLidarResponse(2e-9).Apply(current);
    const auto volts = LidarVoltageWaveform::FromCurrentWaveform(response, IdealTransimpedanceAmplifier(1e4));
    const auto digital = LidarDigitalWaveform::FromVoltageWaveform(volts, IdealUniformAdc(12, 0, 1));
    const auto physical = LidarDigitalMultiEchoEstimator({{15e-9}, {24, 47}}).Estimate(digital, 0, {40e-9, 140e-9}, 0, 4);
    Check("multi echo real two pulse chain produces two raw ranges", physical.size() == 2 && physical[0].measured.rangeMeters > 0 && physical[1].measured.rangeMeters > physical[0].measured.rangeMeters);
    Check("multi echo real receiver windows remain disjoint", physical.size() == 2 && physical[0].lastBinIndex < physical[1].firstBinIndex);
    Check("multi echo real receiver retains expected causal centroid delay", physical.size() == 2 && std::abs(physical[0].measured.arrivalTimeSeconds - 72e-9) < 1e-9 && std::abs(physical[1].measured.arrivalTimeSeconds - 102e-9) < 1e-9);
}
