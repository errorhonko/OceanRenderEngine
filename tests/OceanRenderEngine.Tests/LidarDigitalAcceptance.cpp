#include "LidarDigitalRangeEstimator.h"
#include "LidarDigitalBaselineEstimator.h"
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
bool Near(double a, double b) {
    return std::isfinite(a) && std::isfinite(b) &&
        std::abs(a-b) <= 1e-12 * std::max(std::abs(a), std::abs(b));
}
template<class E, class F> bool Throws(F f) {
    try { f(); } catch (const E&) { return true; }
    return false;
}
LidarDigitalWaveform Make(const LidarWaveformConfig& config,
    std::vector<double> values, const IdealUniformAdc& adc) {
    return LidarDigitalWaveform::FromVoltageWaveform(
        LidarVoltageWaveform(config, std::move(values)), adc);
}
}

void RunLidarDigitalAcceptanceTests() {
    IdealUniformAdc adc(12, 0, 1);
    LidarVoltageWaveform voltage({10e-9, 2e-9, 5}, {-.1, 0, .25, 1, 1.2});
    const auto before = voltage.AverageVoltageBinsV();
    const auto digital = LidarDigitalWaveform::FromVoltageWaveform(voltage, adc);
    const std::uint32_t expected[] = {0, 0, 1024, 4095, 4095};
    bool codes = true, flags = true, times = true, reconstruction = true;
    for (std::size_t i = 0; i < 5; ++i) {
        const auto& s = digital.Samples()[i];
        codes = codes && s.code == expected[i];
        flags = flags && s.belowRange == (i == 0) && s.aboveRange == (i == 4);
        times = times && digital.BinCenterTimeSeconds(i) == voltage.BinCenterTimeSeconds(i);
        reconstruction = reconstruction && digital.ReconstructedVoltageV(i) == adc.CodeCenterVoltageV(expected[i]);
    }
    Check("digital waveform every code analytic", codes && digital.Samples().size() == 5);
    Check("digital waveform every strict range flag retained", flags);
    Check("digital waveform time configuration retained", digital.Config().startTimeSeconds == 10e-9 && digital.Config().binWidthSeconds == 2e-9 && digital.Config().binCount == 5);
    Check("digital waveform every bin time retained", times);
    Check("digital waveform reconstruction uses code centers", reconstruction);
    Check("digital waveform reports range violation", digital.HasOutOfRangeSamples());
    Check("digital conversion leaves voltage unchanged", voltage.AverageVoltageBinsV() == before);
    voltage = LidarVoltageWaveform({0, 1, 1}, {0});
    adc = IdealUniformAdc(8, -1, 1);
    Check("digital waveform owns independent data and ADC", digital.Samples().size() == 5 && digital.Samples()[2].code == 1024 && digital.Adc().BitCount() == 12 && digital.Adc().LsbVoltageV() == 1.0/4096);
    Check("digital waveform checks time index", Throws<std::out_of_range>([&]{ digital.BinCenterTimeSeconds(5); }));
    Check("digital waveform checks reconstruction index", Throws<std::out_of_range>([&]{ digital.ReconstructedVoltageV(5); }));
    const IdealUniformAdc unipolar(12, 0, 1), bipolar(12, -1, 1);
    const auto clean = Make({0, 1, 3}, {0, .5, 1}, unipolar);
    Check("digital waveform exact endpoints do not report clipping", !clean.HasOutOfRangeSamples());
    const auto signedWave = Make({0, 1, 3}, {-.5, 0, .5}, bipolar);
    Check("digital waveform supports bipolar zero and signs", signedWave.Samples()[0].code == 1024 && signedWave.Samples()[1].code == 2048 && signedWave.Samples()[2].code == 3072 && !signedWave.HasOutOfRangeSamples());

    const LidarDigitalRangeEstimator estimator;
    const auto wave = Make({0, 2e-9, 4}, {0, .25, .75, .125}, unipolar);
    const auto result = estimator.Estimate(wave, 1e-9);
    Check("digital range detects known peak bin", result && result->peakBinIndex == 2);
    Check("digital range reports code and baseline amplitude", result && result->peakCode == 3072 && result->peakAboveBaselineCodes == 3072);
    Check("digital range arrival time uses bin center", result && Near(result->arrivalTimeSeconds, 5e-9));
    Check("digital range subtracts emission time", result && Near(result->timeOfFlightSeconds, 4e-9));
    Check("digital range monostatic c over two", result && Near(result->rangeMeters, .5 * 299792458.0 * 4e-9));
    Check("digital range clean peak flags clear", result && !result->peakOutOfRange && !result->searchHasOutOfRangeSamples);
    const auto zero = Make({0, 1, 3}, {0, 0, 0}, unipolar);
    Check("digital range zero waveform not detected", !estimator.Estimate(zero, 0));
    const auto baseline = Make({0, 1, 3}, {0, 0, 0}, bipolar);
    Check("digital range bipolar baseline not detected", !LidarDigitalRangeEstimator(2048).Estimate(baseline, 0));
    const auto belowBaseline = Make({0, 1, 3}, {-.5, -.25, 0}, bipolar);
    Check("digital range below baseline cannot unsigned underflow", !LidarDigitalRangeEstimator(2048).Estimate(belowBaseline, 0));
    const auto signedResult = LidarDigitalRangeEstimator(2048).Estimate(signedWave, 0);
    Check("digital range subtracts bipolar baseline", signedResult && signedResult->peakBinIndex == 2 && signedResult->peakCode == 3072 && signedResult->peakAboveBaselineCodes == 1024);
    Check("digital range threshold equality not detected", !LidarDigitalRangeEstimator(0, 3072).Estimate(wave, 0));
    Check("digital range one code above threshold detected", LidarDigitalRangeEstimator(0, 3071).Estimate(wave, 0).has_value());
    const auto tied = Make({0, 1, 4}, {0, .5, .5, 0}, unipolar);
    const auto tiedResult = estimator.Estimate(tied, 0);
    Check("digital range equal peaks choose earliest bin", tiedResult && tiedResult->peakBinIndex == 1);
    const auto gated = Make({0, 2e-9, 3}, {1.2, .25, .5}, unipolar);
    const auto gatedResult = estimator.Estimate(gated, 3e-9);
    Check("digital range excludes stronger pre-emission peak", gatedResult && gatedResult->peakBinIndex == 2);
    Check("digital range pre-emission clipping excluded from search flag", gated.HasOutOfRangeSamples() && gatedResult && !gatedResult->searchHasOutOfRangeSamples);
    const auto atEmission = estimator.Estimate(Make({0, 2e-9, 1}, {.5}, unipolar), 1e-9);
    Check("digital range center equal emission yields zero distance", atEmission && atEmission->timeOfFlightSeconds == 0 && atEmission->rangeMeters == 0);
    Check("digital range no post-emission bins not detected", !estimator.Estimate(wave, 1));
    const auto negativeStart = estimator.Estimate(Make({-4e-9, 2e-9, 4}, {1, 1, .25, .5}, unipolar), 0);
    Check("digital range negative time centers excluded", negativeStart && negativeStart->peakBinIndex == 3);
    const auto clipped = estimator.Estimate(Make({0, 1, 4}, {0, 1.2, 1.5, 0}, unipolar), 0);
    Check("digital range clipped plateau chooses first and flags", clipped && clipped->peakBinIndex == 1 && clipped->peakCode == 4095 && clipped->peakOutOfRange && clipped->searchHasOutOfRangeSamples);
    const auto under = estimator.Estimate(Make({0, 1, 3}, {0, .5, -.1}, unipolar), 0);
    Check("digital range nonpeak underrange sets only search flag", under && !under->peakOutOfRange && under->searchHasOutOfRangeSamples);
    const auto endpoint = estimator.Estimate(Make({0, 1, 1}, {1}, unipolar), 0);
    Check("digital range maximum code alone is not clipping", endpoint && endpoint->peakCode == 4095 && !endpoint->peakOutOfRange && !endpoint->searchHasOutOfRangeSamples);
    const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
    Check("digital range rejects negative emission", Throws<std::invalid_argument>([&]{ estimator.Estimate(wave, -1); }));
    Check("digital range rejects NaN emission", Throws<std::invalid_argument>([&]{ estimator.Estimate(wave, nan); }));
    Check("digital range rejects infinite emission", Throws<std::invalid_argument>([&]{ estimator.Estimate(wave, inf); }));
    Check("digital range rejects baseline outside ADC", Throws<std::invalid_argument>([&]{ LidarDigitalRangeEstimator(4096).Estimate(wave, 0); }));
    Check("digital range rejects threshold outside remaining span", Throws<std::invalid_argument>([&]{ LidarDigitalRangeEstimator(2048, 2048).Estimate(signedWave, 0); }));
    Check("digital range maximum baseline safely returns no detection", !LidarDigitalRangeEstimator(4095).Estimate(clean, 0));
    Check("digital range detects distance overflow", Throws<std::overflow_error>([&]{ estimator.Estimate(Make({std::numeric_limits<double>::max()/2, 1, 1}, {.5}, unipolar), 0); }));

    // Full deterministic signal chain, ending at ADC and digital peak ranging.
    // The reference is the output peak, not geometric truth: receiver delay remains.
    LidarWaveform optical({0, .25e-9, 1024});
    optical.AccumulateReturn({1e-12, 100.2e-9, 532}, GaussianPulseProfile(4e-9));
    const auto current = LidarAnalogWaveform::FromOpticalWaveform(optical, LinearPhotodetector(532, .6));
    const auto response = FirstOrderLidarResponse(2e-9).Apply(current);
    const auto volts = LidarVoltageWaveform::FromCurrentWaveform(response, IdealTransimpedanceAmplifier(10e3));
    const auto digits = LidarDigitalWaveform::FromVoltageWaveform(volts, unipolar);
    const auto measured = LidarDigitalRangeEstimator(unipolar.Quantize(0).code, 4).Estimate(digits, 0);
    const auto& values = volts.AverageVoltageBinsV();
    const auto analogPeak = std::size_t(std::max_element(values.begin(), values.end()) - values.begin());
    Check("digital complete optical to ranging chain detects return", measured.has_value());
    Check("digital complete chain stays inside ADC range", !digits.HasOutOfRangeSamples());
    Check("digital complete chain peak within one analog bin", measured && std::abs(measured->arrivalTimeSeconds - volts.BinCenterTimeSeconds(analogPeak)) <= volts.Config().binWidthSeconds);
    Check("digital complete chain distance matches chosen output time", measured && Near(measured->rangeMeters, .5 * 299792458.0 * digits.BinCenterTimeSeconds(measured->peakBinIndex)));

    // Generate exact integer codes through their interval centers.
    const auto fromCodes = [&](const LidarWaveformConfig& cfg,
        const std::vector<std::uint32_t>& codes) {
        std::vector<double> v;
        for (auto code : codes) v.push_back(unipolar.CodeCenterVoltageV(code));
        return Make(cfg, std::move(v), unipolar);
    };
    const auto asymmetric = fromCodes({99.5e-9, 1e-9, 3}, {1, 4, 2});
    const auto centroid = estimator.EstimateCentroid(asymmetric, 90e-9);
    const auto asymmetricPeak = estimator.Estimate(asymmetric, 90e-9);
    const double expectedTime = (100.0 + 4*101.0 + 2*102.0) / 7 * 1e-9;
    Check("digital centroid asymmetric weights analytic time", centroid && Near(centroid->arrivalTimeSeconds, expectedTime));
    Check("digital centroid relative time subtracts emission", centroid && Near(centroid->timeOfFlightSeconds, expectedTime - 90e-9));
    Check("digital centroid analytic monostatic range", centroid && Near(centroid->rangeMeters, .5 * 299792458.0 * (expectedTime - 90e-9)));
    Check("digital centroid can lie between bin centers", centroid && centroid->arrivalTimeSeconds > asymmetric.BinCenterTimeSeconds(1) && centroid->arrivalTimeSeconds < asymmetric.BinCenterTimeSeconds(2));
    Check("digital centroid retains peak diagnostics", centroid && asymmetricPeak && centroid->peakBinIndex == asymmetricPeak->peakBinIndex && centroid->peakCode == asymmetricPeak->peakCode && centroid->peakAboveBaselineCodes == asymmetricPeak->peakAboveBaselineCodes);
    const auto scaled = estimator.EstimateCentroid(fromCodes({99.5e-9, 1e-9, 3}, {2, 8, 4}), 90e-9);
    Check("digital centroid amplitude scaling leaves time unchanged", scaled && centroid && Near(scaled->arrivalTimeSeconds, centroid->arrivalTimeSeconds));
    const auto symmetric = estimator.EstimateCentroid(fromCodes({0, 2e-9, 5}, {0, 2, 8, 2, 0}), 0);
    Check("digital centroid symmetric pulse at center", symmetric && Near(symmetric->arrivalTimeSeconds, 5e-9));
    const auto single = estimator.EstimateCentroid(fromCodes({0, 2e-9, 3}, {0, 1, 0}), 0);
    Check("digital centroid one code single bin detected", single && Near(single->arrivalTimeSeconds, 3e-9));
    Check("digital centroid all zero returns no detection", !estimator.EstimateCentroid(zero, 0));
    Check("digital centroid bipolar baseline returns no detection", !LidarDigitalRangeEstimator(2048).EstimateCentroid(baseline, 0));
    Check("digital centroid below baseline cannot underflow", !LidarDigitalRangeEstimator(2048).EstimateCentroid(belowBaseline, 0));
    const auto offsetWave = fromCodes({97.5e-9, 1e-9, 5}, {1000, 990, 1001, 1004, 1002});
    const auto offset = LidarDigitalRangeEstimator(1000).EstimateCentroid(offsetWave, 90e-9);
    Check("digital centroid subtracts nonzero baseline", offset && Near(offset->arrivalTimeSeconds, expectedTime));
    Check("digital centroid threshold equality no detection", !LidarDigitalRangeEstimator(0, 4).EstimateCentroid(asymmetric, 90e-9));
    const auto thresholded = LidarDigitalRangeEstimator(0, 3).EstimateCentroid(asymmetric, 90e-9);
    Check("digital centroid detection threshold does not discard weaker tails", thresholded && Near(thresholded->arrivalTimeSeconds, expectedTime));
    const auto emissionGated = estimator.EstimateCentroid(fromCodes({0, 2e-9, 3}, {100, 1, 3}), 3e-9);
    Check("digital centroid excludes stronger pre-emission bin", emissionGated && Near(emissionGated->arrivalTimeSeconds, 4.5e-9));
    Check("digital centroid gated relative time analytic", emissionGated && Near(emissionGated->timeOfFlightSeconds, 1.5e-9));
    const auto exactlyAtEmission = estimator.EstimateCentroid(fromCodes({0, 2e-9, 1}, {1}), 1e-9);
    Check("digital centroid emission center included with zero distance", exactlyAtEmission && exactlyAtEmission->rangeMeters == 0 && exactlyAtEmission->timeOfFlightSeconds == 0);
    Check("digital centroid empty post-emission search no detection", !estimator.EstimateCentroid(asymmetric, 1));
    const auto clippedCentroid = estimator.EstimateCentroid(Make({0, 1, 4}, {0, 1.2, 1.5, 0}, unipolar), 0);
    Check("digital centroid clipped plateau averages digitized bins", clippedCentroid && Near(clippedCentroid->arrivalTimeSeconds, 2));
    Check("digital centroid clipping flags and first peak retained", clippedCentroid && clippedCentroid->peakBinIndex == 1 && clippedCentroid->peakOutOfRange && clippedCentroid->searchHasOutOfRangeSamples);
    const auto underCentroid = estimator.EstimateCentroid(Make({0, 1, 3}, {0, .5, -.1}, unipolar), 0);
    Check("digital centroid ignores low clipping but retains diagnostic", underCentroid && Near(underCentroid->arrivalTimeSeconds, 1.5) && !underCentroid->peakOutOfRange && underCentroid->searchHasOutOfRangeSamples);
    const auto beforeClip = estimator.EstimateCentroid(gated, 3e-9);
    Check("digital centroid pre-emission clipping excluded from flag", beforeClip && !beforeClip->searchHasOutOfRangeSamples && Near(beforeClip->arrivalTimeSeconds, (3.0 + 2*5.0)/3 * 1e-9));
    const auto shifted = estimator.EstimateCentroid(fromCodes({1e6, .25, 3}, {1, 4, 2}), 1e6);
    Check("digital centroid large clock offset preserves relative time", shifted && Near(shifted->timeOfFlightSeconds, .125 + .25*8/7));
    Check("digital centroid rejects negative emission", Throws<std::invalid_argument>([&]{ estimator.EstimateCentroid(asymmetric, -1); }));
    Check("digital centroid rejects NaN emission", Throws<std::invalid_argument>([&]{ estimator.EstimateCentroid(asymmetric, nan); }));
    Check("digital centroid rejects infinite emission", Throws<std::invalid_argument>([&]{ estimator.EstimateCentroid(asymmetric, inf); }));
    Check("digital centroid rejects invalid baseline", Throws<std::invalid_argument>([&]{ LidarDigitalRangeEstimator(4096).EstimateCentroid(asymmetric, 0); }));
    Check("digital centroid rejects invalid threshold", Throws<std::invalid_argument>([&]{ LidarDigitalRangeEstimator(2048, 2048).EstimateCentroid(signedWave, 0); }));
    const auto hugeTimes = fromCodes({0, 1e300, 3}, {4095, 0, 4094});
    Check("digital centroid overflow case has finite peak distance", estimator.Estimate(hugeTimes, 0).has_value());
    Check("digital centroid detects weighted distance overflow", Throws<std::overflow_error>([&]{ estimator.EstimateCentroid(hugeTimes, 0); }));
    // Independent direct long-double sum, not the implementation's online mean.
    const auto digitalMean = estimator.EstimateCentroid(digits, 0);
    long double codeSum = 0, codeMoment = 0, voltageSum = 0, voltageMoment = 0;
    for (std::size_t i = 0; i < digits.Samples().size(); ++i) {
        const long double time = digits.BinCenterTimeSeconds(i);
        const long double code = digits.Samples()[i].code;
        codeSum += code;
        codeMoment += code * time;
        voltageSum += values[i];
        voltageMoment += static_cast<long double>(values[i]) * time;
    }
    const long double directDigitalTime = codeMoment / codeSum;
    const long double analogTime = voltageMoment / voltageSum;
    Check("digital centroid full chain matches independent weighted sum", digitalMean && std::abs(digitalMean->arrivalTimeSeconds - directDigitalTime) < 1e-18);
    Check("digital centroid full chain range follows weighted time", digitalMean && Near(digitalMean->rangeMeters, .5 * 299792458.0 * double(directDigitalTime)));
    Check("digital centroid full chain retains output peak diagnostics", digitalMean && measured && digitalMean->peakBinIndex == measured->peakBinIndex && digitalMean->peakCode == measured->peakCode && !digitalMean->searchHasOutOfRangeSamples);
    // Baseline code zero: q*LSB approximates positive voltage with <1 LSB error.
    // This is not the code-center reconstruction's half-LSB bound.
    long double voltageErrorMoment = 0;
    for (std::size_t i = 0; i < values.size(); ++i) {
        const long double quantized = static_cast<long double>(digits.Samples()[i].code) * unipolar.LsbVoltageV();
        voltageErrorMoment += std::abs(quantized - values[i]) * std::abs(digits.BinCenterTimeSeconds(i) - analogTime);
    }
    const long double quantizationTimeBound = voltageErrorMoment / (codeSum * unipolar.LsbVoltageV());
    Check("digital centroid analog difference obeys amplitude-error bound", digitalMean && std::abs(digitalMean->arrivalTimeSeconds - analogTime) <= quantizationTimeBound + 1e-18);
    std::cout << "[INFO] centroid analog ns=" << double(analogTime*1e9)
        << " digital ns=" << double(directDigitalTime*1e9)
        << " quantization shift ps=" << double((directDigitalTime-analogTime)*1e12) << '\n';

    // Binary-exact times: centers 1,3,5,7 seconds isolate window semantics.
    const auto windowWave = fromCodes({0, 2, 4}, {100, 1, 3, 200});
    const LidarReturnWindow innerWindow{3, 7};
    const auto windowPeak = estimator.Estimate(windowWave, 0, innerWindow);
    const auto windowMean = estimator.EstimateCentroid(windowWave, 0, innerWindow);
    Check("digital window excludes stronger outside peaks", windowPeak && windowPeak->peakBinIndex == 2 && windowPeak->peakCode == 3);
    Check("digital window centroid uses only selected weights", windowMean && Near(windowMean->arrivalTimeSeconds, 4.5));
    Check("digital window centroid retains selected peak diagnostic", windowMean && windowMean->peakBinIndex == 2 && windowMean->peakAboveBaselineCodes == 3);
    const auto leftIncluded = estimator.Estimate(windowWave, 0, LidarReturnWindow{3, 5});
    const auto leftMean = estimator.EstimateCentroid(windowWave, 0, LidarReturnWindow{3, 5});
    Check("digital window left boundary includes center", leftIncluded && leftIncluded->peakBinIndex == 1 && leftIncluded->arrivalTimeSeconds == 3);
    Check("digital window right boundary excludes stronger center", leftMean && leftMean->arrivalTimeSeconds == 3);
    Check("digital window peak threshold ignores outside strong signal", !LidarDigitalRangeEstimator(0, 3).Estimate(windowWave, 0, innerWindow));
    Check("digital window centroid detection uses same window", !LidarDigitalRangeEstimator(0, 3).EstimateCentroid(windowWave, 0, innerWindow));
    const auto intersectPeak = estimator.Estimate(windowWave, 5, innerWindow);
    const auto intersectMean = estimator.EstimateCentroid(windowWave, 5, innerWindow);
    Check("digital window intersects emission gate for peak", intersectPeak && intersectPeak->peakBinIndex == 2 && intersectPeak->rangeMeters == 0);
    Check("digital window intersects emission gate for centroid", intersectMean && intersectMean->arrivalTimeSeconds == 5 && intersectMean->timeOfFlightSeconds == 0);
    Check("digital window before emission has no detection", !estimator.EstimateCentroid(windowWave, 8, innerWindow));
    Check("digital window no bin centers returns no peak", !estimator.Estimate(windowWave, 0, LidarReturnWindow{2, 2.5}));
    Check("digital window no bin centers returns no centroid", !estimator.EstimateCentroid(windowWave, 0, LidarReturnWindow{2, 2.5}));
    Check("digital window outside recording returns no detection", !estimator.EstimateCentroid(windowWave, 0, LidarReturnWindow{20, 30}));
    const auto partial = estimator.EstimateCentroid(windowWave, 0, LidarReturnWindow{-10, 4});
    Check("digital window partial overlap and negative start valid", partial && Near(partial->arrivalTimeSeconds, 103.0/101));
    const auto windowClipped = Make({0, 2, 4}, {1.2, .25, .5, -.1}, unipolar);
    const auto cleanWindowPeak = estimator.Estimate(windowClipped, 0, innerWindow);
    const auto cleanWindowMean = estimator.EstimateCentroid(windowClipped, 0, innerWindow);
    Check("digital window outside clipping excluded from peak flags", cleanWindowPeak && !cleanWindowPeak->peakOutOfRange && !cleanWindowPeak->searchHasOutOfRangeSamples && windowClipped.HasOutOfRangeSamples());
    Check("digital window outside clipping excluded from centroid flags", cleanWindowMean && !cleanWindowMean->searchHasOutOfRangeSamples && Near(cleanWindowMean->arrivalTimeSeconds, 13.0/3));
    const auto clippedWindow = estimator.EstimateCentroid(windowClipped, 0, LidarReturnWindow{1, 3});
    Check("digital window inside clipping still reported", clippedWindow && clippedWindow->peakOutOfRange && clippedWindow->searchHasOutOfRangeSamples);
    const auto oldPeak = estimator.Estimate(windowWave, 0);
    const auto fullPeak = estimator.Estimate(windowWave, 0, LidarReturnWindow{0, 8});
    const auto oldMean = estimator.EstimateCentroid(windowWave, 0);
    const auto fullMean = estimator.EstimateCentroid(windowWave, 0, LidarReturnWindow{0, 8});
    Check("digital window omitted preserves full peak search", oldPeak && fullPeak && oldPeak->peakBinIndex == fullPeak->peakBinIndex && oldPeak->rangeMeters == fullPeak->rangeMeters);
    Check("digital window omitted preserves full centroid search", oldMean && fullMean && oldMean->arrivalTimeSeconds == fullMean->arrivalTimeSeconds && oldMean->rangeMeters == fullMean->rangeMeters);
    Check("digital window explicit nullopt preserves centroid", oldMean && estimator.EstimateCentroid(windowWave, 0, std::nullopt)->arrivalTimeSeconds == oldMean->arrivalTimeSeconds);
    Check("digital window rejects zero duration", Throws<std::invalid_argument>([&]{ estimator.Estimate(windowWave, 0, LidarReturnWindow{3, 3}); }));
    Check("digital window rejects reversed bounds", Throws<std::invalid_argument>([&]{ estimator.EstimateCentroid(windowWave, 0, LidarReturnWindow{7, 3}); }));
    Check("digital window rejects NaN start", Throws<std::invalid_argument>([&]{ estimator.Estimate(windowWave, 0, LidarReturnWindow{nan, 7}); }));
    Check("digital window rejects NaN end through centroid", Throws<std::invalid_argument>([&]{ estimator.EstimateCentroid(windowWave, 0, LidarReturnWindow{0, nan}); }));
    Check("digital window rejects infinite start", Throws<std::invalid_argument>([&]{ estimator.Estimate(windowWave, 0, LidarReturnWindow{-inf, 7}); }));
    Check("digital window rejects infinite end", Throws<std::invalid_argument>([&]{ estimator.EstimateCentroid(windowWave, 0, LidarReturnWindow{0, inf}); }));
    Check("digital window invalid bounds checked even on zero signal", Throws<std::invalid_argument>([&]{ estimator.EstimateCentroid(zero, 0, LidarReturnWindow{1, 1}); }));

    const auto neighborhoodWave = fromCodes({0,2,6}, {1,4,10,6,2,0});
    const auto singlePeak = estimator.EstimatePeakNeighborhoodCentroid(neighborhoodWave,0,{0,0});
    Check("neighborhood zero before after uses peak only", singlePeak && singlePeak->peakBinIndex == 2 && singlePeak->arrivalTimeSeconds == 5);
    const auto balanced = estimator.EstimatePeakNeighborhoodCentroid(neighborhoodWave,0,{1,1});
    Check("neighborhood analytic local centroid", balanced && Near(balanced->arrivalTimeSeconds,5.2));
    Check("neighborhood reports centroid propagation time and range", balanced && Near(balanced->timeOfFlightSeconds,5.2) && Near(balanced->rangeMeters,.5*299792458.0*5.2));
    Check("neighborhood retains peak diagnostics", balanced && balanced->peakBinIndex == 2 && balanced->peakCode == 10 && balanced->peakAboveBaselineCodes == 10);
    const auto beforeOnly = estimator.EstimatePeakNeighborhoodCentroid(neighborhoodWave,0,{1,0});
    const auto afterOnly = estimator.EstimatePeakNeighborhoodCentroid(neighborhoodWave,0,{0,1});
    Check("neighborhood before only analytic", beforeOnly && Near(beforeOnly->arrivalTimeSeconds,31.0/7));
    Check("neighborhood after only analytic", afterOnly && Near(afterOnly->arrivalTimeSeconds,5.75));
    const auto largest = std::numeric_limits<std::size_t>::max();
    const auto all = estimator.EstimatePeakNeighborhoodCentroid(neighborhoodWave,0,{largest,largest});
    const auto allReference = estimator.EstimateCentroid(neighborhoodWave,0);
    Check("neighborhood huge counts safely cover full record", all && allReference && all->arrivalTimeSeconds == allReference->arrivalTimeSeconds);
    const auto firstEdge = estimator.EstimatePeakNeighborhoodCentroid(fromCodes({0,2,3},{10,4,1}),0,{largest,1});
    Check("neighborhood first bin truncates before without underflow", firstEdge && firstEdge->peakBinIndex == 0 && Near(firstEdge->arrivalTimeSeconds,11.0/7));
    const auto lastEdge = estimator.EstimatePeakNeighborhoodCentroid(fromCodes({0,2,3},{1,4,10}),0,{1,largest});
    Check("neighborhood last bin truncates after without overflow", lastEdge && lastEdge->peakBinIndex == 2 && Near(lastEdge->arrivalTimeSeconds,31.0/7));
    const auto oneBin = estimator.EstimatePeakNeighborhoodCentroid(fromCodes({0,2,1},{10}),0,{largest,largest});
    Check("neighborhood one bin record supported", oneBin && oneBin->arrivalTimeSeconds == 1);
    const auto intersected = estimator.EstimatePeakNeighborhoodCentroid(neighborhoodWave,0,{largest,largest},LidarReturnWindow{3,7});
    Check("neighborhood intersects search with half open bounds", intersected && Near(intersected->arrivalTimeSeconds,31.0/7));
    const auto fractionalGate = estimator.EstimatePeakNeighborhoodCentroid(neighborhoodWave,0,{largest,largest},LidarReturnWindow{3.5,6});
    Check("neighborhood fractional search uses selected bin centers", fractionalGate && fractionalGate->arrivalTimeSeconds == 5);
    const auto relativeNeighborhood = estimator.EstimatePeakNeighborhoodCentroid(neighborhoodWave,5,{largest,largest});
    Check("neighborhood emission gate applies inside local window", relativeNeighborhood && Near(relativeNeighborhood->arrivalTimeSeconds,55.0/9) && Near(relativeNeighborhood->timeOfFlightSeconds,10.0/9));
    Check("neighborhood zero signal returns no detection", !estimator.EstimatePeakNeighborhoodCentroid(zero,0,{1,1}));
    Check("neighborhood equality threshold returns no detection", !LidarDigitalRangeEstimator(0,10).EstimatePeakNeighborhoodCentroid(neighborhoodWave,0,{1,1}));
    Check("neighborhood one code above threshold detected", LidarDigitalRangeEstimator(0,9).EstimatePeakNeighborhoodCentroid(neighborhoodWave,0,{1,1}).has_value());
    const auto withBaseline = LidarDigitalRangeEstimator(100).EstimatePeakNeighborhoodCentroid(fromCodes({0,2,5},{80,100,110,106,100}),0,{1,1});
    Check("neighborhood baseline subtraction analytic", withBaseline && Near(withBaseline->arrivalTimeSeconds,5.75) && withBaseline->peakAboveBaselineCodes == 10);
    const auto tiedLocal = estimator.EstimatePeakNeighborhoodCentroid(fromCodes({0,2,3},{0,10,10}),0,{0,0});
    Check("neighborhood equal peak tie still chooses earliest", tiedLocal && tiedLocal->peakBinIndex == 1 && tiedLocal->arrivalTimeSeconds == 3);
    const auto clippingWave = Make({0,2,6},{-.1,.25,.5,0,0,0},unipolar);
    const auto narrowClipping = estimator.EstimatePeakNeighborhoodCentroid(clippingWave,0,{0,0});
    Check("neighborhood preserves broad search clipping outside local window", narrowClipping && !narrowClipping->peakOutOfRange && narrowClipping->searchHasOutOfRangeSamples);
    const auto excludesClipping = estimator.EstimatePeakNeighborhoodCentroid(clippingWave,0,{0,0},LidarReturnWindow{3,11});
    Check("neighborhood clipping outside broad search not reported", excludesClipping && !excludesClipping->searchHasOutOfRangeSamples);
    const auto clippedLocal = estimator.EstimatePeakNeighborhoodCentroid(Make({0,2,3},{0,1.2,0},unipolar),0,{0,0});
    Check("neighborhood clipped peak diagnostic retained", clippedLocal && clippedLocal->peakOutOfRange && clippedLocal->searchHasOutOfRangeSamples);
    Check("neighborhood invalid search window rejected", Throws<std::invalid_argument>([&]{ estimator.EstimatePeakNeighborhoodCentroid(neighborhoodWave,0,{1,1},LidarReturnWindow{3,3}); }));
    Check("neighborhood negative emission rejected", Throws<std::invalid_argument>([&]{ estimator.EstimatePeakNeighborhoodCentroid(neighborhoodWave,-1,{1,1}); }));
    Check("neighborhood nonfinite emission rejected", Throws<std::invalid_argument>([&]{ estimator.EstimatePeakNeighborhoodCentroid(neighborhoodWave,nan,{1,1}); }));
    Check("neighborhood outside search returns no detection", !estimator.EstimatePeakNeighborhoodCentroid(neighborhoodWave,0,{1,1},LidarReturnWindow{20,30}));
    Check("neighborhood collapsed double time boundaries rejected", Throws<std::domain_error>([&]{ estimator.EstimatePeakNeighborhoodCentroid(fromCodes({1e20,1,1},{10}),0,{0,0}); }));
    const auto production12 = estimator.EstimatePeakNeighborhoodCentroid(digits,0,{16,31},LidarReturnWindow{90e-9,125e-9});
    Check("neighborhood physical chain detects return", production12 && std::isfinite(production12->rangeMeters));
    if (production12) {
        const auto index = production12->peakBinIndex;
        const auto low = index-std::min(std::size_t{16},index);
        const auto high = index+std::min(std::size_t{31},digits.Samples().size()-1-index);
        const auto& cfg = digits.Config();
        const LidarReturnWindow manualWindow{std::max(90e-9,cfg.startTimeSeconds+low*cfg.binWidthSeconds),std::min(125e-9,cfg.startTimeSeconds+(high+1)*cfg.binWidthSeconds)};
        const auto manual = estimator.EstimateCentroid(digits,0,manualWindow);
        Check("neighborhood physical chain equals manual window", manual && manual->arrivalTimeSeconds == production12->arrivalTimeSeconds && manual->rangeMeters == production12->rangeMeters);
    }

    const auto fractionalWave = fromCodes({0,2,3},{2048,2050,2051});
    const LidarDigitalRangeEstimator fractionalEstimator(2047.5,3.0);
    const auto fractionalPeak = fractionalEstimator.Estimate(fractionalWave,0);
    Check("fractional baseline peak raw code stays integer", fractionalPeak && fractionalPeak->peakCode == 2051 && fractionalPeak->peakBinIndex == 2);
    Check("fractional baseline peak amplitude retains half code", fractionalPeak && fractionalPeak->peakAboveBaselineCodes == 3.5);
    const auto fractionalMean = fractionalEstimator.EstimateCentroid(fractionalWave,0);
    Check("fractional baseline centroid uses unrounded weights", fractionalMean && Near(fractionalMean->arrivalTimeSeconds,51.0/13));
    Check("fractional baseline centroid keeps subthreshold tails", fractionalMean && fractionalMean->arrivalTimeSeconds < fractionalWave.BinCenterTimeSeconds(2));
    Check("fractional baseline centroid range analytic", fractionalMean && Near(fractionalMean->rangeMeters,.5*299792458.0*51/13));
    const auto roundedMean = LidarDigitalRangeEstimator(2048,3-1e-6).EstimateCentroid(fractionalWave,0);
    Check("fractional baseline differs from prematurely rounded baseline", roundedMean && fractionalMean && Near(roundedMean->arrivalTimeSeconds,4.2) && std::abs(roundedMean->arrivalTimeSeconds-fractionalMean->arrivalTimeSeconds)>.1);
    Check("fractional threshold equality is not detection", !LidarDigitalRangeEstimator(2047.5,3.5).Estimate(fractionalWave,0));
    Check("fractional threshold immediately below amplitude detects", LidarDigitalRangeEstimator(2047.5,std::nextafter(3.5,0.0)).Estimate(fractionalWave,0).has_value());
    Check("fractional threshold immediately above amplitude rejects", !LidarDigitalRangeEstimator(2047.5,std::nextafter(3.5,inf)).Estimate(fractionalWave,0));
    const auto fractionalLocal = fractionalEstimator.EstimatePeakNeighborhoodCentroid(fractionalWave,0,{1,0});
    Check("fractional baseline neighborhood centroid analytic", fractionalLocal && Near(fractionalLocal->arrivalTimeSeconds,25.0/6) && fractionalLocal->peakAboveBaselineCodes == 3.5);
    Check("fractional baseline empty positive signal not detected", !LidarDigitalRangeEstimator(2047.5,0).Estimate(fromCodes({0,1,3},{2047,2047,2047}),0));
    const auto fractionMaximum = LidarDigitalRangeEstimator(4094.5,.49).Estimate(fromCodes({0,1,1},{4095}),0);
    Check("fractional baseline at upper code edge supported", fractionMaximum && fractionMaximum->peakAboveBaselineCodes == .5 && !fractionMaximum->peakOutOfRange);
    Check("fractional remaining span threshold equality no detection", !LidarDigitalRangeEstimator(4094.5,.5).Estimate(fromCodes({0,1,1},{4095}),0));
    Check("fractional threshold beyond remaining span rejected", Throws<std::invalid_argument>([&]{LidarDigitalRangeEstimator(4094.5,.51).Estimate(fractionalWave,0);}));
    Check("fractional baseline outside ADC span rejected", Throws<std::invalid_argument>([&]{LidarDigitalRangeEstimator(4095.1).Estimate(fractionalWave,0);}));
    Check("fractional baseline negative rejected", Throws<std::invalid_argument>([]{LidarDigitalRangeEstimator bad(-.5);}));
    Check("fractional baseline NaN rejected", Throws<std::invalid_argument>([&]{LidarDigitalRangeEstimator bad(nan);}));
    Check("fractional baseline infinity rejected", Throws<std::invalid_argument>([&]{LidarDigitalRangeEstimator bad(inf);}));
    Check("fractional threshold negative rejected", Throws<std::invalid_argument>([]{LidarDigitalRangeEstimator bad(0,-.5);}));
    Check("fractional threshold NaN rejected", Throws<std::invalid_argument>([&]{LidarDigitalRangeEstimator bad(0,nan);}));
    Check("fractional threshold infinity rejected", Throws<std::invalid_argument>([&]{LidarDigitalRangeEstimator bad(0,inf);}));
    const auto nonBinary = LidarDigitalRangeEstimator(2047.3,3).Estimate(fractionalWave,0);
    Check("fractional baseline nonbinary decimal supported", nonBinary && Near(nonBinary->peakAboveBaselineCodes,3.7));
    const auto baselineInput = fromCodes({0,1,6},{2047,2048,2047,2048,2050,2051});
    const auto estimatedBaseline = LidarDigitalBaselineEstimator::Estimate(baselineInput,{0,4});
    const LidarDigitalRangeEstimator fromEstimate(estimatedBaseline.meanCode,6*estimatedBaseline.sigmaCodes);
    const auto estimatedPeak = fromEstimate.Estimate(baselineInput,0,LidarReturnWindow{4,6});
    const auto estimatedMean = fromEstimate.EstimateCentroid(baselineInput,0,LidarReturnWindow{4,6});
    Check("fractional baseline estimate feeds peak detection without rounding", estimatedPeak && estimatedPeak->peakAboveBaselineCodes == 3.5 && estimatedPeak->peakCode == 2051);
    Check("fractional estimated sigma threshold feeds centroid", estimatedMean && Near(estimatedMean->arrivalTimeSeconds,61.0/12));
    const auto integralDouble = LidarDigitalRangeEstimator(0.0,0.0).EstimateCentroid(neighborhoodWave,0);
    const auto integralOld = estimator.EstimateCentroid(neighborhoodWave,0);
    Check("fractional support preserves integer parameter results", integralDouble && integralOld && integralDouble->arrivalTimeSeconds == integralOld->arrivalTimeSeconds && integralDouble->peakAboveBaselineCodes == integralOld->peakAboveBaselineCodes);
}
