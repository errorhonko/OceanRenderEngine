#include "LidarRangeCalibration.h"
#include "FirstOrderLidarResponse.h"
#include "GaussianPulseProfile.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
constexpr double halfC = .5 * 299792458.0;
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
LidarDigitalRangeEstimate Measurement(double tof, double emit = 0) {
    return {7, emit + tof, tof, halfC * tof, 100, 80, false, false};
}
bool Same(const LidarDigitalRangeEstimate& a, const LidarDigitalRangeEstimate& b) {
    return a.peakBinIndex == b.peakBinIndex && a.arrivalTimeSeconds == b.arrivalTimeSeconds &&
        a.timeOfFlightSeconds == b.timeOfFlightSeconds && a.rangeMeters == b.rangeMeters &&
        a.peakCode == b.peakCode && a.peakAboveBaselineCodes == b.peakAboveBaselineCodes &&
        a.peakOutOfRange == b.peakOutOfRange && a.searchHasOutOfRangeSamples == b.searchHasOutOfRangeSamples;
}
LidarDigitalWaveform Signal(double arrival) {
    LidarWaveform optical({0, .25e-9, 1024});
    optical.AccumulateReturn({1e-12, arrival, 532}, GaussianPulseProfile(4e-9));
    const auto current = LidarAnalogWaveform::FromOpticalWaveform(optical, LinearPhotodetector(532, .6));
    const auto response = FirstOrderLidarResponse(2e-9).Apply(current);
    const auto voltage = LidarVoltageWaveform::FromCurrentWaveform(response, IdealTransimpedanceAmplifier(10e3));
    return LidarDigitalWaveform::FromVoltageWaveform(voltage, IdealUniformAdc(12, 0, 1));
}
}

void RunLidarRangeCalibrationAcceptanceTests() {
    const LidarRangeCalibration zero, calibration(2e-9);
    const auto measured = Measurement(102e-9, 10e-9);
    const auto corrected = calibration.Apply(measured);
    const auto unchanged = zero.Apply(measured);
    Check("calibration default delay zero", zero.SystemDelaySeconds() == 0);
    Check("calibration stores known delay", calibration.SystemDelaySeconds() == 2e-9);
    Check("calibration zero delay preserves time and range", unchanged && unchanged->correctedTimeOfFlightSeconds == measured.timeOfFlightSeconds && unchanged->correctedRangeMeters == measured.rangeMeters);
    Check("calibration subtracts known two ns delay", corrected && Near(corrected->correctedTimeOfFlightSeconds, 100e-9));
    Check("calibration known corrected distance analytic", corrected && Near(corrected->correctedRangeMeters, halfC * 100e-9));
    Check("calibration distance reduction equals half c times delay", corrected && Near(measured.rangeMeters - corrected->correctedRangeMeters, halfC * 2e-9));
    Check("calibration preserves original result fields", corrected && Same(corrected->measured, measured));
    Check("calibration does not modify input arrival or time", measured.arrivalTimeSeconds == 10e-9 + 102e-9 && measured.timeOfFlightSeconds == 102e-9);
    Check("calibration rejects negative corrected time without clamping", !calibration.Apply(Measurement(1e-9)));
    const auto atZero = calibration.Apply(Measurement(2e-9));
    Check("calibration equality yields valid zero distance", atZero && atZero->correctedTimeOfFlightSeconds == 0 && atZero->correctedRangeMeters == 0);
    auto flagged = measured;
    flagged.peakOutOfRange = true;
    flagged.searchHasOutOfRangeSamples = true;
    const auto flaggedResult = calibration.Apply(flagged);
    Check("calibration application preserves clipping diagnostic", flaggedResult && Same(flaggedResult->measured, flagged));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const double maximum = std::numeric_limits<double>::max();
    Check("calibration rejects negative delay", Throws<std::invalid_argument>([]{ LidarRangeCalibration bad(-1); }));
    Check("calibration rejects NaN delay", Throws<std::invalid_argument>([&]{ LidarRangeCalibration bad(nan); }));
    Check("calibration rejects infinite delay", Throws<std::invalid_argument>([&]{ LidarRangeCalibration bad(inf); }));
    auto invalid = measured; invalid.arrivalTimeSeconds = nan;
    Check("calibration rejects invalid arrival", Throws<std::invalid_argument>([&]{ zero.Apply(invalid); }));
    invalid = measured; invalid.timeOfFlightSeconds = inf;
    Check("calibration rejects infinite measured time", Throws<std::invalid_argument>([&]{ zero.Apply(invalid); }));
    invalid = measured; invalid.timeOfFlightSeconds = -1;
    Check("calibration rejects negative measured time", Throws<std::invalid_argument>([&]{ zero.Apply(invalid); }));
    invalid = measured; invalid.rangeMeters = -1;
    Check("calibration rejects negative raw range", Throws<std::invalid_argument>([&]{ zero.Apply(invalid); }));
    invalid = measured; invalid.rangeMeters = nan;
    Check("calibration rejects nonfinite raw range", Throws<std::invalid_argument>([&]{ zero.Apply(invalid); }));
    // Public result struct can contain inconsistent manually supplied fields.
    invalid = measured; invalid.timeOfFlightSeconds = maximum;
    Check("calibration detects recomputed range overflow", Throws<std::overflow_error>([&]{ zero.Apply(invalid); }));

    const double referenceRange = 15.0;
    const auto reference = Measurement(referenceRange / halfC + 2e-9, 10e-9);
    const auto fitted = LidarRangeCalibration::FromReferenceMeasurement(reference, referenceRange);
    Check("calibration reference factory recovers known delay", Near(fitted.SystemDelaySeconds(), 2e-9));
    const auto referenceCorrected = fitted.Apply(reference);
    Check("calibration reference algebra closes at known range", referenceCorrected && Near(referenceCorrected->correctedRangeMeters, referenceRange));
    const auto second = Measurement(30.0 / halfC + 2e-9, 40e-9);
    const auto secondCorrected = fitted.Apply(second);
    Check("calibration known fixed delay transfers to separate target", secondCorrected && Near(secondCorrected->correctedRangeMeters, 30));
    Check("calibration different emission time does not change fitted delay", Near(LidarRangeCalibration::FromReferenceMeasurement(Measurement(reference.timeOfFlightSeconds, 0), referenceRange).SystemDelaySeconds(), fitted.SystemDelaySeconds()));
    const auto noDelayReference = Measurement(referenceRange / halfC);
    Check("calibration reference zero delay supported", LidarRangeCalibration::FromReferenceMeasurement(noDelayReference, referenceRange).SystemDelaySeconds() == 0);
    Check("calibration zero range reference supported", LidarRangeCalibration::FromReferenceMeasurement(Measurement(2e-9), 0).SystemDelaySeconds() == 2e-9);
    Check("calibration rejects negative true reference range", Throws<std::invalid_argument>([&]{ LidarRangeCalibration::FromReferenceMeasurement(reference, -1); }));
    Check("calibration rejects NaN true reference range", Throws<std::invalid_argument>([&]{ LidarRangeCalibration::FromReferenceMeasurement(reference, nan); }));
    Check("calibration rejects infinite true reference range", Throws<std::invalid_argument>([&]{ LidarRangeCalibration::FromReferenceMeasurement(reference, inf); }));
    invalid = reference; invalid.arrivalTimeSeconds = inf;
    Check("calibration factory rejects nonfinite reference arrival", Throws<std::invalid_argument>([&]{ LidarRangeCalibration::FromReferenceMeasurement(invalid, referenceRange); }));
    invalid = reference; invalid.timeOfFlightSeconds = nan;
    Check("calibration factory rejects nonfinite reference time", Throws<std::invalid_argument>([&]{ LidarRangeCalibration::FromReferenceMeasurement(invalid, referenceRange); }));
    invalid = reference; invalid.timeOfFlightSeconds = -1;
    Check("calibration factory rejects negative reference time", Throws<std::invalid_argument>([&]{ LidarRangeCalibration::FromReferenceMeasurement(invalid, referenceRange); }));
    invalid = reference; invalid.rangeMeters = inf;
    Check("calibration factory rejects nonfinite reference raw range", Throws<std::invalid_argument>([&]{ LidarRangeCalibration::FromReferenceMeasurement(invalid, referenceRange); }));
    invalid = reference; invalid.rangeMeters = -1;
    Check("calibration factory rejects negative reference raw range", Throws<std::invalid_argument>([&]{ LidarRangeCalibration::FromReferenceMeasurement(invalid, referenceRange); }));
    invalid = reference; invalid.peakOutOfRange = true;
    Check("calibration factory rejects peak clipping", Throws<std::invalid_argument>([&]{ LidarRangeCalibration::FromReferenceMeasurement(invalid, referenceRange); }));
    invalid = reference; invalid.searchHasOutOfRangeSamples = true;
    Check("calibration factory rejects nonpeak search clipping", Throws<std::invalid_argument>([&]{ LidarRangeCalibration::FromReferenceMeasurement(invalid, referenceRange); }));
    Check("calibration negative inferred delay explicitly rejected", Throws<std::domain_error>([&]{ LidarRangeCalibration::FromReferenceMeasurement(Measurement(referenceRange / halfC - 1e-9), referenceRange); }));
    const double largeRange = .75 * maximum;
    const auto largeReference = Measurement(largeRange / halfC);
    Check("calibration large range avoids twice range overflow", LidarRangeCalibration::FromReferenceMeasurement(largeReference, largeRange).SystemDelaySeconds() == 0);

    // Independent shifted pulse, not the calibration pulse reused as a target.
    // Same shape, response, ADC and relative window; no real-world universality claim.
    const LidarDigitalRangeEstimator estimator(0, 4);
    const auto referenceSignal = Signal(100.2e-9);
    const auto targetSignal = Signal(170.2e-9);
    const auto referenceMeasurement = estimator.EstimateCentroid(referenceSignal, 0, LidarReturnWindow{80e-9, 140e-9});
    const auto targetMeasurement = estimator.EstimateCentroid(targetSignal, 10e-9, LidarReturnWindow{150e-9, 210e-9});
    Check("calibration full chain reference and target detected", referenceMeasurement && targetMeasurement);
    if (!referenceMeasurement || !targetMeasurement) return;
    const auto chainCalibration = LidarRangeCalibration::FromReferenceMeasurement(*referenceMeasurement, halfC * 100.2e-9);
    const auto chainCorrected = chainCalibration.Apply(*targetMeasurement);
    Check("calibration full chain derived delay is positive", chainCalibration.SystemDelaySeconds() > 0);
    Check("calibration full chain transfers to shifted independent pulse", chainCorrected && std::abs(chainCorrected->correctedRangeMeters - halfC * 160.2e-9) < 1e-8);
    Check("calibration full chain retains measured result and flags", chainCorrected && Same(chainCorrected->measured, *targetMeasurement));
    Check("calibration full chain uncorrected target has positive bias", targetMeasurement->rangeMeters > halfC * 160.2e-9);
    std::cout << "[INFO] calibration fitted delay ns=" << chainCalibration.SystemDelaySeconds()*1e9
        << " target raw range m=" << targetMeasurement->rangeMeters
        << " target corrected range m=" << (chainCorrected ? chainCorrected->correctedRangeMeters : -1) << '\n';
}
