#include "lidar/signal_processing/LidarDigitalPeakDetector.h"
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
template<class E, class F> bool Throws(F f)
{
    try { f(); } catch (const E&) { return true; }
    return false;
}
LidarDigitalWaveform Codes(const std::vector<std::uint32_t>& codes, double start = 0, double dt = 1)
{
    const IdealUniformAdc adc(12, 0, 4096);
    std::vector<double> voltage;
    for (const auto code : codes) voltage.push_back(adc.CodeCenterVoltageV(code));
    return LidarDigitalWaveform::FromVoltageWaveform(
        LidarVoltageWaveform({start, dt, codes.size()}, voltage), adc);
}
std::vector<LidarDigitalPeak> Detect(const std::vector<std::uint32_t>& codes,
    double separation = 0, double baseline = 0, double threshold = 0)
{
    return LidarDigitalPeakDetector({separation}).Detect(Codes(codes), 0,
        {0, double(codes.size())}, baseline, threshold);
}
bool Same(const std::vector<LidarDigitalPeak>& a, const std::vector<LidarDigitalPeak>& b)
{
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i].peakBinIndex != b[i].peakBinIndex || a[i].plateauFirstBin != b[i].plateauFirstBin ||
            a[i].plateauLastBin != b[i].plateauLastBin || a[i].peakCode != b[i].peakCode ||
            a[i].peakAboveBaselineCodes != b[i].peakAboveBaselineCodes ||
            a[i].plateauOutOfRange != b[i].plateauOutOfRange) return false;
    return true;
}
}

void RunLidarDigitalPeakDetectorAcceptanceTests()
{
    const auto one = Detect({0, 2, 9, 2, 0});
    Check("digital peaks single local maximum", one.size() == 1 && one[0].peakBinIndex == 2);
    Check("digital peaks single bin plateau bounds and amplitude", one.size() == 1 &&
        one[0].plateauFirstBin == 2 && one[0].plateauLastBin == 2 && one[0].peakCode == 9 &&
        one[0].peakAboveBaselineCodes == 9 && !one[0].plateauOutOfRange);
    const auto two = Detect({0, 9, 0, 4, 0});
    Check("digital peaks two resolved echoes returned chronologically", two.size() == 2 && two[0].peakBinIndex == 1 && two[1].peakBinIndex == 3);
    const auto ascendingStrength = Detect({0, 4, 0, 9, 0});
    Check("digital peaks chronological output not amplitude order", ascendingStrength.size() == 2 && ascendingStrength[0].peakCode == 4 && ascendingStrength[1].peakCode == 9);
    Check("digital peaks threshold equality rejected", Detect({0, 9, 0}, 0, 0, 9).empty());
    Check("digital peaks strictly above fractional threshold accepted", Detect({0, 9, 0}, 0, 0, 8.5).size() == 1);
    const auto fraction = Detect({100, 109, 100}, 0, 100.25, 8.5);
    Check("digital peaks fractional baseline amplitude preserved", fraction.size() == 1 && fraction[0].peakAboveBaselineCodes == 8.75);
    Check("digital peaks below baseline cannot underflow", Detect({100, 109, 100}, 0, 110, 0).empty());
    Check("digital peaks low amplitude noise rejected", Detect({100, 102, 100, 101, 100}, 0, 100, 2).empty());
    Check("digital peaks constant and zero waveforms yield no peaks", Detect({0, 0, 0}).empty() && Detect({10, 10, 10}).empty());
    Check("digital peaks monotonic records yield no peaks", Detect({0, 2, 4, 6}).empty() && Detect({6, 4, 2, 0}).empty());
    const auto odd = Detect({0, 9, 9, 9, 0});
    Check("digital peaks odd plateau represented once at midpoint", odd.size() == 1 && odd[0].peakBinIndex == 2 && odd[0].plateauFirstBin == 1 && odd[0].plateauLastBin == 3);
    const auto even = Detect({0, 9, 9, 0});
    Check("digital peaks even plateau chooses earlier midpoint", even.size() == 1 && even[0].peakBinIndex == 1 && even[0].plateauLastBin == 2);
    Check("digital peaks endpoint peaks excluded", Detect({9, 0, 9}).empty());
    Check("digital peaks endpoint plateaus excluded", Detect({9, 9, 0, 9, 9}).empty());
    Check("digital peaks fewer than three bins yield no complete peak", Detect({9}).empty() && Detect({0, 9}).empty());
    Check("digital peaks gap smaller than minimum suppresses weaker", Detect({0, 9, 0, 4, 0}, 3).size() == 1);
    const auto later = Detect({0, 4, 0, 9, 0}, 3);
    Check("digital peaks stronger later echo wins suppression", later.size() == 1 && later[0].peakBinIndex == 3);
    Check("digital peaks separation equality retains both", Detect({0, 9, 0, 4, 0}, 2).size() == 2);
    const auto tie = Detect({0, 9, 0, 9, 0}, 3);
    Check("digital peaks equal strength suppression favors earlier", tie.size() == 1 && tie[0].peakBinIndex == 1);
    const auto chain = Detect({0, 8, 0, 9, 0, 10, 0}, 3);
    Check("digital peaks greedy suppression checks all retained peaks", chain.size() == 2 && chain[0].peakBinIndex == 1 && chain[1].peakBinIndex == 5);
    const auto timeWave = Codes({0, 9, 0, 4, 0}, 0, .25e-9);
    Check("digital peaks separation uses seconds not bin count", LidarDigitalPeakDetector({.75e-9}).Detect(timeWave, 0, {0, 2e-9}, 0, 0).size() == 1);
    const auto clockWave = Codes({0, 9, 0, 4, 0}, 1000, .25e-9);
    Check("digital peaks clock shift preserves index based separation",
        Same(LidarDigitalPeakDetector({.75e-9}).Detect(timeWave, 0, {0, 2e-9}, 0, 0),
            LidarDigitalPeakDetector({.75e-9}).Detect(clockWave, 1000, {1000, 1000 + 2e-9}, 0, 0)));
    const LidarDigitalPeakDetector detector;
    const auto gated = Codes({0, 9, 0, 4, 0});
    Check("digital peaks pre emission peak excluded", detector.Detect(gated, 2, {0, 5}, 0, 0).size() == 1);
    Check("digital peaks search start includes equal bin center as neighbor",
        detector.Detect(gated, 0, {.5, 4.5}, 0, 0).size() == 1);
    // End center is excluded: second peak touches the effective interval edge.
    Check("digital peaks half open window excludes end center and truncated peak",
        detector.Detect(gated, 0, {0, 3.5}, 0, 0).size() == 1);
    Check("digital peaks window clipped plateau not falsely complete",
        detector.Detect(Codes({0, 9, 9, 9, 0}), 0, {2, 5}, 0, 0).empty());
    Check("digital peaks disjoint search returns empty", detector.Detect(gated, 0, {10, 20}, 0, 0).empty());
    Check("digital peaks no post emission bins returns empty", detector.Detect(gated, 10, {0, 5}, 0, 0).empty());
    Check("digital peaks threshold beyond available span is valid no detection", detector.Detect(gated, 0, {0, 5}, 0, 5000).empty());
    Check("digital peaks baseline at maximum yields no detection", detector.Detect(gated, 0, {0, 5}, 4095, 0).empty());
    const IdealUniformAdc adc(12, 0, 4096);
    const auto clipped = LidarDigitalWaveform::FromVoltageWaveform(
        LidarVoltageWaveform({0, 1, 4}, {.5, 4095.5, 5000, .5}), adc);
    const auto clipping = detector.Detect(clipped, 0, {0, 4}, 0, 0);
    Check("digital peaks clipping aggregated across entire plateau", clipping.size() == 1 && clipping[0].peakBinIndex == 1 && !clipped.Samples()[1].aboveRange && clipping[0].plateauOutOfRange);
    const auto maximum = Detect({0, 4095, 4095, 0});
    Check("digital peaks maximum ADC code alone is not clipping", maximum.size() == 1 && !maximum[0].plateauOutOfRange);
    const auto before = gated.Samples();
    const auto replay = detector.Detect(gated, 0, {0, 5}, 0, 0);
    Check("digital peaks repeated detection deterministic", Same(replay, detector.Detect(gated, 0, {0, 5}, 0, 0)));
    bool unchanged = before.size() == gated.Samples().size();
    for (std::size_t i = 0; i < before.size(); ++i) unchanged = unchanged && before[i].code == gated.Samples()[i].code;
    Check("digital peaks input waveform unchanged", unchanged);
    const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
    Check("digital peaks negative separation rejected", Throws<std::invalid_argument>([] { LidarDigitalPeakDetector d({-1}); }));
    Check("digital peaks NaN separation rejected", Throws<std::invalid_argument>([&] { LidarDigitalPeakDetector d({nan}); }));
    Check("digital peaks infinite separation rejected", Throws<std::invalid_argument>([&] { LidarDigitalPeakDetector d({inf}); }));
    Check("digital peaks negative emission rejected", Throws<std::invalid_argument>([&] { detector.Detect(gated, -1, {0, 5}, 0, 0); }));
    Check("digital peaks NaN emission rejected", Throws<std::invalid_argument>([&] { detector.Detect(gated, nan, {0, 5}, 0, 0); }));
    Check("digital peaks infinite emission rejected", Throws<std::invalid_argument>([&] { detector.Detect(gated, inf, {0, 5}, 0, 0); }));
    Check("digital peaks reversed search rejected", Throws<std::invalid_argument>([&] { detector.Detect(gated, 0, {5, 0}, 0, 0); }));
    Check("digital peaks zero duration search rejected", Throws<std::invalid_argument>([&] { detector.Detect(gated, 0, {1, 1}, 0, 0); }));
    Check("digital peaks nonfinite search rejected", Throws<std::invalid_argument>([&] { detector.Detect(gated, 0, {0, inf}, 0, 0); }));
    Check("digital peaks negative baseline rejected", Throws<std::invalid_argument>([&] { detector.Detect(gated, 0, {0, 5}, -1, 0); }));
    Check("digital peaks nonfinite baseline rejected", Throws<std::invalid_argument>([&] { detector.Detect(gated, 0, {0, 5}, nan, 0); }));
    Check("digital peaks baseline beyond ADC rejected", Throws<std::invalid_argument>([&] { detector.Detect(gated, 0, {0, 5}, 4096, 0); }));
    Check("digital peaks negative threshold rejected", Throws<std::invalid_argument>([&] { detector.Detect(gated, 0, {0, 5}, 0, -1); }));
    Check("digital peaks nonfinite threshold rejected", Throws<std::invalid_argument>([&] { detector.Detect(gated, 0, {0, 5}, 0, inf); }));
    Check("digital peaks collapsed absolute time bins rejected", Throws<std::domain_error>([&] {
        detector.Detect(Codes({0, 9, 0}, 1e10, .25e-9), 1e10, {1e10, 1e10 + 1}, 0, 0);
    }));

    // Real optical pulse -> detector -> causal response -> voltage -> ADC.
    // Detect output peaks, not geometric arrival time; receiver delay remains.
    LidarWaveform optical({0, .25e-9, 1024});
    const GaussianPulseProfile profile(4e-9);
    optical.AccumulateReturn({1e-12, 70e-9, 532}, profile);
    optical.AccumulateReturn({.6e-12, 100e-9, 532}, profile);
    const auto current = LidarAnalogWaveform::FromOpticalWaveform(optical, LinearPhotodetector(532, .6));
    const auto response = FirstOrderLidarResponse(2e-9).Apply(current);
    const auto volts = LidarVoltageWaveform::FromCurrentWaveform(response, IdealTransimpedanceAmplifier(1e4));
    const IdealUniformAdc chainAdc(12, 0, 1);
    const auto digital = LidarDigitalWaveform::FromVoltageWaveform(volts, chainAdc);
    const auto physical = LidarDigitalPeakDetector({15e-9}).Detect(digital, 0, {40e-9, 140e-9}, 0, 4);
    Check("digital peaks real two pulse ADC chain yields two candidates", physical.size() == 2 && !digital.HasOutOfRangeSamples());
    Check("digital peaks real chain candidates near delayed pulse locations", physical.size() == 2 &&
        std::abs(digital.BinCenterTimeSeconds(physical[0].peakBinIndex) - 70e-9) < 6e-9 &&
        std::abs(digital.BinCenterTimeSeconds(physical[1].peakBinIndex) - 100e-9) < 6e-9);
    Check("digital peaks large separation suppresses weaker real echo",
        LidarDigitalPeakDetector({50e-9}).Detect(digital, 0, {40e-9, 140e-9}, 0, 4).size() == 1);
}
