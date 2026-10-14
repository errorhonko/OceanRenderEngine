#include "lidar/scan/LidarMultiFrameDriver.h"
#include "lidar/output/LidarPcdWriter.h"
#include "lidar/emitter/beam/GaussianLidarBeamProfile.h"
#include "lidar/emitter/pulse/GaussianPulseProfile.h"
#include "reflection/lidar/LambertianLidarScattering.h"
#include "core/ray/HittableList.h"
#include "geometry/primitives/Triangle.h"
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{
void Check(const char* name, bool ok)
{
    if (!ok) { std::cerr << "[FAIL] " << name << '\n'; throw std::runtime_error(name); }
    std::cout << "[PASS] " << name << '\n';
}
template<class E, class F> bool Throws(F f) { try { f(); } catch (const E&) { return true; } return false; }
LidarReceiverEchoResult Echo(double range, std::size_t index)
{
    constexpr double hc = .5 * 299792458.0;
    LidarReceiverEchoResult result;
    result.estimate.peak = {index, index, index, 2080, 32, false};
    result.estimate.firstBinIndex = index - 1;
    result.estimate.lastBinIndex = index + 1;
    result.estimate.measured = {index, .125 + range / hc, range / hc, range, 2080, 32, false, false};
    return result;
}
void AddPlane(HittableList& world, float z, float xmax)
{
    const Vector3f a(-100, -100, z), b(xmax, -100, z), c(-100, 100, z), d(xmax, 100, z);
    const Vector3f n(0, 0, -1), uv(0, 0, 0);
    // Geometric normal comes from winding, not the supplied shading normals.
    world.add(std::make_shared<Triangle>(a, c, b, n, n, n, uv, uv, uv, nullptr));
    world.add(std::make_shared<Triangle>(c, d, b, n, n, n, uv, uv, uv, nullptr));
}
class TwoPulses final : public LidarScanPlan
{
public:
    std::size_t Size() const override { return 2; }
    LidarFiringEvent Firing(std::size_t i) const override
    {
        if (i >= 2) throw std::out_of_range("Plan index");
        return {100 + i, 70000, 1.0 + double(i) * 1e-4, {0, 0, 0}, {0, 0, 1}, std::nullopt};
    }
};
LidarScanDriverConfig Settings()
{
    LidarScanDriverConfig c;
    c.echoMode = LidarScanEchoMode::Multi;
    c.pulseEnergyJ = 1e-8f;
    c.samplesPerPulse = 64;
    c.raySamplingMode = LidarRaySamplingMode::Stratified2D;
    c.beamProfile = std::make_shared<GaussianLidarBeamProfile>(.005, .015);
    c.receiverFovHalfAngleRadians = .03f;
    c.reception.noiseWindow = {0, 20e-9};
    c.reception.searchWindow = {40e-9, 140e-9};
    c.reception.noiseSigmaVoltageV = 0;
    c.reception.adcBitCount = 16;
    c.reception.minimumThresholdCodes = 4;
    c.reception.calibration = LidarRangeCalibration(2e-9);
    c.reception.multiEcho.peakDetection.minimumPeakSeparationSeconds = 15e-9;
    c.rangeSource = LidarPointRangeSource::Corrected;
    return c;
}
}

void RunLidarMultiEchoPointCloudAcceptanceTests()
{
    const LidarPointCloudBuilder raw(LidarPointRangeSource::Raw), corrected(LidarPointRangeSource::Corrected);
    LidarReceiverMultiEchoResult reception;
    reception.echoes = {Echo(12, 5), Echo(18, 20)};
    LidarMultiEchoPulseMeasurement measurement{LidarPulseMetadata(501, .125, {1, 2, 3}, {0, 0, 1}), reception};
    const auto points = raw.BuildPoints(measurement);
    Check("multi points two ranges project independently on same center axis", points.size() == 2 && points[0].positionWorld.z == 15 && points[1].positionWorld.z == 21);
    Check("multi points detected return identity assigned", points.size() == 2 && points[0].returnNumber == 1 && points[1].returnNumber == 2 && points[0].returnCount == 2 && points[1].returnCount == 2 && points[0].pulseId == 501 && points[1].pulseId == 501);
    Check("multi points arrival timestamps preserved independently", points.size() == 2 && points[0].arrivalTimeSeconds == reception.echoes[0].estimate.measured.arrivalTimeSeconds && points[1].arrivalTimeSeconds == reception.echoes[1].estimate.measured.arrivalTimeSeconds);
    Check("multi points corrected source never falls back to raw", corrected.BuildPoints(measurement).empty());
    const LidarRangeCalibration calibration(2 / (.5 * 299792458.0));
    for (auto& echo : measurement.reception.echoes) echo.corrected = calibration.Apply(echo.estimate.measured);
    const auto calibrated = corrected.BuildPoints(measurement);
    Check("multi points corrected range selected for every echo", calibrated.size() == 2 && std::abs(calibrated[0].rangeMeters - 10) < 1e-12 && std::abs(calibrated[1].rangeMeters - 16) < 1e-12);
    measurement.reception.echoes[0].corrected.reset();
    const auto partialCorrection = corrected.BuildPoints(measurement);
    Check("multi points skipped correction preserves original numbering", partialCorrection.size() == 1 && partialCorrection[0].returnNumber == 2 && partialCorrection[0].returnCount == 2);
    measurement.reception.hasAdcOutOfRangeSamples = true;
    for (auto& echo : measurement.reception.echoes) echo.estimate.measured.searchHasOutOfRangeSamples = true;
    measurement.reception.echoes[0].estimate.peak.plateauOutOfRange = true;
    const auto clean = raw.BuildPoints(measurement);
    Check("multi points clipped plateau rejected without deleting clean neighbor", clean.size() == 1 && clean[0].returnNumber == 2 && clean[0].searchHasOutOfRangeSamples && clean[0].hasAdcOutOfRangeSamples);
    measurement.reception.echoes[0].estimate.peak.plateauOutOfRange = false;
    measurement.reception.echoes[0].estimate.localWindowHasOutOfRangeSamples = true;
    Check("multi points local tail clipping also rejected", raw.BuildPoints(measurement).size() == 1);
    const auto acceptedClipping = LidarPointCloudBuilder(LidarPointRangeSource::Raw, false).BuildPoints(measurement);
    Check("multi points clipping acceptance policy preserves local diagnostics", acceptedClipping.size() == 2 && acceptedClipping[0].localWindowHasOutOfRangeSamples && acceptedClipping[0].returnNumber == 1);
    measurement.reception.echoes.clear();
    Check("multi points no echoes produces no origin point", raw.BuildPoints(measurement).empty());

    HittableList world;
    AddPlane(world, 10, 0); // Left part of the footprint sees a near opaque target.
    AddPlane(world, 15, 100); // Right part reaches the unobstructed far target.
    const LambertianLidarScattering scattering(.5);
    const GaussianPulseProfile profile(4e-9);
    const TwoPulses plan;
    const LidarScanPoseConfig pose{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}};
    auto config = Settings();
    const LidarScanDriver driver(world, scattering, profile, plan, pose, config);
    const auto frame = driver.SimulateFrame();
    Check("multi scan real split footprint emits two pulses and four points", frame.pulses.size() == 2 && frame.points.size() == 4);
    bool counts = true, association = true, geometry = true, energy = true;
    for (const auto& pulse : frame.pulses)
    {
        counts = counts && pulse.multiReception && pulse.multiReception->echoes.size() == 2 && pulse.points.size() == 2 &&
            pulse.emittedRayCount == 64 && pulse.surfaceHitCount == 64 && pulse.returnSampleCount > 2;
        for (const auto& point : pulse.points)
            association = association && point.pulseId == pulse.scan.pulse.PulseId() && point.channelId == 70000 && point.positionSensor &&
                point.emissionTimeSeconds == pulse.scan.pulse.EmissionTimeSeconds();
        if (pulse.points.size() == 2)
            geometry = geometry && std::abs(pulse.points[0].rangeMeters - 10) < .05 && std::abs(pulse.points[1].rangeMeters - 15) < .05 &&
                pulse.points[0].positionWorld.x == 0 && pulse.points[1].positionWorld.x == 0;
        else geometry = false;
        energy = energy && pulse.receivedEnergyJ > 0 && std::abs(pulse.recordedEnergyJ / pulse.receivedEnergyJ - 1) < 1e-12;
    }
    Check("multi scan many MC rays collapse to two measured returns per pulse", counts);
    Check("multi scan channel time and sensor coordinates preserved on every return", association);
    Check("multi scan near and far corrected geometry within five cm", geometry);
    Check("multi scan preserves optical energy with no extra sample normalization", energy);
    Check("multi scan legacy single result not silently filled with first echo", !frame.pulses[0].point && !frame.pulses[0].reception.measured);
    const auto replay = driver.SimulatePulse(1);
    Check("multi scan individual replay matches frame return set", replay.points.size() == 2 && replay.points[0].rangeMeters == frame.pulses[1].points[0].rangeMeters && replay.points[1].rangeMeters == frame.pulses[1].points[1].rangeMeters);
    Check("multi scan flatten order follows pulse then return", frame.points.size() == 4 && frame.points[0].pulseId == 100 && frame.points[1].returnNumber == 2 && frame.points[2].pulseId == 101 && frame.points[3].returnNumber == 2);
    const LidarMultiFrameDriver sequence(world, scattering, profile, plan, pose, config, {2, .1, 0});
    const auto later = sequence.SimulateFrame(1);
    Check("multi scan continuous frames carry all returns and shifted pulse ids", later.cloud.points.size() == 4 && later.pulses.size() == 2 && later.cloud.points[0].pulseId == 102 && later.cloud.points[2].pulseId == 103);
    Check("multi scan later frame replay deterministic", sequence.SimulateFrame(1).cloud.points[1].rangeMeters == later.cloud.points[1].rangeMeters);
    const HittableList empty;
    const auto miss = LidarScanDriver(empty, scattering, profile, plan, pose, config).SimulateFrame();
    Check("multi scan misses retain multi reception diagnostics without points", miss.pulses.size() == 2 && miss.points.empty() && miss.pulses[0].multiReception && miss.pulses[0].multiReception->echoes.empty());
    auto legacyConfig = config;
    legacyConfig.echoMode = LidarScanEchoMode::Single;
    const auto legacy = LidarScanDriver(world, scattering, profile, plan, pose, legacyConfig).SimulateFrame();
    Check("multi scan single mode retains old optional point and unified singleton", legacy.points.size() == 2 && legacy.pulses[0].point && legacy.pulses[0].points.size() == 1 && !legacy.pulses[0].multiReception && legacy.points[0].returnCount == 1);
    Check("multi scan switching processing leaves tracing and noise seed keys unchanged", legacy.pulses[0].receivedEnergyJ == frame.pulses[0].receivedEnergyJ && legacy.pulses[0].noiseSeed == frame.pulses[0].noiseSeed && legacy.pulses[0].raySeed == frame.pulses[0].raySeed);
    auto independent = config;
    independent.raySamplingMode = LidarRaySamplingMode::Independent;
    Check("multi scan independent ray sampler also supports multiple measured points", LidarScanDriver(world, scattering, profile, plan, pose, independent).SimulatePulse(0).points.size() == 2);
    auto bad = config;
    bad.echoMode = static_cast<LidarScanEchoMode>(999);
    Check("multi scan unknown echo mode rejected", Throws<std::invalid_argument>([&] { LidarScanDriver d(world, scattering, profile, plan, pose, bad); }));

    std::ostringstream output;
    LidarPcdWriter::Write(output, later.cloud);
    Check("multi PCD accepts multiple distinct return ids from same pulse", output.str().find("POINTS 4\n") != std::string::npos);
    Check("multi PCD exports original detected return identity fields", output.str().find("return_number return_count\n") != std::string::npos && output.str().find("detected count before filtering") != std::string::npos);
    std::istringstream input(output.str());
    std::string line;
    while (std::getline(input, line) && line != "DATA ascii") {}
    bool roundtrip = true;
    std::size_t rowIndex = 0;
    while (std::getline(input, line))
    {
        if (line.empty()) continue;
        std::istringstream row(line);
        double x, y, z, intensity, time, range, amplitude;
        std::uint32_t ring, peak, low, high, flags, source, number, count;
        if (!(row >> x >> y >> z >> intensity >> ring >> time >> range >> amplitude >> peak >> low >> high >> flags >> source >> number >> count))
            throw std::runtime_error("PCD row parse failed");
        const auto& p = later.cloud.points.at(rowIndex++);
        roundtrip = roundtrip && number == p.returnNumber && count == p.returnCount && ring == p.channelId &&
            ((std::uint64_t(high) << 32) | low) == p.pulseId && range == p.rangeMeters;
    }
    Check("multi PCD return metadata independently round trips", roundtrip && rowIndex == 4);
    output.str(""); output.clear();
    LidarPcdWriter::Write(output, later.cloud, LidarPcdCoordinates::SensorAtEmission);
    Check("multi PCD sensor frame export supports every return", output.str().find("sensor_at_emission") != std::string::npos);
    auto invalid = later.cloud;
    invalid.points[1].returnNumber = invalid.points[0].returnNumber;
    Check("multi PCD duplicate pulse return pair rejected", Throws<std::invalid_argument>([&] { LidarPcdWriter::Write(output, invalid); }));
    invalid = later.cloud; invalid.points[0].returnNumber = 0;
    Check("multi PCD zero return number rejected", Throws<std::invalid_argument>([&] { LidarPcdWriter::Write(output, invalid); }));
    invalid = later.cloud; invalid.points[0].returnNumber = 3;
    Check("multi PCD return number beyond detected count rejected", Throws<std::invalid_argument>([&] { LidarPcdWriter::Write(output, invalid); }));
    invalid = later.cloud; invalid.points[1].returnCount = 3;
    Check("multi PCD inconsistent count within one pulse rejected", Throws<std::invalid_argument>([&] { LidarPcdWriter::Write(output, invalid); }));
    invalid = later.cloud; invalid.points[1].channelId = 0;
    Check("multi PCD inconsistent channel within one pulse rejected", Throws<std::invalid_argument>([&] { LidarPcdWriter::Write(output, invalid); }));
    auto filtered = later.cloud;
    filtered.points = {later.cloud.points[1]};
    output.str(""); output.clear();
    LidarPcdWriter::Write(output, filtered);
    Check("multi PCD filtered numbering gaps remain legal", output.str().find("POINTS 1\n") != std::string::npos && filtered.points[0].returnNumber == 2);
    auto flagged = later.cloud;
    flagged.points.resize(1);
    flagged.points[0].peakOutOfRange = false;
    flagged.points[0].searchHasOutOfRangeSamples = true;
    flagged.points[0].hasAdcOutOfRangeSamples = true;
    flagged.points[0].plateauOutOfRange = true;
    flagged.points[0].localWindowHasOutOfRangeSamples = true;
    output.str(""); output.clear();
    LidarPcdWriter::Write(output, flagged);
    Check("multi PCD exports plateau and local clipping flag bits", output.str().find(" 30 1 1 2\n") != std::string::npos);
    invalid = later.cloud; invalid.points[1].emissionTimeSeconds += 1e-5;
    Check("multi PCD inconsistent emission time within one pulse rejected", Throws<std::invalid_argument>([&] { LidarPcdWriter::Write(output, invalid); }));
}

void RunLidarReturnSelectionAcceptanceTests()
{
    LidarReceiverMultiEchoResult reception;
    reception.echoes = {Echo(12, 5), Echo(18, 20), Echo(24, 35)};
    const double amplitudes[] = {10, 30, 20};
    for (std::size_t i = 0; i < 3; ++i)
    {
        reception.echoes[i].estimate.measured.peakAboveBaselineCodes = amplitudes[i];
        reception.echoes[i].estimate.peak.peakAboveBaselineCodes = amplitudes[i];
    }
    LidarMultiEchoPulseMeasurement measurement{LidarPulseMetadata(501, .125, {0, 0, 0}, {0, 0, 1}), reception};
    const LidarPointCloudBuilder all(LidarPointRangeSource::Raw);
    const LidarPointCloudBuilder first(LidarPointRangeSource::Raw, true, LidarReturnSelection::First);
    const LidarPointCloudBuilder strongest(LidarPointRangeSource::Raw, true, LidarReturnSelection::Strongest);
    const LidarPointCloudBuilder last(LidarPointRangeSource::Raw, true, LidarReturnSelection::Last);
    Check("return selection all remains backwards compatible default", all.ReturnSelection() == LidarReturnSelection::All && all.BuildPoints(measurement).size() == 3);
    auto f = first.BuildPoints(measurement), s = strongest.BuildPoints(measurement), l = last.BuildPoints(measurement);
    Check("return selection first uses earliest valid return", f.size() == 1 && f[0].returnNumber == 1 && f[0].rangeMeters == 12);
    Check("return selection strongest uses baseline subtracted code amplitude", s.size() == 1 && s[0].returnNumber == 2 && s[0].peakAboveBaselineCodes == 30);
    Check("return selection last uses latest valid return", l.size() == 1 && l[0].returnNumber == 3 && l[0].rangeMeters == 24);
    Check("return selection does not renumber selected point or detected count", s.size() == 1 && s[0].returnCount == 3 && s[0].pulseId == 501);
    Check("return selection leaves original diagnostics unchanged", measurement.reception.echoes.size() == 3 && measurement.reception.echoes[1].estimate.measured.peakAboveBaselineCodes == 30);
    auto tied = measurement;
    tied.reception.echoes[0].estimate.measured.peakAboveBaselineCodes = 30;
    const auto tie = strongest.BuildPoints(tied);
    Check("return selection equal strength favors earlier return", tie.size() == 1 && tie[0].returnNumber == 1);
    auto clipped = measurement;
    clipped.reception.echoes[1].estimate.peak.plateauOutOfRange = true;
    const auto cleanStrongest = strongest.BuildPoints(clipped);
    Check("return selection strongest occurs after clipping rejection", cleanStrongest.size() == 1 && cleanStrongest[0].returnNumber == 3);
    clipped = measurement; clipped.reception.echoes[0].estimate.localWindowHasOutOfRangeSamples = true;
    const auto cleanFirst = first.BuildPoints(clipped);
    Check("return selection first advances to earliest eligible echo", cleanFirst.size() == 1 && cleanFirst[0].returnNumber == 2);
    clipped = measurement; clipped.reception.echoes[2].estimate.localWindowHasOutOfRangeSamples = true;
    const auto cleanLast = last.BuildPoints(clipped);
    Check("return selection last advances to latest eligible echo", cleanLast.size() == 1 && cleanLast[0].returnNumber == 2);
    for (auto& echo : clipped.reception.echoes) echo.estimate.peak.plateauOutOfRange = true;
    Check("return selection no eligible echo never creates a point", first.BuildPoints(clipped).empty() && strongest.BuildPoints(clipped).empty() && last.BuildPoints(clipped).empty());
    const auto allowed = LidarPointCloudBuilder(LidarPointRangeSource::Raw, false, LidarReturnSelection::Strongest).BuildPoints(clipped);
    Check("return selection optional clipping acceptance still selects strongest", allowed.size() == 1 && allowed[0].returnNumber == 2 && allowed[0].plateauOutOfRange);
    auto correctedMeasurement = measurement;
    const LidarRangeCalibration calibration(2 / (.5 * 299792458.0));
    for (auto& echo : correctedMeasurement.reception.echoes) echo.corrected = calibration.Apply(echo.estimate.measured);
    correctedMeasurement.reception.echoes[1].corrected.reset();
    const auto corrected = LidarPointCloudBuilder(LidarPointRangeSource::Corrected, true, LidarReturnSelection::Strongest).BuildPoints(correctedMeasurement);
    Check("return selection skips unavailable correction before amplitude ranking", corrected.size() == 1 && corrected[0].returnNumber == 3 && std::abs(corrected[0].rangeMeters - 22) < 1e-12);
    auto missing = measurement; missing.reception.echoes.clear();
    Check("return selection all policies safely handle zero candidates", all.BuildPoints(missing).empty() && first.BuildPoints(missing).empty() && strongest.BuildPoints(missing).empty() && last.BuildPoints(missing).empty());
    auto only = measurement; only.reception.echoes.resize(1);
    Check("return selection all policies agree for one eligible echo", first.BuildPoints(only)[0].rangeMeters == last.BuildPoints(only)[0].rangeMeters && strongest.BuildPoints(only)[0].returnCount == 1);
    Check("return selection invalid enum rejected by builder", Throws<std::invalid_argument>([] { LidarPointCloudBuilder b(LidarPointRangeSource::Raw, true, static_cast<LidarReturnSelection>(999)); }));

    HittableList world; AddPlane(world, 10, 0); AddPlane(world, 15, 100);
    const LambertianLidarScattering scattering(.5);
    const GaussianPulseProfile profile(4e-9);
    const TwoPulses plan;
    const LidarScanPoseConfig pose{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}};
    auto config = Settings();
    const auto reference = LidarScanDriver(world, scattering, profile, plan, pose, config).SimulateFrame();
    bool counts = true, identities = true, diagnostics = true, unchangedEnergy = true, replay = true;
    for (const auto policy : {LidarReturnSelection::All, LidarReturnSelection::First, LidarReturnSelection::Strongest, LidarReturnSelection::Last})
    {
        config.returnSelection = policy;
        const LidarScanDriver driver(world, scattering, profile, plan, pose, config);
        const auto frame = driver.SimulateFrame();
        counts = counts && frame.points.size() == (policy == LidarReturnSelection::All ? 4 : 2);
        for (std::size_t i = 0; i < frame.pulses.size(); ++i)
        {
            const auto& pulse = frame.pulses[i];
            diagnostics = diagnostics && pulse.multiReception && pulse.multiReception->echoes.size() == 2 && !pulse.point;
            unchangedEnergy = unchangedEnergy && pulse.receivedEnergyJ == reference.pulses[i].receivedEnergyJ && pulse.recordedEnergyJ == reference.pulses[i].recordedEnergyJ && pulse.raySeed == reference.pulses[i].raySeed && pulse.noiseSeed == reference.pulses[i].noiseSeed;
            for (const auto& point : pulse.points) identities = identities && point.returnCount == 2 && point.channelId == 70000 && point.positionSensor;
            if (policy != LidarReturnSelection::All)
                identities = identities && pulse.points.size() == 1 && pulse.points[0].returnNumber == (policy == LidarReturnSelection::Last ? 2 : 1);
            const auto repeated = driver.SimulatePulse(i);
            replay = replay && repeated.points.size() == pulse.points.size();
            for (std::size_t j = 0; j < pulse.points.size(); ++j)
                replay = replay && repeated.points[j].rangeMeters == pulse.points[j].rangeMeters && repeated.points[j].returnNumber == pulse.points[j].returnNumber;
        }
    }
    Check("return selection scan frame yields expected selected point counts", counts);
    Check("return selection real first strongest last identity and metadata correct", identities);
    Check("return selection preserves all detected echo diagnostics", diagnostics);
    Check("return selection never changes tracing energy normalization or seed keys", unchangedEnergy);
    Check("return selection standalone pulse replay matches selected frame points", replay);
    config.returnSelection = LidarReturnSelection::Last;
    const LidarMultiFrameDriver sequence(world, scattering, profile, plan, pose, config, {2, .1, 0});
    const auto later = sequence.SimulateFrame(1);
    Check("return selection propagates through continuous multi frame scan", later.cloud.points.size() == 2 && later.cloud.points[0].pulseId == 102 && later.cloud.points[0].returnNumber == 2 && later.cloud.points[0].returnCount == 2);
    std::ostringstream output;
    LidarPcdWriter::Write(output, later.cloud);
    Check("return selection PCD retains numbering gaps and detected counts", output.str().find("POINTS 2\n") != std::string::npos && output.str().find(" 2 2\n") != std::string::npos);
    auto bad = config; bad.returnSelection = static_cast<LidarReturnSelection>(999);
    Check("return selection unknown scan policy rejected before tracing", Throws<std::invalid_argument>([&] { LidarScanDriver d(world, scattering, profile, plan, pose, bad); }));
    bad = config; bad.echoMode = LidarScanEchoMode::Single;
    Check("return selection single mode cannot pretend to select detected echoes", Throws<std::invalid_argument>([&] { LidarScanDriver d(world, scattering, profile, plan, pose, bad); }));
    bad.returnSelection = LidarReturnSelection::All;
    Check("return selection legacy single default remains available", LidarScanDriver(world, scattering, profile, plan, pose, bad).SimulateFrame().points.size() == 2);
}
