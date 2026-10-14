#include "lidar/scan/LidarScanDriver.h"
#include "lidar/scan/LidarMultiFrameDriver.h"
#include "lidar/emitter/beam/GaussianLidarBeamProfile.h"
#include "lidar/emitter/pulse/GaussianPulseProfile.h"
#include "reflection/lidar/LambertianLidarScattering.h"
#include "core/ray/HittableList.h"
#include "geometry/primitives/Triangle.h"
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
void Check(const char* name, bool ok) {
    if (!ok) throw std::runtime_error(std::string("[FAIL] ") + name);
    std::cout << "[PASS] " << name << '\n';
}
template<class E, class F> bool Throws(F f) {
    try { f(); } catch (const E&) { return true; } return false;
}
HittableList Plane(float left = -100) {
    HittableList world;
    const Vector3f a(left,0,-100), b(left,0,100), c(100,0,-100), d(100,0,100);
    const Vector3f n(0,1,0), uv(0,0,0);
    world.add(std::make_shared<Triangle>(a,b,c,n,n,n,uv,uv,uv,nullptr));
    world.add(std::make_shared<Triangle>(c,b,d,n,n,n,uv,uv,uv,nullptr));
    return world;
}
LidarScanDriverConfig Settings() {
    LidarScanDriverConfig c;
    c.scan.originWorld = {0,10,0};
    c.scan.azimuthCount = 4; c.scan.elevationCount = 3;
    c.scan.azimuthMinRadians = -.2; c.scan.azimuthMaxRadians = .2;
    c.scan.elevationMinRadians = -.15; c.scan.elevationMaxRadians = .15;
    c.scan.firstPulseId = 100; c.scan.startTimeSeconds = 1.00000001;
    c.scan.pulseIntervalSeconds = 1e-4;
    c.pulseEnergyJ = 1e-8f; c.samplesPerPulse = 5;
    c.reception.noiseWindow = {0,20e-9};
    c.reception.searchWindow = {40e-9,115e-9};
    c.reception.noiseSigmaVoltageV = 0;
    c.reception.adcBitCount = 16;
    c.reception.rangeMethod = LidarReceiverRangeMethod::Centroid;
    c.reception.calibration = LidarRangeCalibration(2e-9);
    c.rangeSource = LidarPointRangeSource::Corrected;
    return c;
}
bool SameMeasurement(const LidarScanPulseResult& a, const LidarScanPulseResult& b) {
    if (a.raySeed != b.raySeed || a.noiseSeed != b.noiseSeed || a.receivedEnergyJ != b.receivedEnergyJ ||
        a.recordedEnergyJ != b.recordedEnergyJ || bool(a.point) != bool(b.point) ||
        bool(a.reception.measured) != bool(b.reception.measured)) return false;
    if (a.reception.measured && (a.reception.measured->arrivalTimeSeconds != b.reception.measured->arrivalTimeSeconds ||
        a.reception.measured->peakCode != b.reception.measured->peakCode)) return false;
    return !a.point || (a.point->rangeMeters == b.point->rangeMeters && a.point->pulseId == b.point->pulseId);
}
}

void RunLidarScanDriverAcceptanceTests() {
    const auto world = Plane();
    const LambertianLidarScattering scattering(.5), dark(0);
    const GaussianPulseProfile profile(4e-9);
    auto c = Settings();
    const LidarScanDriver driver(world,scattering,profile,c);
    const auto frame = driver.SimulateFrame();
    Check("scan driver frame retains all twelve pulse records",driver.Size()==12 && frame.pulses.size()==12);
    Check("scan driver actual plane chain detects twelve points",frame.points.size()==12);
    bool geometry=true,association=true,counts=true,energy=true,directions=true;
    for (std::size_t i=0; i<frame.pulses.size(); ++i) {
        const auto& r=frame.pulses[i]; const auto& p=r.scan.pulse;
        const auto& direction=p.CenterDirectionWorld(); const double expected=-10/double(direction.y);
        geometry=geometry && r.point && std::abs(r.point->rangeMeters-expected)<.05 &&
            std::abs(r.point->positionWorld.y)<.05;
        association=association && r.scan.scanIndex==i && p.PulseId()==100+i && r.point &&
            r.point->pulseId==p.PulseId() && r.point->emissionTimeSeconds==p.EmissionTimeSeconds() &&
            frame.points[i].pulseId==p.PulseId();
        counts=counts && r.emittedRayCount==5 && r.surfaceHitCount==5 && r.returnSampleCount==5;
        energy=energy && r.receivedEnergyJ>0 && std::abs(r.recordedEnergyJ-r.receivedEnergyJ)<r.receivedEnergyJ*1e-12;
        directions=directions && r.point && r.point->centerDirectionWorld.x==direction.x &&
            std::abs(r.point->positionWorld.x-r.point->rangeMeters*direction.x)<1e-5 &&
            std::abs(r.point->positionWorld.z-r.point->rangeMeters*direction.z)<1e-5;
    }
    Check("scan driver measured plane range and height within five cm",geometry);
    Check("scan driver metadata and point association preserved",association);
    Check("scan driver five MC rays produce one point per pulse",counts && frame.points.size()==frame.pulses.size());
    Check("scan driver temporal accumulation preserves return energy",energy);
    Check("scan driver projects measured range along scan axis",directions);
    Check("scan driver reception windows follow each nonzero emission time",frame.pulses.back().reception.measured &&
        frame.pulses.back().reception.measured->arrivalTimeSeconds>frame.pulses.back().scan.pulse.EmissionTimeSeconds());
    Check("scan driver ray seeds differ between pulses",frame.pulses[0].raySeed!=frame.pulses[1].raySeed);
    Check("scan driver noise seeds differ between pulses",frame.pulses[0].noiseSeed!=frame.pulses[1].noiseSeed);
    Check("scan driver individual pulse replay equals frame record",SameMeasurement(driver.SimulatePulse(7),frame.pulses[7]));
    Check("scan driver repeated frame has identical measurements",SameMeasurement(driver.SimulateFrame().pulses[4],frame.pulses[4]));
    Check("scan driver out of range pulse index rejected",Throws<std::out_of_range>([&]{driver.SimulatePulse(12);}));

    // 用同一 seed 独立调用底层真实追踪，核对转向、接收光轴以及 1/N。
    const auto& r=frame.pulses[0]; const auto& p=r.scan.pulse;
    const LaserEmitter emitter(p.OriginWorld(),p.CenterDirectionWorld(),c.wavelengthNm,c.pulseEnergyJ,0);
    const LidarReceiver receiver(p.OriginWorld(),p.CenterDirectionWorld(),c.receiverFovHalfAngleRadians,
        c.receiverApertureAreaSquareMeters,c.receiverOpticalEfficiency);
    IndependentSampler sampler(r.raySeed);
    const auto pulse=LidarIntegrator(world,emitter,receiver,c.samplesPerPulse,scattering).SimulatePulse(sampler,p.EmissionTimeSeconds());
    Check("scan driver optics and energy equal standalone integrator",pulse.TotalReceivedEnergyJ()==r.receivedEnergyJ);
    auto one=c; one.samplesPerPulse=1;
    const auto single=LidarScanDriver(world,scattering,profile,one).SimulatePulse(0);
    Check("scan driver does not divide MC energy twice",std::abs(single.receivedEnergyJ-r.receivedEnergyJ)<r.receivedEnergyJ*1e-12);
    auto raw=c; raw.rangeSource=LidarPointRangeSource::Raw; raw.reception.calibration.reset();
    const auto rawResult=LidarScanDriver(world,scattering,profile,raw).SimulatePulse(0);
    Check("scan driver raw range mode works without calibration",rawResult.point && rawResult.point->rangeSource==LidarPointRangeSource::Raw);
    Check("scan driver corrected point uses calibrated range only",r.point && r.reception.corrected &&
        r.point->rangeMeters==r.reception.corrected->correctedRangeMeters &&
        std::abs(rawResult.point->rangeMeters-r.point->rangeMeters-.5*299792458.0*2e-9)<1e-8);
    const HittableList empty;
    const auto missed=LidarScanDriver(empty,scattering,profile,c).SimulateFrame();
    bool misses=missed.pulses.size()==12 && missed.points.empty();
    for (const auto& m:missed.pulses) misses=misses && !m.point && !m.reception.measured && m.surfaceHitCount==0 && m.recordedEnergyJ==0;
    Check("scan driver empty scene retains misses without origin points",misses);
    const auto half=Plane(0);
    const auto mixed=LidarScanDriver(half,scattering,profile,c).SimulateFrame();
    Check("scan driver partial plane retains twelve records and six points",mixed.pulses.size()==12 && mixed.points.size()==6);
    const auto black=LidarScanDriver(world,dark,profile,c).SimulatePulse(0);
    Check("scan driver zero reflectance retains hit but no measured point",black.surfaceHitCount==5 && black.returnSampleCount==0 && !black.point);
    auto clipped=c; clipped.pulseEnergyJ=1e-5f;
    const auto saturated=LidarScanDriver(world,scattering,profile,clipped).SimulatePulse(0);
    Check("scan driver clipped echo diagnostics retained but point rejected",saturated.reception.measured && saturated.reception.hasAdcOutOfRangeSamples && !saturated.point);
    clipped.rejectClippedReturns=false;
    Check("scan driver clipping rejection policy configurable",LidarScanDriver(world,scattering,profile,clipped).SimulatePulse(0).point.has_value());
    auto snake=c; snake.scan.serpentine=true;
    const auto snakeFrame=LidarScanDriver(world,scattering,profile,snake).SimulateFrame();
    Check("scan driver serpentine row reverses spatial columns not pulse ids",snakeFrame.pulses[4].scan.firing.raster->azimuthIndex==3 && snakeFrame.pulses[7].scan.firing.raster->azimuthIndex==0 && snakeFrame.pulses[4].scan.pulse.PulseId()==104);
    auto cone=c; cone.divergenceHalfAngleRadians=.02f; cone.samplesPerPulse=128; cone.reception.noiseSigmaVoltageV=1e-4;
    const LidarScanDriver noisy(world,scattering,profile,cone);
    const auto conePulse=noisy.SimulatePulse(0);
    Check("scan driver divergent noisy beam still yields at most one point",conePulse.returnSampleCount>1 && conePulse.point.has_value());
    Check("scan driver MC and noise replay reproducible",SameMeasurement(conePulse,noisy.SimulatePulse(0)));
    auto changed=cone; changed.noiseSeed=999;
    const auto changedPulse=LidarScanDriver(world,scattering,profile,changed).SimulatePulse(0);
    Check("scan driver changing noise seed leaves optical energy unchanged",changedPulse.receivedEnergyJ==conePulse.receivedEnergyJ && changedPulse.raySeed==conePulse.raySeed && changedPulse.noiseSeed!=conePulse.noiseSeed);
    auto offset=c; offset.waveform.startTimeSeconds=10e-9; offset.reception.noiseWindow={10e-9,30e-9};
    const auto offsetPulse=LidarScanDriver(world,scattering,profile,offset).SimulatePulse(0);
    Check("scan driver nonzero relative record offset is supported",offsetPulse.point && std::abs(offsetPulse.point->rangeMeters-r.point->rangeMeters)<1e-5);
    auto zeroClock=c; zeroClock.scan.startTimeSeconds=0;
    const auto zeroPulse=LidarScanDriver(world,scattering,profile,zeroClock).SimulatePulse(0);
    Check("scan driver clock shift preserves geometry energy and measured range",zeroPulse.receivedEnergyJ==r.receivedEnergyJ && zeroPulse.point && std::abs(zeroPulse.point->rangeMeters-r.point->rangeMeters)<1e-5);
    Check("scan driver ray and noise keys depend on pulse id not clock",zeroPulse.raySeed==r.raySeed && zeroPulse.noiseSeed==r.noiseSeed);
    auto snapshot=c; const LidarScanDriver copied(world,scattering,profile,snapshot); snapshot.scan.azimuthCount=1;
    Check("scan driver owns configuration snapshot",copied.Size()==12 && copied.Config().scan.azimuthCount==4);

    auto bad=c; bad.samplesPerPulse=0;
    Check("scan driver zero samples rejected",Throws<std::invalid_argument>([&]{LidarScanDriver d(world,scattering,profile,bad);}));
    bad=c; bad.reception.wavelengthNm=1064;
    Check("scan driver mismatched wavelength rejected",Throws<std::invalid_argument>([&]{LidarScanDriver d(world,scattering,profile,bad);}));
    bad=c; bad.reception.calibration.reset();
    Check("scan driver missing corrected range calibration rejected",Throws<std::invalid_argument>([&]{LidarScanDriver d(world,scattering,profile,bad);}));
    bad=c; bad.reception.searchWindow.endTimeSeconds=1;
    Check("scan driver reception window outside record rejected",Throws<std::invalid_argument>([&]{LidarScanDriver d(world,scattering,profile,bad);}));
    bad=c; bad.reception.noiseWindow={0,.1e-9};
    Check("scan driver insufficient noise bins rejected before tracing",Throws<std::invalid_argument>([&]{LidarScanDriver d(world,scattering,profile,bad);}));
    bad=c; bad.scan.startTimeSeconds=1e10; bad.scan.pulseIntervalSeconds=.01;
    Check("scan driver large clock incompatible with ns bins rejected",Throws<std::invalid_argument>([&]{LidarScanDriver d(world,scattering,profile,bad);}));
    bad=c; bad.pulseEnergyJ=-1;
    Check("scan driver invalid optical settings reuse emitter validation",Throws<std::invalid_argument>([&]{LidarScanDriver d(world,scattering,profile,bad);}));
    bad=c; bad.reception.noiseWindow={0,80e-9};
    Check("scan driver overlapping reception windows rejected",Throws<std::invalid_argument>([&]{LidarScanDriver d(world,scattering,profile,bad);}));
}

void RunStratifiedScanDriverAcceptanceTests()
{
    const auto world = Plane();
    const LambertianLidarScattering scattering(.5);
    const GaussianPulseProfile profile(4e-9);
    auto config = Settings();
    config.samplesPerPulse = 64;
    config.beamProfile = std::make_shared<GaussianLidarBeamProfile>(.001, .003);
    Check("stratified scan independent remains default", config.raySamplingMode == LidarRaySamplingMode::Independent);
    const auto independent = LidarScanDriver(world, scattering, profile, config).SimulatePulse(0);
    config.raySamplingMode = LidarRaySamplingMode::Stratified2D;
    config.xStrata = 8;
    config.yStrata = 8;
    const LidarScanDriver driver(world, scattering, profile, config);
    const auto frame = driver.SimulateFrame();
    Check("stratified scan full frame retains twelve pulses and twelve points", frame.pulses.size() == 12 && frame.points.size() == 12);
    bool counts = true, energy = true, replay = true, ids = true;
    for (std::size_t i = 0; i < frame.pulses.size(); ++i)
    {
        const auto& r = frame.pulses[i];
        counts = counts && r.emittedRayCount == 64 && r.surfaceHitCount == 64 && r.returnSampleCount == 64;
        energy = energy && r.receivedEnergyJ > 0 &&
            std::abs(r.recordedEnergyJ / r.receivedEnergyJ - 1) < 1e-12;
        replay = replay && SameMeasurement(r, driver.SimulatePulse(i));
        ids = ids && r.point && r.point->pulseId == 100 + i && frame.points[i].pulseId == 100 + i;
    }
    Check("stratified scan every pulse gets a fresh complete 64 cell batch", counts);
    Check("stratified scan temporal accumulation preserves optical energy", energy);
    Check("stratified scan individual replay matches every frame pulse", replay);
    Check("stratified scan metadata and point association unchanged", ids);
    const auto again = driver.SimulateFrame();
    bool sameFrame = again.pulses.size() == frame.pulses.size();
    for (std::size_t i = 0; i < frame.pulses.size(); ++i)
        sameFrame = sameFrame && SameMeasurement(frame.pulses[i], again.pulses[i]);
    Check("stratified scan repeated frame replays exactly", sameFrame);
    const auto& r = frame.pulses[0];
    const auto& p = r.scan.pulse;
    const LaserEmitter emitter(p.OriginWorld(), p.CenterDirectionWorld(), config.wavelengthNm,
        config.pulseEnergyJ, config.beamProfile);
    const LidarReceiver receiver(p.OriginWorld(), p.CenterDirectionWorld(), config.receiverFovHalfAngleRadians,
        config.receiverApertureAreaSquareMeters, config.receiverOpticalEfficiency);
    Stratified2DSampler sampler(8, 8, r.raySeed);
    const auto direct = LidarIntegrator(world, emitter, receiver, 64, scattering).SimulatePulse(sampler, p.EmissionTimeSeconds());
    Check("stratified scan energy exactly matches standalone stratified integrator",
        sampler.Exhausted() && direct.TotalReceivedEnergyJ() == r.receivedEnergyJ);
    Check("stratified scan sampling mode preserves ray and receiver noise seed keys",
        independent.raySeed == r.raySeed && independent.noiseSeed == r.noiseSeed);
    Check("stratified scan actual sampling differs from independent random",
        independent.receivedEnergyJ != r.receivedEnergyJ);
    auto noisy = config;
    noisy.reception.noiseSigmaVoltageV = 1e-4;
    const LidarScanDriver noisyDriver(world, scattering, profile, noisy);
    const auto noisyPulse = noisyDriver.SimulatePulse(0);
    Check("stratified scan optical and receiver noise replay together", SameMeasurement(noisyPulse, noisyDriver.SimulatePulse(0)));
    noisy.noiseSeed = 999;
    const auto differentNoise = LidarScanDriver(world, scattering, profile, noisy).SimulatePulse(0);
    Check("stratified scan changing noise seed leaves optical energy unchanged",
        noisyPulse.receivedEnergyJ == differentNoise.receivedEnergyJ && noisyPulse.noiseSeed != differentNoise.noiseSeed);
    auto differentRay = config;
    differentRay.raySeed = 777;
    const auto otherRay = LidarScanDriver(world, scattering, profile, differentRay).SimulatePulse(0);
    Check("stratified scan changing ray seed changes Gaussian optical samples",
        otherRay.raySeed != r.raySeed && otherRay.receivedEnergyJ != r.receivedEnergyJ);
    const HittableList empty;
    const auto missed = LidarScanDriver(empty, scattering, profile, config).SimulateFrame();
    bool misses = missed.pulses.size() == 12 && missed.points.empty();
    for (const auto& miss : missed.pulses)
        misses = misses && miss.emittedRayCount == 64 && miss.surfaceHitCount == 0 && miss.receivedEnergyJ == 0;
    Check("stratified scan misses retain all emitted rays in denominator", misses);
    const auto half = Plane(0);
    const auto partial = LidarScanDriver(half, scattering, profile, config).SimulateFrame();
    Check("stratified scan partial plane preserves misses and six valid points",
        partial.pulses.size() == 12 && partial.points.size() == 6);
    auto cone = config;
    cone.beamProfile.reset();
    cone.divergenceHalfAngleRadians = .003f;
    Check("stratified scan legacy uniform cone also works", LidarScanDriver(world, scattering, profile, cone).SimulatePulse(0).surfaceHitCount == 64);
    cone.divergenceHalfAngleRadians = 0;
    const auto delta = LidarScanDriver(world, scattering, profile, cone).SimulatePulse(0);
    cone.raySamplingMode = LidarRaySamplingMode::Independent;
    Check("stratified scan delta energy unchanged from independent mode",
        delta.receivedEnergyJ == LidarScanDriver(world, scattering, profile, cone).SimulatePulse(0).receivedEnergyJ);
    auto rectangular = config;
    rectangular.xStrata = 7;
    rectangular.yStrata = 5;
    rectangular.samplesPerPulse = 35;
    Check("stratified scan non square grid uses exact product", LidarScanDriver(world, scattering, profile, rectangular).SimulatePulse(0).emittedRayCount == 35);
    const LidarScanDriver snapshot(world, scattering, profile, rectangular);
    rectangular.xStrata = 0;
    Check("stratified scan owns sampling configuration snapshot",
        snapshot.Config().xStrata == 7 && snapshot.SimulatePulse(0).emittedRayCount == 35);
    const LidarScanSequence plan(config.scan);
    const LidarScanPoseConfig pose{config.scan.originWorld, config.scan.forwardWorld, config.scan.upHintWorld};
    const LidarMultiFrameDriver sequence(world, scattering, profile, plan, pose, config, {2, .1, 0});
    const auto nextFrame = sequence.SimulateFrame(1);
    bool propagated = nextFrame.pulses.size() == 12 && nextFrame.cloud.points.size() == 12;
    for (const auto& pulse : nextFrame.pulses) propagated = propagated && pulse.emittedRayCount == 64;
    Check("stratified scan mode propagates through continuous multi frame driver", propagated);
    Check("stratified scan later frame replays with fresh batches",
        SameMeasurement(nextFrame.pulses[0], sequence.SimulateFrame(1).pulses[0]));

    auto bad = config;
    bad.samplesPerPulse = 63;
    Check("stratified scan rejects incomplete batch before tracing", Throws<std::invalid_argument>([&] { LidarScanDriver d(world, scattering, profile, bad); }));
    bad = config; bad.samplesPerPulse = 65;
    Check("stratified scan rejects sample count exceeding batch before tracing", Throws<std::invalid_argument>([&] { LidarScanDriver d(world, scattering, profile, bad); }));
    bad = config; bad.xStrata = 0;
    Check("stratified scan zero X rejected", Throws<std::invalid_argument>([&] { LidarScanDriver d(world, scattering, profile, bad); }));
    bad = config; bad.yStrata = 0;
    Check("stratified scan zero Y rejected", Throws<std::invalid_argument>([&] { LidarScanDriver d(world, scattering, profile, bad); }));
    bad = config; bad.xStrata = std::numeric_limits<std::size_t>::max(); bad.yStrata = 2;
    Check("stratified scan grid product overflow rejected", Throws<std::length_error>([&] { LidarScanDriver d(world, scattering, profile, bad); }));
    bad = config; bad.xStrata = Stratified2DSampler::MaxStrataPerAxis + 1;
    Check("stratified scan excessive float grid resolution rejected", Throws<std::invalid_argument>([&] { LidarScanDriver d(world, scattering, profile, bad); }));
    bad = config; bad.raySamplingMode = static_cast<LidarRaySamplingMode>(999);
    Check("stratified scan unknown sampling mode rejected", Throws<std::invalid_argument>([&] { LidarScanDriver d(world, scattering, profile, bad); }));
    bad = config; bad.raySamplingMode = LidarRaySamplingMode::Independent; bad.xStrata = 0; bad.yStrata = 0; bad.samplesPerPulse = 5;
    Check("stratified scan independent mode ignores inactive grid fields", LidarScanDriver(world, scattering, profile, bad).SimulatePulse(0).emittedRayCount == 5);
}
