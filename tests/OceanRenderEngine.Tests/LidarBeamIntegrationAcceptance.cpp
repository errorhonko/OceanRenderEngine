#include "lidar/emitter/beam/GaussianLidarBeamProfile.h"
#include "lidar/scan/LidarMultiFrameDriver.h"
#include "lidar/emitter/pulse/GaussianPulseProfile.h"
#include "reflection/lidar/LambertianLidarScattering.h"
#include "geometry/primitives/Triangle.h"
#include "core/ray/HittableList.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace {
void Check(const char* name, bool ok)
{
    if (!ok) throw std::runtime_error(std::string("[FAIL] ") + name);
    std::cout << "[PASS] " << name << '\n';
}
template<class E, class F> bool Throws(F action)
{
    try { action(); } catch (const E&) { return true; } return false;
}
bool Near(double a, double b, double eps = 1e-7) { return std::abs(a-b) <= eps; }
bool SameDirection(const Vector3f& a, const Vector3f& b)
{ return a.x==b.x && a.y==b.y && a.z==b.z; }

class SampleFixture final : public LidarBeamProfile
{
public:
    SampleFixture(LidarBeamDirectionSample sample, bool delta) : sample(sample), delta(delta) {}
    LidarBeamDirectionSample SampleDirection(const Point2f&) const override { return sample; }
    double EnergyDensityPerSr(const Vector3f&) const override { return 0; }
    double PdfDirection(const Vector3f&) const override { return sample.pdfPerSr; }
    bool IsDeltaDirection() const override { return delta; }
private:
    LidarBeamDirectionSample sample;
    bool delta;
};
HittableList Plane()
{
    HittableList world;
    const Vector3f a(-100,0,-100), b(-100,0,100), c(100,0,-100), d(100,0,100), n(0,1,0), uv(0,0,0);
    world.add(std::make_shared<Triangle>(a,b,c,n,n,n,uv,uv,uv,nullptr));
    world.add(std::make_shared<Triangle>(c,b,d,n,n,n,uv,uv,uv,nullptr));
    return world;
}
LidarScanDriverConfig Settings()
{
    LidarScanDriverConfig c;
    c.scan.originWorld={0,10,0};
    c.scan.azimuthCount=2; c.scan.elevationCount=2;
    c.scan.azimuthMinRadians=-.1; c.scan.azimuthMaxRadians=.1;
    c.scan.elevationMinRadians=-.1; c.scan.elevationMaxRadians=.1;
    c.scan.startTimeSeconds=1; c.scan.pulseIntervalSeconds=1e-4;
    c.pulseEnergyJ=1e-8f; c.samplesPerPulse=64;
    c.reception.noiseWindow={0,20e-9}; c.reception.searchWindow={40e-9,115e-9};
    c.reception.noiseSigmaVoltageV=0; c.reception.adcBitCount=16;
    c.reception.rangeMethod=LidarReceiverRangeMethod::Centroid;
    c.reception.calibration=LidarRangeCalibration(2e-9);
    c.rangeSource=LidarPointRangeSource::Corrected;
    return c;
}
}

void RunLidarBeamIntegrationAcceptanceTests()
{
    static_assert(std::is_same_v<decltype(LaserEmissionSample::energyWeightJ),double>);
    static_assert(std::is_same_v<decltype(LaserEmissionSample::directionPdf),double>);
    const auto gaussian=std::make_shared<GaussianLidarBeamProfile>(.001,.003);
    const Vector3f origin(1,10,-2), axis=Vector3f(1,-2,3).normalize();
    const LaserEmitter emitter(origin,axis,532,1e-8f,gaussian);
    const Point2f u(.7f,.2f);
    const auto local=gaussian->SampleDirection(u);
    const auto emission=emitter.SampleRay(u,1.00000001);
    const auto expected=Frame::FromZ(axis).FromLocal(local.directionLocal).normalize();
    Check("beam integration Gaussian local direction rotates into emitter frame",SameDirection(emission.ray.dir,expected));
    Check("beam integration ray retains emitter origin",SameDirection(emission.ray.orig,origin));
    Check("beam integration wavelength and double clock retained",emission.wavelengthNm==532 && emission.emissionTimeSeconds==1.00000001);
    Check("beam integration Gaussian pulse energy is not weighted twice",emission.energyWeightJ==double(emitter.PulseEnergyJ()));
    Check("beam integration solid angle PDF survives rigid rotation",emission.directionPdf==local.pdfPerSr);
    Check("beam integration custom Gaussian is not delta",!emitter.IsDeltaDirection() && !emission.deltaDirection);
    Check("beam integration Gaussian center follows emitter axis",Near(emitter.SampleRay(Point2f(0,.5f)).ray.dir.dot(axis),1,2e-7));
    Check("beam integration custom legacy angle query explicitly rejected",Throws<std::logic_error>([&]{emitter.DivergenceHalfAngleRadians();}));
    Check("beam integration custom null profile rejected",Throws<std::invalid_argument>([&]{LaserEmitter e(origin,axis,532,1e-8f,std::shared_ptr<const LidarBeamProfile>{});}));
    Check("beam integration custom invalid sample rejected before delegation",Throws<std::out_of_range>([&]{emitter.SampleRay(Point2f(-1,.5f));}));
    Check("beam integration custom invalid clock rejected",Throws<std::invalid_argument>([&]{emitter.SampleRay(u,-1);}));
    const LaserEmitter legacyDelta(origin,axis,532,1e-8f,0.0f);
    const LaserEmitter legacyCone(origin,axis,532,1e-8f,.1f);
    Check("beam integration legacy constructors now own concrete beam models",
        dynamic_cast<const DeltaLidarBeamProfile*>(legacyDelta.BeamProfile().get()) &&
        dynamic_cast<const UniformConeLidarBeamProfile*>(legacyCone.BeamProfile().get()));
    Check("beam integration legacy delta and cone metadata unchanged",legacyDelta.IsDeltaDirection() && !legacyCone.IsDeltaDirection() && legacyCone.DivergenceHalfAngleRadians()==.1f);
    const auto detached=[&]{auto temporary=std::make_shared<GaussianLidarBeamProfile>(.001,.003); return LaserEmitter(origin,axis,532,1e-8f,temporary);}();
    Check("beam integration emitter owns profile after caller releases it",detached.SampleRay(u).directionPdf>0);
    const LaserEmitter copy=emitter;
    Check("beam integration emitter copies share immutable profile",copy.BeamProfile()==emitter.BeamProfile() && SameDirection(copy.SampleRay(u).ray.dir,emission.ray.dir));
    const auto tiny=std::make_shared<GaussianLidarBeamProfile>(1e-20,3e-20);
    const auto precise=LaserEmitter(origin,{0,0,1},532,1e-8f,tiny).SampleRay(Point2f(0,0));
    Check("beam integration double PDF avoids float overflow",std::isfinite(precise.directionPdf) && precise.directionPdf>std::numeric_limits<float>::max());

    const auto weighted=std::make_shared<SampleFixture>(LidarBeamDirectionSample{{0,0,1},.25,.75,false},false);
    Check("beam integration generic energy weight multiplied exactly once",LaserEmitter(origin,axis,532,1e-8f,weighted).SampleRay(u).energyWeightJ==double(1e-8f)*.75);
    const auto delta=std::make_shared<SampleFixture>(LidarBeamDirectionSample{{0,0,1},0,1,true},true);
    Check("beam integration optional Delta model propagates flags",LaserEmitter(origin,axis,532,1e-8f,delta).SampleRay(u).deltaDirection && LaserEmitter(origin,axis,532,1e-8f,delta).IsDeltaDirection());
    const auto badResult=[&](LidarBeamDirectionSample sample,bool deltaFlag=false){
        auto bad=std::make_shared<SampleFixture>(sample,deltaFlag);
        return Throws<std::domain_error>([&]{LaserEmitter(origin,axis,532,1e-8f,bad).SampleRay(u);});
    };
    Check("beam integration invalid custom zero direction rejected",badResult({{0,0,0},1,1,false}));
    Check("beam integration invalid custom nonunit direction rejected",badResult({{0,0,2},1,1,false}));
    Check("beam integration invalid custom zero continuous PDF rejected",badResult({{0,0,1},0,1,false}));
    Check("beam integration invalid custom infinite PDF rejected",badResult({{0,0,1},std::numeric_limits<double>::infinity(),1,false}));
    Check("beam integration invalid custom negative weight rejected",badResult({{0,0,1},1,-1,false}));
    Check("beam integration invalid custom NaN weight rejected",badResult({{0,0,1},1,std::numeric_limits<double>::quiet_NaN(),false}));
    Check("beam integration inconsistent Delta flags rejected",badResult({{0,0,1},0,1,true},false));
    Check("beam integration Delta ordinary PDF rejected",badResult({{0,0,1},1,1,true},true));
    const auto huge=std::make_shared<SampleFixture>(LidarBeamDirectionSample{{0,0,1},1,std::numeric_limits<double>::max(),false},false);
    Check("beam integration custom weighted energy overflow rejected",Throws<std::overflow_error>([&]{LaserEmitter(origin,axis,532,2,huge).SampleRay(u);}));

    const auto world=Plane();
    const LambertianLidarScattering scattering(.5);
    const GaussianPulseProfile pulseProfile(4e-9);
    auto config=Settings();
    Check("beam integration scan optional profile defaults to legacy",!config.beamProfile);
    config.beamProfile=gaussian;
    const LidarScanDriver driver(world,scattering,pulseProfile,config);
    const auto frame=driver.SimulateFrame();
    Check("beam integration Gaussian scan runs complete waveform to point chain",frame.pulses.size()==4 && frame.points.size()==4);
    bool counts=true,energy=true;
    for(const auto& pulse:frame.pulses){
        counts=counts && pulse.emittedRayCount==64 && pulse.surfaceHitCount==64 && pulse.returnSampleCount==64;
        energy=energy && pulse.receivedEnergyJ>0 && Near(pulse.receivedEnergyJ,pulse.recordedEnergyJ,pulse.receivedEnergyJ*1e-12);
    }
    Check("beam integration Gaussian scan keeps one measurement per pulse",counts && frame.points.size()==frame.pulses.size());
    Check("beam integration Gaussian pulse profile preserves received energy",energy);
    const auto& first=frame.pulses.front(); const auto& metadata=first.scan.pulse;
    IndependentSampler sampler(first.raySeed);
    const LaserEmitter direct(metadata.OriginWorld(),metadata.CenterDirectionWorld(),config.wavelengthNm,config.pulseEnergyJ,gaussian);
    const LidarReceiver receiver(metadata.OriginWorld(),metadata.CenterDirectionWorld(),config.receiverFovHalfAngleRadians,config.receiverApertureAreaSquareMeters,config.receiverOpticalEfficiency);
    const auto returns=LidarIntegrator(world,direct,receiver,config.samplesPerPulse,scattering).SimulatePulse(sampler,metadata.EmissionTimeSeconds());
    Check("beam integration scan shares model and matches standalone MC energy",returns.TotalReceivedEnergyJ()==first.receivedEnergyJ);
    const auto replay=driver.SimulatePulse(0);
    Check("beam integration Gaussian scan replay remains deterministic",replay.receivedEnergyJ==first.receivedEnergyJ && replay.point->rangeMeters==first.point->rangeMeters);
    const auto lifetime=[&]{auto c=Settings();c.beamProfile=std::make_shared<GaussianLidarBeamProfile>(.001,.003);return LidarScanDriver(world,scattering,pulseProfile,c);}();
    Check("beam integration driver owns shared profile after config destruction",lifetime.SimulatePulse(0).point.has_value());
    const LidarScanSequence plan(config.scan);
    const LidarScanPoseConfig pose{config.scan.originWorld,config.scan.forwardWorld,config.scan.upHintWorld};
    const LidarMultiFrameDriver multi(world,scattering,pulseProfile,plan,pose,config,{2,.1,0});
    Check("beam integration multiframe config propagates optional profile",multi.SimulateFrame(1).cloud.points.size()==4);
    auto conflict=config;conflict.divergenceHalfAngleRadians=.01f;
    Check("beam integration conflicting custom model and legacy cone rejected",Throws<std::invalid_argument>([&]{LidarScanDriver d(world,scattering,pulseProfile,conflict);}));
    auto old=Settings();old.divergenceHalfAngleRadians=.01f;
    Check("beam integration optional profile removal restores legacy cone",LidarScanDriver(world,scattering,pulseProfile,old).SimulatePulse(0).point.has_value());
}
