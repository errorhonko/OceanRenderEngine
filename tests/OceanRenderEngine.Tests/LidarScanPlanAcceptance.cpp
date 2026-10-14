#include "lidar/scan/LidarScanDriver.h"
#include "lidar/emitter/pulse/GaussianPulseProfile.h"
#include "reflection/lidar/LambertianLidarScattering.h"
#include "core/ray/HittableList.h"
#include "geometry/primitives/Triangle.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void Check(const char* name,bool ok) {
    if (!ok) throw std::runtime_error(std::string("[FAIL] ")+name);
    std::cout<<"[PASS] "<<name<<'\n';
}
template<class E,class F> bool Throws(F f) {
    try { f(); } catch(const E&) {return true;} return false;
}
bool Near(double a,double b,double epsilon=1e-5) {return std::abs(a-b)<epsilon;}
class ExplicitPlan final : public LidarScanPlan {
public:
    std::vector<LidarFiringEvent> events;
    std::size_t Size() const override {return events.size();}
    LidarFiringEvent Firing(std::size_t i) const override {return events.at(i);}
};
HittableList Plane() {
    HittableList world;
    const Vector3f a(-100,0,-100),b(-100,0,100),c(100,0,-100),d(100,0,100),n(0,1,0),uv(0,0,0);
    world.add(std::make_shared<Triangle>(a,b,c,n,n,n,uv,uv,uv,nullptr));
    world.add(std::make_shared<Triangle>(c,b,d,n,n,n,uv,uv,uv,nullptr));
    return world;
}
LidarScanDriverConfig Config() {
    LidarScanDriverConfig c;
    c.pulseEnergyJ=1e-8f;c.samplesPerPulse=3;
    c.reception.noiseWindow={0,20e-9};c.reception.searchWindow={40e-9,115e-9};
    c.reception.noiseSigmaVoltageV=0;c.reception.adcBitCount=16;
    c.reception.rangeMethod=LidarReceiverRangeMethod::Centroid;
    c.reception.calibration=LidarRangeCalibration(2e-9);
    c.rangeSource=LidarPointRangeSource::Corrected;
    return c;
}
ExplicitPlan Three() {
    ExplicitPlan p;
    p.events={
        {91,7,1.0,{0,0,0},{0,0,1},std::nullopt},
        {12,9,1.0,{1,0,0},{0,0,2},std::nullopt},
        {1000,2,1.0001,{0,0,0},{.1f,0,1},std::nullopt}};
    return p;
}
}

void RunLidarScanPlanAcceptanceTests() {
    LidarScanSequenceConfig rectangle; rectangle.originWorld={0,10,0};
    rectangle.azimuthCount=4;rectangle.elevationCount=3;rectangle.firstPulseId=50;
    rectangle.startTimeSeconds=1;rectangle.serpentine=true;
    const LidarScanSequence raster(rectangle);
    const LidarScanPlan& generic=raster;
    Check("scan plan rectangular implementation exposes polymorphic size",generic.Size()==12);
    const auto event=generic.Firing(4);
    Check("scan plan local firing origin excludes world mounting translation",event.originSensor.x==0 && event.originSensor.y==0 && event.originSensor.z==0);
    Check("scan plan raster diagnostics optional but present for rectangle",event.raster && event.raster->azimuthIndex==3 && event.raster->elevationIndex==1);
    Check("scan plan raster uses single physical channel not row as channel",event.channelId==0);
    Check("scan plan event id and double time preserved",event.pulseId==54 && event.emissionTimeSeconds==std::fma(4.0,rectangle.pulseIntervalSeconds,1.0));
    const auto sample=raster.Sample(4);
    const LidarScanPose mounting({rectangle.originWorld,rectangle.forwardWorld,rectangle.upHintWorld});
    const auto metadata=mounting.ToWorld(event);
    Check("scan plan legacy world sample agrees with explicit pose adapter",metadata.PulseId()==sample.pulse.PulseId() &&
        metadata.CenterDirectionWorld().x==sample.pulse.CenterDirectionWorld().x && metadata.OriginWorld().y==10);
    Check("scan plan out of range firing rejected",Throws<std::out_of_range>([&]{generic.Firing(12);}));
    const LidarScanPose rotated({{2,3,4},{1,0,0},{0,0,1}});
    const auto transformed=rotated.ToWorld({1,0,.1,{1,2,3},{0,0,7},std::nullopt});
    Check("scan pose transforms local forward to configured world forward",Near(transformed.CenterDirectionWorld().x,1) && Near(transformed.CenterDirectionWorld().y,0));
    Check("scan pose transforms beam origin offset and translation",Near(transformed.OriginWorld().x,5) && Near(transformed.OriginWorld().y,4) && Near(transformed.OriginWorld().z,6));

    const auto world=Plane();const LambertianLidarScattering scattering(.5);const GaussianPulseProfile profile(4e-9);
    auto config=Config();auto explicitPlan=Three();const LidarScanPoseConfig pose{{0,10,0},{0,-1,0},{0,0,1}};
    const LidarScanDriver driver(world,scattering,profile,explicitPlan,pose,config);
    const auto frame=driver.SimulateFrame();
    Check("scan plan arbitrary non raster events run complete real chain",frame.pulses.size()==3 && frame.points.size()==3);
    Check("scan plan driver does not invent rectangle indices",!frame.pulses[0].scan.firing.raster && !frame.pulses[2].scan.firing.raster);
    Check("scan plan channel ids retained in measurement records",frame.pulses[0].scan.firing.channelId==7 && frame.pulses[1].scan.firing.channelId==9);
    Check("scan plan simultaneous distinct channels remain separate pulses",frame.pulses[0].scan.pulse.EmissionTimeSeconds()==frame.pulses[1].scan.pulse.EmissionTimeSeconds() && frame.points.size()==3);
    Check("scan plan nonmonotonic pulse ids preserve supplied order",frame.points[0].pulseId==91 && frame.points[1].pulseId==12 && frame.points[2].pulseId==1000);
    Check("scan plan per beam origin offsets affect measured point",Near(frame.points[1].positionWorld.x,1) && Near(frame.pulses[1].scan.pulse.OriginWorld().x,1));
    Check("scan plan arbitrary direction traces and measures tilted beam",frame.points[2].positionWorld.x>.9 && std::abs(frame.points[2].positionWorld.y)<.05);
    Check("scan plan driver uses supplied pose instead of legacy config scan",frame.pulses[0].scan.pulse.OriginWorld().y==10 && config.scan.originWorld.y==0);
    explicitPlan.events.clear();
    Check("scan plan driver snapshot unaffected by source plan mutation",driver.Size()==3 && driver.SimulatePulse(1).point->pulseId==12);
    const auto detached=[&] {auto local=Three();return LidarScanDriver(world,scattering,profile,local,pose,config);}();
    Check("scan plan source may be destroyed after driver construction",detached.SimulatePulse(0).point->pulseId==91);
    config.scan=rectangle;
    const LidarScanDriver legacy(world,scattering,profile,config);
    const LidarScanDriver injected(world,scattering,profile,raster,pose,config);
    const auto a=legacy.SimulatePulse(4),b=injected.SimulatePulse(4);
    Check("scan plan injected raster preserves legacy optical energy",a.receivedEnergyJ==b.receivedEnergyJ && a.raySeed==b.raySeed);
    Check("scan plan injected raster preserves legacy digital measurement",a.point && b.point && a.point->rangeMeters==b.point->rangeMeters && a.noiseSeed==b.noiseSeed);

    auto bad=Three();bad.events.clear();
    Check("scan plan empty plan rejected",Throws<std::invalid_argument>([&]{LidarScanDriver d(world,scattering,profile,bad,pose,config);}));
    bad=Three();bad.events[1].pulseId=bad.events[0].pulseId;
    Check("scan plan duplicate pulse ids rejected",Throws<std::invalid_argument>([&]{LidarScanDriver d(world,scattering,profile,bad,pose,config);}));
    bad=Three();bad.events[2].emissionTimeSeconds=.5;
    Check("scan plan backward event time rejected",Throws<std::invalid_argument>([&]{LidarScanDriver d(world,scattering,profile,bad,pose,config);}));
    bad=Three();bad.events[0].emissionTimeSeconds=-1;
    Check("scan plan negative event time rejected",Throws<std::invalid_argument>([&]{LidarScanDriver d(world,scattering,profile,bad,pose,config);}));
    bad=Three();bad.events[0].emissionTimeSeconds=std::numeric_limits<double>::quiet_NaN();
    Check("scan plan nonfinite event time rejected",Throws<std::invalid_argument>([&]{LidarScanDriver d(world,scattering,profile,bad,pose,config);}));
    bad=Three();bad.events[1].directionSensor={0,0,0};
    Check("scan plan zero local beam direction rejected",Throws<std::invalid_argument>([&]{LidarScanDriver d(world,scattering,profile,bad,pose,config);}));
    bad=Three();bad.events[1].originSensor.x=std::numeric_limits<float>::infinity();
    Check("scan plan nonfinite beam origin rejected",Throws<std::invalid_argument>([&]{LidarScanDriver d(world,scattering,profile,bad,pose,config);}));
    Check("scan pose parallel up and forward rejected",Throws<std::invalid_argument>([]{LidarScanPose p({{0,0,0},{0,0,1},{0,0,1}});}));
    Check("scan pose overflowing transformed origin rejected",Throws<std::overflow_error>([]{
        const float max=std::numeric_limits<float>::max();
        LidarScanPose p({{max,0,0},{1,0,0},{0,0,1}});
        p.ToWorld({1,0,0,{0,0,max},{0,0,1},std::nullopt});
    }));
}
