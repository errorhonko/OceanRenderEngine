#include "lidar/scan/LidarRotaryScanPlan.h"
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

namespace {
constexpr double RotaryPi=3.14159265358979323846;
void Check(const char* name,bool ok) {
    if (!ok) throw std::runtime_error(std::string("[FAIL] ")+name);
    std::cout<<"[PASS] "<<name<<'\n';
}
bool Near(double a,double b,double epsilon=1e-6) {return std::abs(a-b)<epsilon;}
template<class E,class F> bool Throws(F f) {
    try {f();} catch(const E&) {return true;}return false;
}
LidarRotaryScanConfig Four() {
    LidarRotaryScanConfig c;
    c.channels={{10,-15*RotaryPi/180},{20,-5*RotaryPi/180},{30,5*RotaryPi/180},{40,15*RotaryPi/180}};
    c.azimuthSamplesPerRevolution=4;c.rotationFrequencyHz=10;
    c.startTimeSeconds=1.00000001;c.firstPulseId=501;
    return c;
}
HittableList FrontPlane() {
    HittableList world;
    const Vector3f a(-100,-100,10),b(-100,100,10),c(100,-100,10),d(100,100,10),n(0,0,-1),uv(0,0,0);
    world.add(std::make_shared<Triangle>(a,b,c,n,n,n,uv,uv,uv,nullptr));
    world.add(std::make_shared<Triangle>(c,b,d,n,n,n,uv,uv,uv,nullptr));
    return world;
}
}

void RunLidarRotaryScanAcceptanceTests() {
    auto c=Four();const LidarRotaryScanPlan plan(c);const LidarScanPlan& base=plan;
    Check("rotary plan four channels by four columns gives sixteen events",base.Size()==16);
    Check("rotary plan ten Hz revolution lasts one tenth second",Near(plan.FrameDurationSeconds(),.1,1e-15));
    Check("rotary plan four columns have twenty five ms spacing",Near(plan.ColumnPeriodSeconds(),.025,1e-15));
    bool times=true,ids=true,elevations=true,normalized=true,noRaster=true;
    for (std::size_t i=0;i<plan.Size();++i) {
        const auto e=base.Firing(i); const auto& channel=c.channels[i%4];
        times=times && e.emissionTimeSeconds==std::fma(double(i/4),.025,c.startTimeSeconds);
        ids=ids && e.pulseId==501+i && e.channelId==channel.channelId;
        elevations=elevations && Near(e.directionSensor.y,std::sin(channel.elevationRadians));
        const auto d=e.directionSensor;
        normalized=normalized && Near(double(d.x)*d.x+double(d.y)*d.y+double(d.z)*d.z,1);
        noRaster=noRaster && !e.raster;
    }
    Check("rotary plan simultaneous channels share each column timestamp",times);
    Check("rotary plan channel ids and pulse ids follow firing order",ids);
    Check("rotary plan each channel retains fixed elevation",elevations);
    Check("rotary plan local directions remain unit length",normalized);
    Check("rotary plan does not fabricate raster cells",noRaster);
    Check("rotary plan quarter turn points toward local positive X",plan.Firing(4).directionSensor.x>.9 && Near(plan.Firing(4).directionSensor.z,0));
    Check("rotary plan half turn points behind sensor",plan.Firing(8).directionSensor.z<-.9);
    Check("rotary plan three quarter turn points toward negative X",plan.Firing(12).directionSensor.x<-.9);
    Check("rotary plan avoids duplicate revolution endpoint",plan.Firing(15).emissionTimeSeconds<c.startTimeSeconds+plan.FrameDurationSeconds());
    Check("rotary plan out of range firing rejected",Throws<std::out_of_range>([&]{plan.Firing(16);}));
    auto delayed=c;delayed.channels[0].firingOffsetSeconds=.005;
    delayed.channels[1].firingOffsetSeconds=.002;delayed.channels[3].firingOffsetSeconds=.002;
    delayed.channels[0].azimuthOffsetRadians=.2;delayed.channels[0].originSensor={1,2,3};
    const LidarRotaryScanPlan staggered(delayed);
    Check("rotary plan sorts channels by firing time not channel number",staggered.Firing(0).channelId==30 && staggered.Firing(3).channelId==10);
    Check("rotary plan simultaneous offset ties retain configured order",staggered.Firing(1).channelId==20 && staggered.Firing(2).channelId==40);
    const auto lag=staggered.Firing(3);const double angle=.2+2*RotaryPi*10*.005;
    Check("rotary plan delayed firing advances angle at actual firing time",Near(lag.directionSensor.x,std::cos(delayed.channels[0].elevationRadians)*std::sin(angle)));
    Check("rotary plan channel time offset preserved",lag.emissionTimeSeconds==c.startTimeSeconds+.005);
    Check("rotary plan channel local origin offset preserved",lag.originSensor.x==1 && lag.originSensor.y==2 && lag.originSensor.z==3);
    bool chronological=true;for(std::size_t i=1;i<staggered.Size();++i)
        chronological=chronological && staggered.Firing(i).emissionTimeSeconds>=staggered.Firing(i-1).emissionTimeSeconds;
    Check("rotary plan delayed events remain chronological across columns",chronological);
    auto shifted=c;shifted.startTimeSeconds=0;const LidarRotaryScanPlan zero(shifted);
    Check("rotary plan large clock shift does not alter local direction",zero.Firing(7).directionSensor.x==plan.Firing(7).directionSensor.x && zero.Firing(7).directionSensor.y==plan.Firing(7).directionSensor.y);
    shifted=c;shifted.startAzimuthRadians=-RotaryPi/2;const LidarRotaryScanPlan negative(shifted);
    Check("rotary plan negative starting phase wraps correctly",negative.Firing(0).directionSensor.x<-.9 && Near(negative.Firing(0).directionSensor.z,0));
    shifted=c;shifted.startAzimuthRadians=2*RotaryPi;const LidarRotaryScanPlan wrapped(shifted);
    Check("rotary plan full turn phase equals zero phase",wrapped.Firing(0).directionSensor.z==plan.Firing(0).directionSensor.z);
    shifted=c;shifted.rotationFrequencyHz=20;const LidarRotaryScanPlan faster(shifted);
    Check("rotary plan increased frequency halves period not spatial sampling",Near(faster.ColumnPeriodSeconds(),.0125,1e-15) && faster.Firing(4).directionSensor.x==plan.Firing(4).directionSensor.x);
    auto snapshot=c;const LidarRotaryScanPlan copied(snapshot);snapshot.channels.clear();
    Check("rotary plan owns immutable configuration snapshot",copied.Size()==16 && copied.Config().channels.size()==4);

    const auto world=FrontPlane();const LambertianLidarScattering scattering(.5);const GaussianPulseProfile profile(4e-9);
    LidarScanDriverConfig optical;optical.pulseEnergyJ=1e-8f;optical.samplesPerPulse=3;
    optical.reception.noiseWindow={0,20e-9};optical.reception.searchWindow={40e-9,115e-9};
    optical.reception.noiseSigmaVoltageV=0;optical.reception.adcBitCount=16;
    optical.reception.rangeMethod=LidarReceiverRangeMethod::Centroid;
    optical.reception.calibration=LidarRangeCalibration(2e-9);optical.rangeSource=LidarPointRangeSource::Corrected;
    const LidarScanPoseConfig pose{{0,0,0},{0,0,1},{0,1,0}};
    const LidarScanDriver driver(world,scattering,profile,plan,pose,optical);
    const auto frame=driver.SimulateFrame();
    Check("rotary real chain preserves all sixteen measurement records",frame.pulses.size()==16);
    Check("rotary real chain only four forward beams detect front plane",frame.points.size()==4);
    bool plane=true,channels=true,misses=true;
    for(std::size_t i=0;i<frame.pulses.size();++i) {
        const auto& r=frame.pulses[i];channels=channels && r.scan.firing.channelId==c.channels[i%4].channelId;
        if(i<4) plane=plane && r.point && std::abs(r.point->positionWorld.z-10)<.05 &&
            std::abs(r.point->rangeMeters-10/std::cos(c.channels[i].elevationRadians))<.05 && r.surfaceHitCount==3;
        else misses=misses && !r.point && !r.reception.measured;
    }
    Check("rotary real chain measured plane distances within five cm",plane);
    Check("rotary real chain retains channel association",channels);
    Check("rotary rear and side scans retained as misses not origin points",misses);
    const auto replay=driver.SimulatePulse(2);
    Check("rotary real chain independent pulse replay reproducible",replay.point && replay.point->rangeMeters==frame.points[2].rangeMeters && replay.noiseSeed==frame.pulses[2].noiseSeed);

    auto bad=c;bad.channels.clear();
    Check("rotary empty channel list rejected",Throws<std::invalid_argument>([&]{LidarRotaryScanPlan p(bad);}));
    bad=c;bad.azimuthSamplesPerRevolution=0;
    Check("rotary zero horizontal sample count rejected",Throws<std::invalid_argument>([&]{LidarRotaryScanPlan p(bad);}));
    bad=c;bad.azimuthSamplesPerRevolution=std::numeric_limits<std::size_t>::max();
    Check("rotary event count overflow rejected",Throws<std::invalid_argument>([&]{LidarRotaryScanPlan p(bad);}));
    bad=c;bad.firstPulseId=std::numeric_limits<std::uint64_t>::max();
    Check("rotary pulse id overflow rejected",Throws<std::invalid_argument>([&]{LidarRotaryScanPlan p(bad);}));
    bad=c;bad.rotationFrequencyHz=0;
    Check("rotary nonpositive rotation frequency rejected",Throws<std::invalid_argument>([&]{LidarRotaryScanPlan p(bad);}));
    bad=c;bad.startTimeSeconds=-1;
    Check("rotary negative start time rejected",Throws<std::invalid_argument>([&]{LidarRotaryScanPlan p(bad);}));
    bad=c;bad.channels[1].channelId=10;
    Check("rotary duplicate channel ids rejected",Throws<std::invalid_argument>([&]{LidarRotaryScanPlan p(bad);}));
    bad=c;bad.channels[0].elevationRadians=RotaryPi;
    Check("rotary elevation outside physical range rejected",Throws<std::invalid_argument>([&]{LidarRotaryScanPlan p(bad);}));
    bad=c;bad.channels[0].firingOffsetSeconds=-1e-9;
    Check("rotary negative firing offset rejected",Throws<std::invalid_argument>([&]{LidarRotaryScanPlan p(bad);}));
    bad=c;bad.channels[0].firingOffsetSeconds=.025;
    Check("rotary firing offset crossing next column rejected",Throws<std::invalid_argument>([&]{LidarRotaryScanPlan p(bad);}));
    bad=c;bad.startAzimuthRadians=std::numeric_limits<double>::infinity();
    Check("rotary nonfinite starting angle rejected",Throws<std::invalid_argument>([&]{LidarRotaryScanPlan p(bad);}));
    bad=c;bad.channels[0].azimuthOffsetRadians=std::numeric_limits<double>::quiet_NaN();
    Check("rotary nonfinite channel phase rejected",Throws<std::invalid_argument>([&]{LidarRotaryScanPlan p(bad);}));
    bad=c;bad.channels[0].originSensor.x=std::numeric_limits<float>::infinity();
    Check("rotary nonfinite beam origin rejected",Throws<std::invalid_argument>([&]{LidarRotaryScanPlan p(bad);}));
    bad=c;bad.startTimeSeconds=1e10;bad.channels[0].firingOffsetSeconds=1e-9;
    Check("rotary indistinguishable distinct firing offsets rejected",Throws<std::invalid_argument>([&]{LidarRotaryScanPlan p(bad);}));
    bad=c;bad.startTimeSeconds=1e10;for(auto& channel:bad.channels)channel.firingOffsetSeconds=1e-9;
    Check("rotary shared nonzero offset lost at large clock rejected",Throws<std::invalid_argument>([&]{LidarRotaryScanPlan p(bad);}));
    bad=c;bad.startTimeSeconds=0;for(auto& channel:bad.channels)channel.firingOffsetSeconds=std::nextafter(.025,0.0);
    Check("rotary last event rounding into next revolution rejected",Throws<std::invalid_argument>([&]{LidarRotaryScanPlan p(bad);}));
}
