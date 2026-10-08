#include "LidarScanSequence.h"
#include "LidarPointCloudBuilder.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
constexpr double pi=3.14159265358979323846;
void Check(const char* name,bool ok){if(!ok)throw std::runtime_error(std::string("[FAIL] ")+name);std::cout<<"[PASS] "<<name<<'\n';}
bool Near(double a,double b){return std::abs(a-b)<1e-6;}
template<class E,class F>bool Throws(F f){try{f();}catch(const E&){return true;}return false;}
double Length(const Vector3f& v){return std::sqrt(double(v.x)*v.x+double(v.y)*v.y+double(v.z)*v.z);}
LidarScanSequenceConfig Grid(){LidarScanSequenceConfig c;c.originWorld={0,10,0};c.azimuthCount=2;c.elevationCount=2;c.azimuthMinRadians=-.2;c.azimuthMaxRadians=.2;c.elevationMinRadians=-.1;c.elevationMaxRadians=.1;c.firstPulseId=100;c.startTimeSeconds=.001;return c;}
}

void RunLidarScanSequenceAcceptanceTests(){
    const LidarScanSequence defaultScan(LidarScanSequenceConfig{});const auto center=defaultScan.Sample(0);
    Check("scan default single pulse count",defaultScan.Size()==1);
    Check("scan zero local angles point forward down Y",center.azimuthRadians==0&&center.elevationRadians==0&&center.pulse.CenterDirectionWorld().x==0&&center.pulse.CenterDirectionWorld().y==-1&&center.pulse.CenterDirectionWorld().z==0);
    auto config=Grid();const LidarScanSequence grid(config);
    Check("scan rectangular grid pulse count",grid.Size()==4);
    const auto first=grid.Sample(0);const auto last=grid.Sample(3);
    Check("scan angles use cell centers not endpoints",Near(first.azimuthRadians,-.1)&&Near(first.elevationRadians,-.05)&&Near(last.azimuthRadians,.1)&&Near(last.elevationRadians,.05));
    const auto& direction=first.pulse.CenterDirectionWorld();
    Check("scan local to world analytic direction",Near(direction.x,std::cos(-.05)*std::sin(-.1))&&Near(direction.y,-std::cos(-.05)*std::cos(-.1))&&Near(direction.z,std::sin(-.05)));
    bool unit=true,order=true,ids=true,times=true,origin=true;
    for(std::size_t i=0;i<grid.Size();++i){const auto s=grid.Sample(i);unit=unit&&Near(Length(s.pulse.CenterDirectionWorld()),1);order=order&&s.scanIndex==i&&s.azimuthIndex==i%2&&s.elevationIndex==i/2;ids=ids&&s.pulse.PulseId()==100+i;times=times&&std::abs(s.pulse.EmissionTimeSeconds()-(.001+i*.0001))<1e-15;origin=origin&&s.pulse.OriginWorld().y==10&&s.pulse.OriginWorld().x==0&&s.pulse.OriginWorld().z==0;}
    Check("scan every direction normalized",unit);
    Check("scan row major order has azimuth fast axis",order);
    Check("scan ids increase by emission order",ids);
    Check("scan double emission times increase by pulse interval",times);
    Check("scan fixed world origin retained",origin);
    config.serpentine=true;const LidarScanSequence snake(config);
    Check("scan serpentine reverses odd row column order",snake.Sample(2).azimuthIndex==1&&snake.Sample(3).azimuthIndex==0&&snake.Sample(2).elevationIndex==1);
    Check("scan serpentine keeps time and id in emission order",snake.Sample(2).pulse.PulseId()==102&&snake.Sample(3).pulse.EmissionTimeSeconds()>snake.Sample(2).pulse.EmissionTimeSeconds());
    Check("scan serpentine changes direction not angle grid",snake.Sample(2).azimuthRadians==grid.Sample(3).azimuthRadians&&snake.Sample(3).azimuthRadians==grid.Sample(2).azimuthRadians);
    config=Grid();config.azimuthCount=4;config.elevationCount=1;config.azimuthMinRadians=-pi;config.azimuthMaxRadians=pi;config.elevationMinRadians=0;config.elevationMaxRadians=0;
    const LidarScanSequence circle(config);
    Check("scan full circle avoids repeated endpoints",Near(circle.Sample(0).azimuthRadians,-.75*pi)&&Near(circle.Sample(3).azimuthRadians,.75*pi)&&!Near(circle.Sample(0).pulse.CenterDirectionWorld().x,circle.Sample(3).pulse.CenterDirectionWorld().x));
    config=LidarScanSequenceConfig{};config.forwardWorld={0,0,2};config.upHintWorld={0,3,0};config.azimuthMinRadians=pi/2;config.azimuthMaxRadians=pi/2;config.elevationMinRadians=0;config.elevationMaxRadians=0;
    const auto rotated=LidarScanSequence(config).Sample(0);
    Check("scan frame orientation supports arbitrary forward up",Near(rotated.pulse.CenterDirectionWorld().x,1)&&Near(rotated.pulse.CenterDirectionWorld().y,0)&&Near(rotated.pulse.CenterDirectionWorld().z,0));
    config.azimuthMinRadians=0;config.azimuthMaxRadians=0;config.elevationMinRadians=pi/2;config.elevationMaxRadians=pi/2;
    Check("scan positive elevation follows local up",Near(LidarScanSequence(config).Sample(0).pulse.CenterDirectionWorld().y,1));
    config=Grid();const LidarScanSequence snapshot(config);config.originWorld={9,9,9};config.azimuthCount=50;
    Check("scan owns independent config snapshot",snapshot.Size()==4&&snapshot.Config().originWorld.y==10&&snapshot.Config().azimuthCount==2);
    Check("scan repeated index deterministic",snapshot.Sample(1).azimuthRadians==snapshot.Sample(1).azimuthRadians&&snapshot.Sample(1).pulse.EmissionTimeSeconds()==snapshot.Sample(1).pulse.EmissionTimeSeconds());
    Check("scan invalid sample index rejected",Throws<std::out_of_range>([&]{grid.Sample(4);}));
    const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
    const float fnan=std::numeric_limits<float>::quiet_NaN();
    config=Grid();config.azimuthCount=0;
    Check("scan zero dimension rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.azimuthCount=std::numeric_limits<std::size_t>::max();config.elevationCount=2;
    Check("scan dimension multiplication overflow rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.firstPulseId=std::numeric_limits<std::uint64_t>::max();
    Check("scan pulse id overflow rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=LidarScanSequenceConfig{};config.firstPulseId=std::numeric_limits<std::uint64_t>::max();
    Check("scan single maximum pulse id valid",LidarScanSequence(config).Sample(0).pulse.PulseId()==config.firstPulseId);
    config=Grid();config.azimuthMinRadians=nan;
    Check("scan NaN angle rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.elevationMaxRadians=inf;
    Check("scan infinite angle rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.azimuthMinRadians=.5;
    Check("scan reversed angle bounds rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.azimuthMaxRadians=config.azimuthMinRadians;
    Check("scan collapsed multicolumn axis rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.azimuthMinRadians=-2*pi;config.azimuthMaxRadians=2*pi;
    Check("scan azimuth span beyond full turn rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.elevationMaxRadians=pi;
    Check("scan elevation outside hemisphere bounds rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.originWorld.x=fnan;
    Check("scan nonfinite origin rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.forwardWorld={0,0,0};
    Check("scan zero forward rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.upHintWorld={0,-1,0};
    Check("scan parallel up forward rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.upHintWorld={0,0,0};
    Check("scan zero up hint rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.forwardWorld.z=fnan;
    Check("scan nonfinite frame axis rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.startTimeSeconds=-1;
    Check("scan negative start time rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.pulseIntervalSeconds=0;
    Check("scan zero pulse interval rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.pulseIntervalSeconds=inf;
    Check("scan infinite pulse interval rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.startTimeSeconds=std::numeric_limits<double>::max();config.pulseIntervalSeconds=std::numeric_limits<double>::max();
    Check("scan final time overflow rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    config=Grid();config.startTimeSeconds=1e20;config.pulseIntervalSeconds=1e-9;
    Check("scan unresolved pulse times rejected",Throws<std::invalid_argument>([&]{LidarScanSequence bad(config);}));
    const LidarPointCloudBuilder builder(LidarPointRangeSource::Raw);bool plane=true,association=true;
    for(std::size_t i=0;i<grid.Size();++i){
        const auto s=grid.Sample(i);const auto d=s.pulse.CenterDirectionWorld();const double range=-s.pulse.OriginWorld().y/d.y;
        LidarReceiverPipelineResult reception;reception.measured=LidarDigitalRangeEstimate{0,s.pulse.EmissionTimeSeconds()+range/(.5*299792458.0),range/(.5*299792458.0),range,100,20,false,false};
        const auto point=builder.BuildPoint(LidarPulseMeasurement{s.pulse,reception});
        plane=plane&&point&&std::abs(point->positionWorld.y)<1e-5&&std::abs(point->positionWorld.x-10*std::tan(s.azimuthRadians))<1e-5&&std::abs(point->positionWorld.z-10*std::tan(s.elevationRadians)/std::cos(s.azimuthRadians))<1e-5;
        association=association&&point&&point->pulseId==s.pulse.PulseId()&&point->emissionTimeSeconds==s.pulse.EmissionTimeSeconds();
    }
    Check("scan direction to point analytic plane projection",plane);
    Check("scan point cloud preserves pulse id timestamp association",association);
    const auto s=grid.Sample(0);const LaserEmitter emitter(s.pulse.OriginWorld(),s.pulse.CenterDirectionWorld(),532,1e-6f,.001f);
    Check("scan direction feeds emitter central axis",Near(emitter.Direction().x,s.pulse.CenterDirectionWorld().x)&&Near(emitter.Direction().y,s.pulse.CenterDirectionWorld().y)&&Near(emitter.Direction().z,s.pulse.CenterDirectionWorld().z));
}
