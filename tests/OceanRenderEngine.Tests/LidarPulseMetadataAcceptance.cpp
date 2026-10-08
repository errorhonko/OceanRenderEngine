#include "LidarPulseMetadata.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace {
void Check(const char* name,bool ok) {
    if(!ok)throw std::runtime_error(std::string("[FAIL] ")+name);
    std::cout<<"[PASS] "<<name<<'\n';
}
template<class E,class F>bool Throws(F f) {
    try{f();}catch(const E&){return true;}return false;
}
bool Near(double a,double b){return std::abs(a-b)<1e-6;}
bool Same(const Vector3f& a,const Vector3f& b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
double Length(const Vector3f& v){return std::sqrt(double(v.x)*v.x+double(v.y)*v.y+double(v.z)*v.z);}
static_assert(std::is_same_v<decltype(std::declval<const LidarPulseMetadata&>().OriginWorld()),const Vector3f&>);
static_assert(std::is_same_v<decltype(std::declval<const LidarPulseMetadata&>().CenterDirectionWorld()),const Vector3f&>);
}

void RunLidarPulseMetadataAcceptanceTests(){
    const double time=.123456789123;
    const LidarPulseMetadata pulse(42,time,{1,10,-2},{3,4,0});
    Check("pulse metadata stores pulse id",pulse.PulseId()==42);
    Check("pulse metadata stores world origin",Same(pulse.OriginWorld(),{1,10,-2}));
    Check("pulse metadata normalizes direction components",Near(pulse.CenterDirectionWorld().x,.6)&&Near(pulse.CenterDirectionWorld().y,.8)&&pulse.CenterDirectionWorld().z==0);
    Check("pulse metadata direction unit length",Near(Length(pulse.CenterDirectionWorld()),1));
    Check("pulse metadata preserves double emission time",pulse.EmissionTimeSeconds()==time&&pulse.EmissionTimeSeconds()!=double(float(time)));
    const double longTime=1234567.125000001;
    Check("pulse metadata avoids float narrowing at long clock offset",LidarPulseMetadata(1,longTime,{0,0,0},{0,-1,0}).EmissionTimeSeconds()==longTime&&double(float(longTime))!=longTime);
    const LidarPulseMetadata zero(0,0,{0,0,0},{0,-2,0});
    Check("pulse metadata zero id and zero time valid",zero.PulseId()==0&&zero.EmissionTimeSeconds()==0);
    Check("pulse metadata negative direction signs retained",Same(zero.CenterDirectionWorld(),{0,-1,0}));
    const auto maximumId=std::numeric_limits<std::uint64_t>::max();
    Check("pulse metadata full 64 bit id retained",LidarPulseMetadata(maximumId,0,{0,0,0},{1,0,0}).PulseId()==maximumId);
    Vector3f origin(1,2,3),direction(0,-2,0);
    const LidarPulseMetadata snapshot(8,0,origin,direction);
    origin={9,9,9};direction={1,0,0};
    Check("pulse metadata owns independent origin direction snapshot",Same(snapshot.OriginWorld(),{1,2,3})&&Same(snapshot.CenterDirectionWorld(),{0,-1,0}));
    const float maximum=std::numeric_limits<float>::max();
    const LidarPulseMetadata large(1,0,{0,0,0},{maximum,maximum,maximum});
    Check("pulse metadata huge finite direction normalizes without overflow",Near(Length(large.CenterDirectionWorld()),1)&&Near(large.CenterDirectionWorld().x,1/std::sqrt(3.0)));
    const float tiny=std::numeric_limits<float>::min();
    const LidarPulseMetadata small(1,0,{0,0,0},{tiny,0,0});
    Check("pulse metadata tiny nonzero direction normalizes without underflow",Same(small.CenterDirectionWorld(),{1,0,0}));
    const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
    const float fnan=std::numeric_limits<float>::quiet_NaN(),finf=std::numeric_limits<float>::infinity();
    Check("pulse metadata negative time rejected",Throws<std::invalid_argument>([]{LidarPulseMetadata bad(0,-1,{0,0,0},{0,1,0});}));
    Check("pulse metadata NaN time rejected",Throws<std::invalid_argument>([&]{LidarPulseMetadata bad(0,nan,{0,0,0},{0,1,0});}));
    Check("pulse metadata infinite time rejected",Throws<std::invalid_argument>([&]{LidarPulseMetadata bad(0,inf,{0,0,0},{0,1,0});}));
    Check("pulse metadata NaN origin rejected",Throws<std::invalid_argument>([&]{LidarPulseMetadata bad(0,0,{fnan,0,0},{0,1,0});}));
    Check("pulse metadata infinite origin rejected",Throws<std::invalid_argument>([&]{LidarPulseMetadata bad(0,0,{0,finf,0},{0,1,0});}));
    Check("pulse metadata zero direction rejected",Throws<std::invalid_argument>([]{LidarPulseMetadata bad(0,0,{0,0,0},{0,0,0});}));
    Check("pulse metadata NaN direction rejected",Throws<std::invalid_argument>([&]{LidarPulseMetadata bad(0,0,{0,0,0},{0,fnan,0});}));
    Check("pulse metadata infinite direction rejected",Throws<std::invalid_argument>([&]{LidarPulseMetadata bad(0,0,{0,0,0},{0,0,finf});}));
    LaserEmitter emitter({0,10,0},{0,-1,0},532,1e-6f,.1f);
    const auto fromEmitter=LidarPulseMetadata::FromEmitter(71,time,emitter);
    Check("pulse metadata factory retains supplied id and double time",fromEmitter.PulseId()==71&&fromEmitter.EmissionTimeSeconds()==time);
    Check("pulse metadata factory copies emitter position",Same(fromEmitter.OriginWorld(),emitter.Position()));
    Check("pulse metadata factory uses central emitter direction",Same(fromEmitter.CenterDirectionWorld(),emitter.Direction()));
    const auto sampled=emitter.SampleRay(Point2f(.75f,.125f));
    Check("pulse metadata central direction differs from divergent sampled ray",!Same(fromEmitter.CenterDirectionWorld(),sampled.ray.dir));
    emitter=LaserEmitter({5,5,5},{1,0,0},532,1e-6f,.1f);
    Check("pulse metadata factory snapshot survives emitter change",Same(fromEmitter.OriginWorld(),{0,10,0})&&Same(fromEmitter.CenterDirectionWorld(),{0,-1,0}));
}
