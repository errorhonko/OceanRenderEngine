#include "lidar/emitter/beam/GaussianLidarBeamProfile.h"
#include "lidar/emitter/LaserEmitter.h"
#include "core/sampling/IndependentSampler.h"
#include "geometry/primitives/Triangle.h"
#include "core/ray/HittableList.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>

namespace {
void Check(const char* name,bool ok)
{
    if(!ok) throw std::runtime_error(std::string("[FAIL] ")+name);
    std::cout<<"[PASS] "<<name<<'\n';
}
bool Near(double a,double b,double eps){return std::abs(a-b)<=eps;}
struct D3 {double x,y,z;};
D3 ToDouble(const Vector3f& v){return {v.x,v.y,v.z};}
D3 Scale(D3 v,double s){return {v.x*s,v.y*s,v.z*s};}
D3 Minus(D3 a,D3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
double Dot(D3 a,D3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
D3 Cross(D3 a,D3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
D3 Unit(D3 v){return Scale(v,1/std::sqrt(Dot(v,v)));}
struct PlaneBasis {
    D3 n,x,y;
    explicit PlaneBasis(const Vector3f& axis):n(Unit(ToDouble(axis)))
    {
        // Independent double orthonormal basis, not the emitter's Frame::ToLocal().
        const D3 helper=std::abs(n.z)<.9?D3{0,0,1}:D3{0,1,0};
        x=Unit(Cross(helper,n));y=Cross(n,x);
    }
};
struct Spot {double x,y,rSquared,t;};
Spot IntersectPlane(const LaserEmissionSample& sample,const PlaneBasis& plane,double L)
{
    const D3 d=ToDouble(sample.ray.dir);
    const double forward=Dot(d,plane.n);
    if(!std::isfinite(forward)||forward<=0)throw std::runtime_error("Ray cannot reach forward perpendicular plane.");
    const double t=L/forward;
    // Perpendicular plane is origin + L*n. Compute displacement in double,
    // avoiding unrelated float hit-position quantization in this analytic oracle.
    const D3 offset=Minus(Scale(d,t),Scale(plane.n,L));
    const double x=Dot(offset,plane.x),y=Dot(offset,plane.y);
    return {x,y,x*x+y*y,t};
}
struct Stats {
    double meanX=0,meanY=0,meanR2=0,energy=0,maxR2=0;
    double halfFraction=0,scaleFraction=0;
    bool finite=true;
};
Stats Measure(const LaserEmitter& emitter,double scaleAngle,double cutoffAngle,
    double L,int count=65536,std::uint32_t seed=42)
{
    Stats result;IndependentSampler sampler(seed);const PlaneBasis plane(emitter.Direction());
    const double w=L*std::tan(scaleAngle),rMax=L*std::tan(cutoffAngle);
    int halfCount=0,scaleCount=0;
    for(int i=0;i<count;++i){
        const auto emission=emitter.SampleRay(sampler.Get2D());
        const auto spot=IntersectPlane(emission,plane,L);
        result.meanX+=spot.x/count;result.meanY+=spot.y/count;result.meanR2+=spot.rSquared/count;
        result.energy+=emission.energyWeightJ/count;
        result.maxR2=std::max(result.maxR2,spot.rSquared);
        halfCount+=spot.rSquared<w*w/4;scaleCount+=spot.rSquared<w*w;
        result.finite=result.finite&&std::isfinite(spot.t)&&std::isfinite(spot.rSquared)&&
            spot.rSquared<=rMax*rMax*(1+1e-3)&&emission.directionPdf>0;
    }
    result.halfFraction=double(halfCount)/count;result.scaleFraction=double(scaleCount)/count;
    return result;
}
double TruncatedMomentFactor(const GaussianLidarBeamProfile& beam)
{
    const double ratio=std::tan(beam.MaxHalfAngleRadians())/std::tan(beam.E2HalfAngleRadians());
    const double T=2*ratio*ratio;
    return (1-(T+1)*std::exp(-T))/beam.RetainedFraction();
}
}

void RunGaussianBeamFootprintAcceptanceTests()
{
    constexpr double scale=.001,cutoff=.003;
    const auto beam=std::make_shared<GaussianLidarBeamProfile>(scale,cutoff);
    const Vector3f origin(1,2,3);
    const LaserEmitter emitter(origin,{0,0,1},532,1e-8f,beam);
    const double a=std::tan(scale),Z=beam->RetainedFraction(),factor=TruncatedMomentFactor(*beam);
    const auto at10=Measure(emitter,scale,cutoff,10);
    const auto at100=Measure(emitter,scale,cutoff,100);
    const auto at1000=Measure(emitter,scale,cutoff,1000);
    Check("Gaussian footprint forward planes receive finite supported rays",at10.finite&&at100.finite&&at1000.finite);
    const double fittedW10=std::sqrt(2*at10.meanR2/factor);
    const double fittedW100=std::sqrt(2*at100.meanR2/factor);
    const double fittedW1000=std::sqrt(2*at1000.meanR2/factor);
    Check("Gaussian footprint ten meter width matches L tan half angle",Near(fittedW10/(10*a),1,.02));
    Check("Gaussian footprint hundred meter width matches L tan half angle",Near(fittedW100/(100*a),1,.02));
    Check("Gaussian footprint thousand meter width matches L tan half angle",Near(fittedW1000/(1000*a),1,.02));
    Check("Gaussian footprint paired width scales linearly with distance",Near(fittedW100/fittedW10,10,1e-10)&&Near(fittedW1000/fittedW10,100,1e-9));
    Check("Gaussian footprint centroid remains on beam axis",std::hypot(at100.meanX,at100.meanY)<100*a*.01);
    Check("Gaussian footprint half radius energy matches analytic circle integral",Near(at100.halfFraction,-std::expm1(-.5)/Z,.005));
    Check("Gaussian footprint e2 radius energy matches analytic circle integral",Near(at100.scaleFraction,-std::expm1(-2.0)/Z,.005));
    Check("Gaussian footprint ideal plane intercepted energy is distance independent",
        Near(at10.energy/emitter.PulseEnergyJ(),1,1e-10)&&at10.energy==at100.energy&&at100.energy==at1000.energy);
    Check("Gaussian footprint encircled energy is distance independent",at10.halfFraction==at100.halfFraction&&at100.halfFraction==at1000.halfFraction&&at10.scaleFraction==at1000.scaleFraction);
    bool sampleCountEnergy=true;
    for(const int count:{64,256,4096}){
        const auto result=Measure(emitter,scale,cutoff,100,count);
        sampleCountEnergy=sampleCountEnergy&&Near(result.energy/emitter.PulseEnergyJ(),1,1e-10);
    }
    Check("Gaussian footprint changing sample count does not change total energy",sampleCountEnergy);
    const auto replay=Measure(emitter,scale,cutoff,100);
    Check("Gaussian footprint fixed seed projection replays exactly",replay.meanR2==at100.meanR2&&replay.halfFraction==at100.halfFraction);
    const auto anotherSeed=Measure(emitter,scale,cutoff,100,65536,43);
    Check("Gaussian footprint independent seed retains analytic width",Near(std::sqrt(2*anotherSeed.meanR2/factor)/(100*a),1,.02));

    bool rotationWidth=true,rotationEnergy=true,rotationCircles=true,rotationSupport=true;
    for(const Vector3f axis:{Vector3f(0,-1,0),Vector3f(1,1,1),Vector3f(-2,1,3)}){
        const LaserEmitter rotated(origin,axis,532,1e-8f,beam);
        const auto s=Measure(rotated,scale,cutoff,100);
        rotationWidth=rotationWidth&&Near(s.meanR2/at100.meanR2,1,.002);
        rotationEnergy=rotationEnergy&&s.energy==at100.energy;
        rotationCircles=rotationCircles&&Near(s.halfFraction,at100.halfFraction,.001)&&Near(s.scaleFraction,at100.scaleFraction,.001);
        rotationSupport=rotationSupport&&s.finite;
    }
    Check("Gaussian footprint rotated emitter preserves transverse second moment",rotationWidth);
    Check("Gaussian footprint rotated emitter preserves intercepted energy",rotationEnergy);
    Check("Gaussian footprint rotated emitter preserves encircled fractions",rotationCircles);
    Check("Gaussian footprint rotated emitter preserves forward cutoff support",rotationSupport);
    const auto small=std::make_shared<GaussianLidarBeamProfile>(1e-6,3e-6);
    const LaserEmitter smallAxis(origin,{0,0,1},532,1e-8f,small),smallRotated(origin,{1,1,1},532,1e-8f,small);
    const auto narrow=Measure(smallAxis,1e-6,3e-6,100);
    const auto narrowRotated=Measure(smallRotated,1e-6,3e-6,100);
    Check("Gaussian footprint microradian rotated width remains within numerical tolerance",Near(narrowRotated.meanR2/narrow.meanR2,1,.02));

    // Real production triangle intersection, independently compared to plane oracle.
    const Vector3f center=origin+Vector3f(0,0,10),n(0,0,-1),uv(0,0,0);
    const Vector3f p0=center+Vector3f(-1,-1,0),p1=center+Vector3f(1,-1,0),p2=center+Vector3f(-1,1,0),p3=center+Vector3f(1,1,0);
    HittableList plane;
    plane.add(std::make_shared<Triangle>(p0,p1,p2,n,n,n,uv,uv,uv,nullptr));
    plane.add(std::make_shared<Triangle>(p2,p1,p3,n,n,n,uv,uv,uv,nullptr));
    IndependentSampler random(42);const PlaneBasis oracle(emitter.Direction());
    bool allHit=true,geometryMatches=true;double intercepted=0;
    for(int i=0;i<1024;++i){
        const auto e=emitter.SampleRay(random.Get2D());const auto expected=IntersectPlane(e,oracle,10);HitRecord hit;
        if(!plane.hit(e.ray,1e-4f,std::numeric_limits<float>::infinity(),hit)){allHit=false;continue;}
        geometryMatches=geometryMatches&&Near(hit.t,expected.t,2e-5)&&Near(hit.point.z,center.z,2e-5)&&
            Near(hit.point.x-origin.x,expected.x,2e-6)&&Near(hit.point.y-origin.y,expected.y,2e-6);
        intercepted+=e.energyWeightJ/1024;
    }
    Check("Gaussian footprint real triangle plane intercepts every ray",allHit);
    Check("Gaussian footprint real triangle hits agree with independent plane intersection",geometryMatches);
    Check("Gaussian footprint real triangle full interception preserves emitted energy",Near(intercepted/emitter.PulseEnergyJ(),1,1e-10));

    // Diagnostics only, NOT acceptance of physically resolved ultranarrow beams.
    // Do not assert that collapse must persist: a later precision fix may improve it.
    for(const double width:{1e-3,1e-8,1e-50}){
        const auto b=std::make_shared<GaussianLidarBeamProfile>(width,3*width);
        const LaserEmitter e({0,0,0},{1,1,1},532,1e-8f,b);
        std::set<std::tuple<float,float,float>> local,world;
        for(int i=0;i<256;++i){
            const Point2f u(float((i+.5)/256),float((i%16+.5)/16));
            const auto l=b->SampleDirection(u).directionLocal,w=e.SampleRay(u).ray.dir;
            local.emplace(l.x,l.y,l.z);world.emplace(w.x,w.y,w.z);
        }
        std::cout<<"[DIAG] Gaussian footprint precision width="<<width<<" local_unique="<<local.size()<<" rotated_unique="<<world.size()<<'\n';
    }
}
