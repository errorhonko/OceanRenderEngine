#include "LaserEmitter.h"
#include "LidarIntegrator.h"
#include "LidarScanSequence.h"
#include "LidarReceiverPipeline.h"
#include "GaussianPulseProfile.h"
#include "LambertianLidarScattering.h"
#include "HittableList.h"
#include "Sphere.h"
#include "IndependentSampler.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace {
void Check(const char* name,bool ok){if(!ok)throw std::runtime_error(std::string("[FAIL] ")+name);std::cout<<"[PASS] "<<name<<'\n';}
template<class E,class F>bool Throws(F f){try{f();}catch(const E&){return true;}return false;}
using RaySignature=LaserEmissionSample(LaserEmitter::*)(const Point2f&,double) const;
using PulseSignature=LidarPulseResult(LidarIntegrator::*)(Sampler&,double) const;
static_assert(std::is_same_v<decltype(LaserEmissionSample::emissionTimeSeconds),double>);
static_assert(std::is_same_v<decltype(&LaserEmitter::SampleRay),RaySignature>);
static_assert(std::is_same_v<decltype(&LidarIntegrator::SimulatePulse),PulseSignature>);
static_assert(std::is_same_v<decltype(LaserEmissionSample::energyWeightJ),float>);
}

void RunLidarDoubleTimeAcceptanceTests(){
    const double time=1.00000001,step=.25e-9;
    const LaserEmitter emitter({0,0,5},{0,0,-1},532,1e-9f,0);
    const auto first=emitter.SampleRay(Point2f(.5f,.5f),time);
    const auto second=emitter.SampleRay(Point2f(.5f,.5f),time+step);
    Check("double time old float would collapse sub nanosecond step",float(time)==float(time+step));
    Check("double time emitter stores exact supplied value",first.emissionTimeSeconds==time&&first.emissionTimeSeconds!=double(float(time)));
    Check("double time emitter separates quarter ns at one second",second.emissionTimeSeconds>first.emissionTimeSeconds&&std::abs(second.emissionTimeSeconds-first.emissionTimeSeconds-step)<5e-16);
    Check("double time default emission remains zero",emitter.SampleRay(Point2f(0,0)).emissionTimeSeconds==0);
    const float legacy=2.5e-6f;
    Check("double time legacy float caller remains accepted",emitter.SampleRay(Point2f(0,0),legacy).emissionTimeSeconds==double(legacy));
    const LaserEmitter cone({0,0,5},{0,0,-1},532,1e-9f,.1f);
    Check("double time cone branch retains exact timestamp",cone.SampleRay(Point2f(.7f,.3f),time).emissionTimeSeconds==time);
    Check("double time finite value beyond float range accepted",emitter.SampleRay(Point2f(0,0),1e40).emissionTimeSeconds==1e40);
    const LidarReceiver receiver({0,0,5},{0,0,-1},.25f,.01f,.8f);
    const auto geometry=receiver.EvaluateGeometry(first,{0,0,1});
    const auto geometryNext=receiver.EvaluateGeometry(second,{0,0,1});
    constexpr double tof=8.0/299792458.0;
    Check("double time geometry preserves independent propagation time",geometry&&geometry->timeOfFlightSeconds==tof&&geometry->pathLengthMeters==8);
    Check("double time geometry arrival uses precise emission",geometry&&geometry->arrivalTimeSeconds==time+tof);
    Check("double time geometric arrivals preserve quarter ns spacing",geometry&&geometryNext&&std::abs(geometryNext->arrivalTimeSeconds-geometry->arrivalTimeSeconds-step)<5e-16);
    HittableList world(std::make_shared<Sphere>(Vector3f(0,0,0),1.0f,nullptr));
    const LambertianLidarScattering scattering(.5);
    const LidarIntegrator integrator(world,emitter,receiver,3,scattering);
    IndependentSampler sampler0(42),sampler1(42),sampler2(42);
    const auto zero=integrator.SimulatePulse(sampler0);
    const auto pulse=integrator.SimulatePulse(sampler1,time);
    const auto next=integrator.SimulatePulse(sampler2,time+step);
    Check("double time integrator produces all analytic sphere returns",pulse.returns.size()==3&&next.returns.size()==3&&zero.returns.size()==3);
    bool arrivals=true,steps=true,energies=true;
    for(std::size_t i=0;i<pulse.returns.size();++i){
        arrivals=arrivals&&pulse.returns[i].arrivalTimeSeconds==time+tof;
        steps=steps&&std::abs(next.returns[i].arrivalTimeSeconds-pulse.returns[i].arrivalTimeSeconds-step)<5e-16;
        energies=energies&&pulse.returns[i].receivedEnergyJ==zero.returns[i].receivedEnergyJ&&next.returns[i].receivedEnergyJ==zero.returns[i].receivedEnergyJ;
    }
    Check("double time integrator forwards timestamp without float narrowing",arrivals);
    Check("double time integrator preserves closely spaced pulse arrivals",steps);
    Check("double time changing timestamp leaves return energies unchanged",energies);
    LidarScanSequenceConfig scanConfig;scanConfig.originWorld={0,0,5};scanConfig.forwardWorld={0,0,-1};scanConfig.upHintWorld={0,1,0};scanConfig.azimuthCount=2;scanConfig.startTimeSeconds=time;scanConfig.pulseIntervalSeconds=step;
    const LidarScanSequence scan(scanConfig);
    bool scanTimes=true;
    for(std::size_t i=0;i<scan.Size();++i){
        const auto s=scan.Sample(i);
        scanTimes=scanTimes&&emitter.SampleRay(Point2f(.5f,.5f),s.pulse.EmissionTimeSeconds()).emissionTimeSeconds==s.pulse.EmissionTimeSeconds();
    }
    Check("double time scan metadata can feed emitter without narrowing",scanTimes);
    const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
    Check("double time emitter rejects negative time",Throws<std::invalid_argument>([&]{emitter.SampleRay(Point2f(0,0),-1.0);}));
    Check("double time emitter rejects NaN",Throws<std::invalid_argument>([&]{emitter.SampleRay(Point2f(0,0),nan);}));
    Check("double time emitter rejects infinity",Throws<std::invalid_argument>([&]{emitter.SampleRay(Point2f(0,0),inf);}));
    Check("double time integrator rejects negative time",Throws<std::invalid_argument>([&]{IndependentSampler s;integrator.SimulatePulse(s,-1.0);}));
    Check("double time integrator rejects NaN",Throws<std::invalid_argument>([&]{IndependentSampler s;integrator.SimulatePulse(s,nan);}));
    Check("double time integrator rejects infinity",Throws<std::invalid_argument>([&]{IndependentSampler s;integrator.SimulatePulse(s,inf);}));
    // Absolute record coordinates near 1 s; fast-time offsets remain tens of ns.
    LidarWaveform waveform({time,.25e-9,400});
    const GaussianPulseProfile profile(4e-9);
    for(const auto& r:pulse.returns)waveform.AccumulateReturn(r,profile);
    LidarWaveform zeroWaveform({0,.25e-9,400});
    for(const auto& r:zero.returns)zeroWaveform.AccumulateReturn(r,profile);
    double returnEnergy=0,maxBinDifference=0;
    for(const auto& r:pulse.returns)returnEnergy+=r.receivedEnergyJ;
    for(std::size_t i=0;i<waveform.EnergyBinsJ().size();++i)
        maxBinDifference=std::max(maxBinDifference,std::abs(waveform.EnergyBinsJ()[i]-zeroWaveform.EnergyBinsJ()[i]));
    Check("double time shifted pulse profile preserves total return energy",std::abs(waveform.TotalEnergyJ()-returnEnergy)<returnEnergy*1e-12);
    Check("double time shifted pulse profile matches zero clock bins",maxBinDifference<returnEnergy*1e-7);
    LidarReceiverPipelineConfig config;config.noiseSigmaVoltageV=0;
    config.noiseWindow={time,time+10e-9};config.searchWindow={time+15e-9,time+70e-9};
    config.calibration=LidarRangeCalibration(2e-9);
    const auto measured=LidarReceiverPipeline(config).Process(waveform,time,42);
    Check("double time return waveform pipeline detects echo near large clock",measured.measured&&measured.corrected);
    Check("double time pipeline corrected fast time near analytic geometry",measured.corrected&&std::abs(measured.corrected->correctedTimeOfFlightSeconds-tof)<.5e-9);
    Check("double time pipeline corrected range near four meters",measured.corrected&&std::abs(measured.corrected->correctedRangeMeters-4)<.075);
}
