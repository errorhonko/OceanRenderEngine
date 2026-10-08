#include "LidarPointCloudBuilder.h"
#include "GaussianPulseProfile.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double hc=.5*299792458.0;
void Check(const char* name,bool ok){if(!ok)throw std::runtime_error(std::string("[FAIL] ")+name);std::cout<<"[PASS] "<<name<<'\n';}
bool Near(double a,double b){return std::abs(a-b)<1e-5;}
bool Same(const Vector3f&a,const Vector3f&b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
template<class E,class F>bool Throws(F f){try{f();}catch(const E&){return true;}return false;}
LidarPulseMeasurement Make(double range=12,Vector3f origin={0,10,0},Vector3f direction={0,-1,0}){
    LidarReceiverPipelineResult result;
    result.measured=LidarDigitalRangeEstimate{5,.125+range/hc,range/hc,range,2080,32.5,false,false};
    return {LidarPulseMetadata(7,.125,origin,direction),result};
}
}

void RunLidarPointCloudBuilderAcceptanceTests(){
    const LidarPointCloudBuilder raw(LidarPointRangeSource::Raw),corrected(LidarPointRangeSource::Corrected);
    auto measurement=Make();
    const auto rawPoint=raw.BuildPoint(measurement);
    Check("point builder raw downward world position analytic",rawPoint&&Same(rawPoint->positionWorld,{0,-2,0}));
    Check("point builder preserves pulse id and emission timestamp",rawPoint&&rawPoint->pulseId==7&&rawPoint->emissionTimeSeconds==.125);
    Check("point builder preserves raw arrival timestamp",rawPoint&&rawPoint->arrivalTimeSeconds==measurement.reception.measured->arrivalTimeSeconds);
    Check("point builder exposes raw range source",rawPoint&&rawPoint->rangeMeters==12&&rawPoint->rangeSource==LidarPointRangeSource::Raw&&raw.RangeSource()==LidarPointRangeSource::Raw);
    Check("point builder preserves normalized center direction",rawPoint&&Same(rawPoint->centerDirectionWorld,{0,-1,0}));
    Check("point builder preserves code and fractional amplitude",rawPoint&&rawPoint->peakCode==2080&&rawPoint->peakAboveBaselineCodes==32.5);
    Check("point builder corrected source does not silently fall back",!corrected.BuildPoint(measurement));
    measurement.reception.corrected=LidarRangeCalibration(2/hc).Apply(*measurement.reception.measured);
    const auto correctedPoint=corrected.BuildPoint(measurement);
    Check("point builder corrected distance selected",correctedPoint&&Near(correctedPoint->rangeMeters,10)&&correctedPoint->rangeSource==LidarPointRangeSource::Corrected);
    Check("point builder corrected distance projects to sea plane",correctedPoint&&Near(correctedPoint->positionWorld.y,0)&&correctedPoint->positionWorld.x==0&&correctedPoint->positionWorld.z==0);
    Check("point builder corrected point retains raw arrival time",correctedPoint&&correctedPoint->arrivalTimeSeconds==measurement.reception.measured->arrivalTimeSeconds);
    Check("point builder raw selection unaffected by available correction",raw.BuildPoint(measurement)->positionWorld.y==-2);
    Check("point builder does not modify input range",measurement.reception.measured->rangeMeters==12&&Near(measurement.reception.corrected->correctedRangeMeters,10));
    auto missed=measurement;missed.reception.measured.reset();missed.reception.corrected.reset();
    Check("point builder missed pulse produces no point",!raw.BuildPoint(missed)&&!corrected.BuildPoint(missed));
    auto invalidCorrection=Make();invalidCorrection.reception.corrected=LidarRangeCalibration(1).Apply(*invalidCorrection.reception.measured);
    Check("point builder rejected correction remains unavailable",!corrected.BuildPoint(invalidCorrection)&&raw.BuildPoint(invalidCorrection).has_value());
    const auto zero=raw.BuildPoint(Make(0,{1,2,3}));
    Check("point builder valid zero range equals origin",zero&&Same(zero->positionWorld,{1,2,3}));
    const auto diagonal=raw.BuildPoint(Make(5,{0,0,0},{3,4,0}));
    Check("point builder diagonal direction projects analytically",diagonal&&Near(diagonal->positionWorld.x,3)&&Near(diagonal->positionWorld.y,4)&&diagonal->positionWorld.z==0);
    const auto horizontal=raw.BuildPoint(Make(2,{1,2,3},{0,0,4}));
    Check("point builder normalization avoids range scaling",horizontal&&Same(horizontal->positionWorld,{1,2,5}));
    auto clipped=Make();clipped.reception.measured->peakOutOfRange=true;clipped.reception.hasAdcOutOfRangeSamples=true;
    Check("point builder peak clipping rejected by default",raw.RejectClippedReturns()&&!raw.BuildPoint(clipped));
    clipped.reception.measured->peakOutOfRange=false;clipped.reception.measured->searchHasOutOfRangeSamples=true;
    Check("point builder nonpeak search clipping rejected by default",!raw.BuildPoint(clipped));
    const LidarPointCloudBuilder allowClipping(LidarPointRangeSource::Raw,false);
    const auto allowed=allowClipping.BuildPoint(clipped);
    Check("point builder clipping acceptance is configurable",allowed&&!allowClipping.RejectClippedReturns());
    Check("point builder accepted clipping keeps diagnostics",allowed&&allowed->searchHasOutOfRangeSamples&&allowed->hasAdcOutOfRangeSamples&&!allowed->peakOutOfRange);
    auto outsideClipping=Make();outsideClipping.reception.hasAdcOutOfRangeSamples=true;
    const auto outsidePoint=raw.BuildPoint(outsideClipping);
    Check("point builder record clipping outside search is diagnostic only",outsidePoint&&outsidePoint->hasAdcOutOfRangeSamples&&!outsidePoint->searchHasOutOfRangeSamples);
    clipped.reception.corrected=LidarRangeCalibration(2/hc).Apply(*clipped.reception.measured);
    Check("point builder corrected source honors raw clipping diagnosis",!corrected.BuildPoint(clipped));
    const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
    auto bad=Make();bad.reception.measured->rangeMeters=-1;
    Check("point builder negative range rejected",Throws<std::invalid_argument>([&]{raw.BuildPoint(bad);}));
    bad=Make();bad.reception.measured->rangeMeters=nan;
    Check("point builder NaN range rejected",Throws<std::invalid_argument>([&]{raw.BuildPoint(bad);}));
    bad=Make();bad.reception.measured->rangeMeters=inf;
    Check("point builder infinite range rejected",Throws<std::invalid_argument>([&]{raw.BuildPoint(bad);}));
    bad=Make();bad.reception.measured->peakAboveBaselineCodes=nan;
    Check("point builder nonfinite strength rejected",Throws<std::invalid_argument>([&]{raw.BuildPoint(bad);}));
    bad=Make();bad.reception.measured->peakAboveBaselineCodes=-1;
    Check("point builder negative strength rejected",Throws<std::invalid_argument>([&]{raw.BuildPoint(bad);}));
    bad=Make();bad.reception.measured->arrivalTimeSeconds=nan;
    Check("point builder invalid arrival time rejected",Throws<std::invalid_argument>([&]{raw.BuildPoint(bad);}));
    bad=Make();bad.reception.measured->timeOfFlightSeconds=-1;
    Check("point builder negative raw propagation time rejected",Throws<std::invalid_argument>([&]{raw.BuildPoint(bad);}));
    bad=measurement;bad.reception.corrected->correctedRangeMeters=-1;
    Check("point builder invalid corrected range rejected",Throws<std::invalid_argument>([&]{corrected.BuildPoint(bad);}));
    bad=measurement;bad.reception.corrected->correctedTimeOfFlightSeconds=nan;
    Check("point builder invalid corrected propagation time rejected",Throws<std::invalid_argument>([&]{corrected.BuildPoint(bad);}));
    Check("point builder invalid range source rejected",Throws<std::invalid_argument>([]{LidarPointCloudBuilder badSource(static_cast<LidarPointRangeSource>(127));}));
    const auto enormous=Make(std::numeric_limits<double>::max());
    Check("point builder huge finite range checked before float conversion",Throws<std::overflow_error>([&]{raw.BuildPoint(enormous);}));
    const auto floatMax=std::numeric_limits<float>::max();
    const auto additionOverflow=Make(double(floatMax),{floatMax,0,0},{1,0,0});
    Check("point builder origin plus range overflow rejected",Throws<std::overflow_error>([&]{raw.BuildPoint(additionOverflow);}));

    LidarReceiverPipelineConfig config;config.noiseWindow={20e-9,80e-9};config.searchWindow={90e-9,125e-9};
    config.noiseSigmaVoltageV=0;config.calibration=LidarRangeCalibration(2e-9);
    const LidarReceiverPipeline pipeline(config);
    const LaserEmitter emitter({0,10,0},{0,-1,0},532,1e-6f,.1f);
    const auto pulse=LidarPulseMetadata::FromEmitter(501,0,emitter);
    LidarWaveform wave({0,.25e-9,1024});wave.AccumulateReturn({1e-12,100.2e-9,532},GaussianPulseProfile(4e-9));
    const LidarPulseMeasurement processed{pulse,pipeline.Process(wave,pulse.EmissionTimeSeconds(),42)};
    const auto chainPoint=corrected.BuildPoint(processed);
    Check("point builder receiver pipeline chain produces point",chainPoint&&chainPoint->pulseId==501&&chainPoint->rangeSource==LidarPointRangeSource::Corrected);
    Check("point builder complete chain uses center direction not sampled ray",chainPoint&&chainPoint->positionWorld.x==0&&chainPoint->positionWorld.z==0&&Same(chainPoint->centerDirectionWorld,emitter.Direction()));
    Check("point builder complete chain projection matches selected distance",chainPoint&&processed.reception.corrected&&Near(chainPoint->positionWorld.y,10-processed.reception.corrected->correctedRangeMeters));
    const LidarPulseMeasurement noReturn{pulse,pipeline.Process(LidarWaveform({0,.25e-9,1024}),0,42)};
    Check("point builder actual missed return produces no false origin point",!raw.BuildPoint(noReturn)&&!corrected.BuildPoint(noReturn));
    std::vector<LidarPointSample> points;
    for(const auto& record:{measurement,missed,processed}){if(auto p=corrected.BuildPoint(record))points.push_back(*p);}
    Check("point builder caller can collect only valid pulse points",points.size()==2&&points[0].pulseId==7&&points[1].pulseId==501);
}
