#include "LidarReceiverPipeline.h"
#include "GaussianPulseProfile.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
void Check(const char* name,bool ok) {
    if(!ok)throw std::runtime_error(std::string("[FAIL] ")+name);
    std::cout<<"[PASS] "<<name<<'\n';
}
template<class E,class F>bool Throws(F f) {
    try{f();}catch(const E&){return true;}return false;
}
LidarReceiverPipelineConfig Configuration() {
    LidarReceiverPipelineConfig c;
    c.noiseWindow={20e-9,80e-9};c.searchWindow={90e-9,125e-9};
    return c;
}
LidarWaveform Signal(double energy=1e-12,double arrival=100.2e-9) {
    LidarWaveform wave({0,.25e-9,1024});
    wave.AccumulateReturn({energy,arrival,532},GaussianPulseProfile(4e-9));
    return wave;
}
bool Same(const LidarDigitalRangeEstimate& a,const LidarDigitalRangeEstimate& b) {
    return a.peakBinIndex==b.peakBinIndex&&a.arrivalTimeSeconds==b.arrivalTimeSeconds&&
        a.timeOfFlightSeconds==b.timeOfFlightSeconds&&a.rangeMeters==b.rangeMeters&&
        a.peakCode==b.peakCode&&a.peakAboveBaselineCodes==b.peakAboveBaselineCodes&&
        a.peakOutOfRange==b.peakOutOfRange&&a.searchHasOutOfRangeSamples==b.searchHasOutOfRangeSamples;
}
bool Same(const LidarReceiverPipelineResult& a,const LidarReceiverPipelineResult& b) {
    if(a.baseline.meanCode!=b.baseline.meanCode||a.baseline.sigmaCodes!=b.baseline.sigmaCodes||
       a.baseline.sampleCount!=b.baseline.sampleCount||a.thresholdCodes!=b.thresholdCodes||
       a.hasAdcOutOfRangeSamples!=b.hasAdcOutOfRangeSamples||bool(a.measured)!=bool(b.measured)||bool(a.corrected)!=bool(b.corrected))return false;
    if(a.measured&&!Same(*a.measured,*b.measured))return false;
    if(a.corrected&&(!Same(a.corrected->measured,b.corrected->measured)||
       a.corrected->correctedTimeOfFlightSeconds!=b.corrected->correctedTimeOfFlightSeconds||
       a.corrected->correctedRangeMeters!=b.corrected->correctedRangeMeters))return false;
    return true;
}
LidarReceiverPipelineResult Manual(const LidarReceiverPipelineConfig& c,const LidarWaveform& optical,double emission,std::uint64_t seed) {
    const auto current=LidarAnalogWaveform::FromOpticalWaveform(optical,LinearPhotodetector(c.wavelengthNm,c.quantumEfficiency));
    const auto response=FirstOrderLidarResponse(c.responseTimeConstantSeconds).Apply(current);
    const auto voltage=LidarVoltageWaveform::FromCurrentWaveform(response,IdealTransimpedanceAmplifier(c.transimpedanceGainVoltsPerAmp));
    const auto noisy=LidarVoltageNoise(c.noiseSigmaVoltageV).Apply(voltage,seed);
    const auto digits=LidarDigitalWaveform::FromVoltageWaveform(noisy,IdealUniformAdc(c.adcBitCount,c.adcMinimumVoltageV,c.adcMaximumVoltageV));
    LidarReceiverPipelineResult result;
    result.baseline=LidarDigitalBaselineEstimator::Estimate(digits,c.noiseWindow);
    result.hasAdcOutOfRangeSamples=digits.HasOutOfRangeSamples();
    result.thresholdCodes=std::max(c.minimumThresholdCodes,c.thresholdMultiplier*result.baseline.sigmaCodes);
    if(result.thresholdCodes>=double(digits.Adc().MaximumCode())-result.baseline.meanCode)return result;
    const LidarDigitalRangeEstimator estimator(result.baseline.meanCode,result.thresholdCodes);
    switch(c.rangeMethod){
    case LidarReceiverRangeMethod::Peak:result.measured=estimator.Estimate(digits,emission,c.searchWindow);break;
    case LidarReceiverRangeMethod::Centroid:result.measured=estimator.EstimateCentroid(digits,emission,c.searchWindow);break;
    case LidarReceiverRangeMethod::PeakNeighborhoodCentroid:result.measured=estimator.EstimatePeakNeighborhoodCentroid(digits,emission,c.neighborhood,c.searchWindow);break;
    }
    if(result.measured&&c.calibration)result.corrected=c.calibration->Apply(*result.measured);
    return result;
}
}

void RunLidarReceiverPipelineAcceptanceTests() {
    auto config=Configuration();
    const auto optical=Signal();const auto before=optical.EnergyBinsJ();
    for(auto method:{LidarReceiverRangeMethod::Peak,LidarReceiverRangeMethod::Centroid,LidarReceiverRangeMethod::PeakNeighborhoodCentroid}){
        bool matches=true;
        config.rangeMethod=method;config.calibration=LidarRangeCalibration(2e-9);
        for(double sigma:{0.0,1e-3})for(std::uint64_t seed:{42ULL,43ULL}){
            config.noiseSigmaVoltageV=sigma;
            matches=matches&&Same(LidarReceiverPipeline(config).Process(optical,0,seed),Manual(config,optical,0,seed));
        }
        Check(method==LidarReceiverRangeMethod::Peak?"pipeline peak matches manual chain":method==LidarReceiverRangeMethod::Centroid?"pipeline centroid matches manual chain":"pipeline neighborhood matches manual chain",matches);
    }
    config=Configuration();
    const LidarReceiverPipeline pipeline(config);
    const auto result=pipeline.Process(optical,0,42);
    Check("pipeline exposes estimated baseline sample count",result.baseline.sampleCount==240);
    Check("pipeline exposes actual estimated noise threshold",result.thresholdCodes==6*result.baseline.sigmaCodes);
    Check("pipeline default method detects strong return",result.measured.has_value());
    Check("pipeline no calibration leaves corrected empty",!result.corrected);
    Check("pipeline same seed reproduces entire result",Same(result,pipeline.Process(optical,0,42)));
    const auto another=pipeline.Process(optical,0,43);
    Check("pipeline changed seed changes estimated noise realization",another.baseline.meanCode!=result.baseline.meanCode||another.baseline.sigmaCodes!=result.baseline.sigmaCodes);
    Check("pipeline leaves optical input unchanged",optical.EnergyBinsJ()==before);
    config.noiseSigmaVoltageV=0;config.searchWindow={150e-9,185e-9};
    Check("pipeline owns independent configuration snapshot",pipeline.Config().noiseSigmaVoltageV==1e-3&&pipeline.Config().searchWindow.startTimeSeconds==90e-9&&Same(result,pipeline.Process(optical,0,42)));
    config=Configuration();config.noiseSigmaVoltageV=0;
    const auto clean=LidarReceiverPipeline(config).Process(optical,0,42);
    Check("pipeline zero noise yields exact baseline and zero sigma",clean.baseline.meanCode==2048&&clean.baseline.sigmaCodes==0&&clean.thresholdCodes==0);
    const auto empty=LidarReceiverPipeline(config).Process(Signal(0),0,42);
    Check("pipeline no return has empty measurement not zero distance",!empty.measured&&!empty.corrected&&empty.baseline.sampleCount==240);
    config.minimumThresholdCodes=4;
    const auto floor=LidarReceiverPipeline(config).Process(optical,0,42);
    Check("pipeline minimum threshold works with zero estimated sigma",floor.thresholdCodes==4&&floor.measured.has_value());
    config.minimumThresholdCodes=5000;
    const auto disabled=LidarReceiverPipeline(config).Process(optical,0,42);
    Check("pipeline unattainable threshold safely returns no detection",disabled.thresholdCodes==5000&&!disabled.measured&&!disabled.corrected);
    config=Configuration();config.calibration=LidarRangeCalibration(2e-9);
    const auto calibrated=LidarReceiverPipeline(config).Process(optical,0,42);
    Check("pipeline supplied fixed calibration applied",calibrated.measured&&calibrated.corrected&&calibrated.corrected->correctedTimeOfFlightSeconds==calibrated.measured->timeOfFlightSeconds-2e-9);
    Check("pipeline calibration preserves raw measured result",calibrated.measured&&calibrated.corrected&&Same(*calibrated.measured,calibrated.corrected->measured));
    config.calibration=LidarRangeCalibration(1);
    const auto negativeCorrection=LidarReceiverPipeline(config).Process(optical,0,42);
    Check("pipeline invalid corrected time keeps raw detection",negativeCorrection.measured&&!negativeCorrection.corrected);
    config=Configuration();config.noiseSigmaVoltageV=0;config.quantumEfficiency=0;
    Check("pipeline zero quantum efficiency produces no signal",!LidarReceiverPipeline(config).Process(optical,0,42).measured);
    config=Configuration();config.responseTimeConstantSeconds=0;
    Check("pipeline zero response time uses existing bypass",Same(LidarReceiverPipeline(config).Process(optical,0,42),Manual(config,optical,0,42)));
    config=Configuration();config.neighborhood={0,0};
    const auto one=LidarReceiverPipeline(config).Process(optical,0,42);
    config.rangeMethod=LidarReceiverRangeMethod::Peak;
    const auto peak=LidarReceiverPipeline(config).Process(optical,0,42);
    Check("pipeline one bin neighborhood equals peak time",one.measured&&peak.measured&&one.measured->arrivalTimeSeconds==peak.measured->arrivalTimeSeconds);
    config=Configuration();config.noiseSigmaVoltageV=0;
    const auto outside=LidarReceiverPipeline(config).Process(Signal(1e-9,200e-9),0,42);
    Check("pipeline global ADC clipping exposed outside search",outside.hasAdcOutOfRangeSamples&&!outside.measured);
    const auto saturated=LidarReceiverPipeline(config).Process(Signal(1e-9),0,42);
    Check("pipeline measured clipping flags are not hidden",saturated.hasAdcOutOfRangeSamples&&saturated.measured&&saturated.measured->peakOutOfRange&&saturated.measured->searchHasOutOfRangeSamples);
    config=Configuration();config.adcMinimumVoltageV=0;
    Check("pipeline clipped baseline window propagates domain error",Throws<std::domain_error>([&]{LidarReceiverPipeline(config).Process(optical,0,42);}));
    config=Configuration();config.noiseWindow={20e-9,20.1e-9};
    Check("pipeline insufficient baseline samples rejected",Throws<std::invalid_argument>([&]{LidarReceiverPipeline(config).Process(optical,0,42);}));
    const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
    Check("pipeline negative emission rejected",Throws<std::invalid_argument>([&]{pipeline.Process(optical,-1,42);}));
    Check("pipeline NaN emission rejected",Throws<std::invalid_argument>([&]{pipeline.Process(optical,nan,42);}));
    Check("pipeline infinite emission rejected",Throws<std::invalid_argument>([&]{pipeline.Process(optical,inf,42);}));
    Check("pipeline unset windows rejected",Throws<std::invalid_argument>([]{LidarReceiverPipeline bad(LidarReceiverPipelineConfig{});}));
    config=Configuration();config.noiseWindow={20e-9,20e-9};
    Check("pipeline zero duration noise window rejected",Throws<std::invalid_argument>([&]{LidarReceiverPipeline bad(config);}));
    config=Configuration();config.searchWindow={125e-9,90e-9};
    Check("pipeline reversed search window rejected",Throws<std::invalid_argument>([&]{LidarReceiverPipeline bad(config);}));
    config=Configuration();config.noiseWindow.startTimeSeconds=nan;
    Check("pipeline nonfinite window rejected",Throws<std::invalid_argument>([&]{LidarReceiverPipeline bad(config);}));
    config=Configuration();config.searchWindow={70e-9,125e-9};
    Check("pipeline overlapping noise search windows rejected",Throws<std::invalid_argument>([&]{LidarReceiverPipeline bad(config);}));
    config=Configuration();config.searchWindow.startTimeSeconds=80e-9;
    Check("pipeline touching half open windows accepted",LidarReceiverPipeline(config).Config().searchWindow.startTimeSeconds==80e-9);
    config=Configuration();config.transimpedanceGainVoltsPerAmp=-10000;
    Check("pipeline negative pulse polarity rejected",Throws<std::invalid_argument>([&]{LidarReceiverPipeline bad(config);}));
    config=Configuration();config.thresholdMultiplier=-1;
    Check("pipeline negative threshold multiplier rejected",Throws<std::invalid_argument>([&]{LidarReceiverPipeline bad(config);}));
    config=Configuration();config.thresholdMultiplier=nan;
    Check("pipeline NaN multiplier rejected",Throws<std::invalid_argument>([&]{LidarReceiverPipeline bad(config);}));
    config=Configuration();config.minimumThresholdCodes=-1;
    Check("pipeline negative minimum threshold rejected",Throws<std::invalid_argument>([&]{LidarReceiverPipeline bad(config);}));
    config=Configuration();config.minimumThresholdCodes=inf;
    Check("pipeline infinite minimum threshold rejected",Throws<std::invalid_argument>([&]{LidarReceiverPipeline bad(config);}));
    config=Configuration();config.rangeMethod=static_cast<LidarReceiverRangeMethod>(127);
    Check("pipeline invalid range method rejected",Throws<std::invalid_argument>([&]{LidarReceiverPipeline bad(config);}));
    config=Configuration();config.quantumEfficiency=2;
    Check("pipeline invalid detector config rejected by module",Throws<std::invalid_argument>([&]{LidarReceiverPipeline bad(config);}));
    config=Configuration();config.adcBitCount=0;
    Check("pipeline invalid ADC config rejected by module",Throws<std::invalid_argument>([&]{LidarReceiverPipeline bad(config);}));
    config=Configuration();config.noiseSigmaVoltageV=.01;config.thresholdMultiplier=std::numeric_limits<double>::max();
    Check("pipeline threshold multiplication overflow detected",Throws<std::overflow_error>([&]{LidarReceiverPipeline(config).Process(optical,0,42);}));
}
