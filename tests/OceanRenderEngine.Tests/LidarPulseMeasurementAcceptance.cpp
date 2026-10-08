#include "LidarPulseMeasurement.h"
#include "GaussianPulseProfile.h"
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace {
void Check(const char* name,bool ok){
    if(!ok)throw std::runtime_error(std::string("[FAIL] ")+name);
    std::cout<<"[PASS] "<<name<<'\n';
}
bool Same(const Vector3f& a,const Vector3f& b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
static_assert(!std::is_default_constructible_v<LidarPulseMeasurement>);
static_assert(std::is_copy_constructible_v<LidarPulseMeasurement>);
}

void RunLidarPulseMeasurementAcceptanceTests(){
    LidarPulseMetadata metadata(17,10e-9,{0,10,0},{0,-1,0});
    LidarReceiverPipelineResult reception;
    reception.baseline={2047.5,2.0,240};
    reception.thresholdCodes=12;
    reception.hasAdcOutOfRangeSamples=true;
    reception.measured=LidarDigitalRangeEstimate{5,110e-9,100e-9,.5*299792458.0*100e-9,2080,32.5,true,true};
    reception.corrected=LidarRangeCalibration(2e-9).Apply(*reception.measured);
    const LidarPulseMeasurement measurement{metadata,reception};
    Check("pulse measurement retains id and double timestamp",measurement.pulse.PulseId()==17&&measurement.pulse.EmissionTimeSeconds()==10e-9);
    Check("pulse measurement retains world position and central direction",Same(measurement.pulse.OriginWorld(),{0,10,0})&&Same(measurement.pulse.CenterDirectionWorld(),{0,-1,0}));
    Check("pulse measurement retains baseline diagnostics",measurement.reception.baseline.meanCode==2047.5&&measurement.reception.baseline.sigmaCodes==2&&measurement.reception.baseline.sampleCount==240);
    Check("pulse measurement retains threshold and record clipping",measurement.reception.thresholdCodes==12&&measurement.reception.hasAdcOutOfRangeSamples);
    Check("pulse measurement retains raw distance and fractional strength",measurement.reception.measured&&measurement.reception.measured->rangeMeters==reception.measured->rangeMeters&&measurement.reception.measured->peakAboveBaselineCodes==32.5);
    Check("pulse measurement retains peak and search clipping flags",measurement.reception.measured&&measurement.reception.measured->peakOutOfRange&&measurement.reception.measured->searchHasOutOfRangeSamples);
    Check("pulse measurement retains corrected result and embedded raw result",measurement.reception.corrected&&measurement.reception.corrected->correctedRangeMeters==reception.corrected->correctedRangeMeters&&measurement.reception.corrected->measured.peakAboveBaselineCodes==32.5);
    metadata=LidarPulseMetadata(99,20e-9,{5,5,5},{1,0,0});
    reception.baseline.meanCode=0;reception.measured.reset();reception.corrected.reset();
    Check("pulse measurement independent from original metadata",measurement.pulse.PulseId()==17&&Same(measurement.pulse.OriginWorld(),{0,10,0}));
    Check("pulse measurement independent from original receiver result",measurement.reception.baseline.meanCode==2047.5&&measurement.reception.measured&&measurement.reception.corrected);
    const LidarPulseMeasurement missed{metadata,reception};
    Check("pulse measurement missed return still has pulse metadata",missed.pulse.PulseId()==99&&missed.pulse.EmissionTimeSeconds()==20e-9&&!missed.reception.measured&&!missed.reception.corrected);
    auto rawOnly=measurement.reception;rawOnly.corrected.reset();
    const LidarPulseMeasurement uncalibrated{metadata,rawOnly};
    Check("pulse measurement raw result without correction supported",uncalibrated.reception.measured&&!uncalibrated.reception.corrected);
    const auto copy=measurement;
    Check("pulse measurement copied result retains association",copy.pulse.PulseId()==measurement.pulse.PulseId()&&copy.reception.measured->rangeMeters==measurement.reception.measured->rangeMeters);
    const std::vector<LidarPulseMeasurement> records{measurement,missed};
    Check("pulse measurement collection preserves detected and missed records",records.size()==2&&records[0].pulse.PulseId()==17&&records[0].reception.measured&&records[1].pulse.PulseId()==99&&!records[1].reception.measured);

    LidarReceiverPipelineConfig config;
    config.noiseWindow={20e-9,80e-9};config.searchWindow={90e-9,125e-9};
    config.noiseSigmaVoltageV=0;config.calibration=LidarRangeCalibration(2e-9);
    const LidarReceiverPipeline pipeline(config);
    LaserEmitter emitter({0,10,0},{0,-1,0},532,1e-6f,.1f);
    const auto realMetadata=LidarPulseMetadata::FromEmitter(101,10e-9,emitter);
    LidarWaveform wave({0,.25e-9,1024});
    wave.AccumulateReturn({1e-12,100.2e-9,532},GaussianPulseProfile(4e-9));
    const LidarPulseMeasurement processed{realMetadata,pipeline.Process(wave,realMetadata.EmissionTimeSeconds(),42)};
    Check("pulse measurement binds actual pipeline output",processed.pulse.PulseId()==101&&processed.reception.measured&&processed.reception.corrected);
    Check("pulse measurement pipeline uses metadata emission timestamp",processed.reception.measured&&processed.reception.measured->timeOfFlightSeconds==processed.reception.measured->arrivalTimeSeconds-processed.pulse.EmissionTimeSeconds());
    const LidarPulseMeasurement noReturn{realMetadata,pipeline.Process(LidarWaveform({0,.25e-9,1024}),realMetadata.EmissionTimeSeconds(),42)};
    Check("pulse measurement actual no return pipeline record retained",noReturn.pulse.PulseId()==101&&!noReturn.reception.measured&&!noReturn.reception.corrected&&noReturn.reception.baseline.sampleCount==240);
}
