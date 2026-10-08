#include "LidarDigitalBaselineEstimator.h"
#include "LidarVoltageNoise.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void Check(const char* name, bool ok) {
    if (!ok) throw std::runtime_error(std::string("[FAIL] ") + name);
    std::cout << "[PASS] " << name << '\n';
}
bool Near(double a, double b) {
    return std::isfinite(a) && std::isfinite(b) &&
        std::abs(a-b) <= 1e-12 * std::max(std::abs(a),std::abs(b));
}
template<class E,class F> bool Throws(F f) {
    try { f(); } catch (const E&) { return true; } return false;
}
LidarDigitalWaveform FromCodes(const LidarWaveformConfig& cfg,
    const std::vector<std::uint32_t>& codes,const IdealUniformAdc& adc) {
    std::vector<double> voltages;
    for(auto code:codes) voltages.push_back(adc.CodeCenterVoltageV(code));
    return LidarDigitalWaveform::FromVoltageWaveform(LidarVoltageWaveform(cfg,std::move(voltages)),adc);
}
LidarDigitalBaselineEstimate Estimate(const LidarDigitalWaveform& wave,double start,double end) {
    return LidarDigitalBaselineEstimator::Estimate(wave,{start,end});
}
}

void RunLidarDigitalBaselineAcceptanceTests() {
    const IdealUniformAdc adc(12,-1,1);
    const auto constant=FromCodes({0,1,4},{2048,2048,2048,2048},adc);
    const auto c=Estimate(constant,0,4);
    Check("baseline constant code exact mean",c.meanCode==2048);
    Check("baseline constant code zero sigma",c.sigmaCodes==0);
    Check("baseline selected sample count",c.sampleCount==4);
    const auto textbook=FromCodes({0,1,8},{2,4,4,4,5,5,7,9},adc);
    const auto t=Estimate(textbook,0,8);
    Check("baseline textbook sequence mean five",t.meanCode==5);
    Check("baseline sample variance uses N minus one",Near(t.sigmaCodes,std::sqrt(32.0/7)));
    const auto fractional=Estimate(FromCodes({0,1,2},{2047,2048},adc),0,2);
    Check("baseline preserves fractional mean code",fractional.meanCode==2047.5);
    Check("baseline two sample sigma analytic",Near(fractional.sigmaCodes,std::sqrt(.5)));
    const auto reversed=Estimate(FromCodes({0,1,8},{9,7,5,5,4,4,4,2},adc),0,8);
    Check("baseline sequence order leaves statistics unchanged",Near(reversed.meanCode,t.meanCode)&&Near(reversed.sigmaCodes,t.sigmaCodes));
    const IdealUniformAdc highAdc(24,0,1);
    const auto high=Estimate(FromCodes({0,1,3},{16777000,16777001,16777002},highAdc),0,3);
    Check("baseline large code small variation numerically stable",high.meanCode==16777001&&high.sigmaCodes==1);
    const auto fullSpan=Estimate(FromCodes({0,1,2},{0,4095},adc),0,2);
    Check("baseline full code span finite analytic sigma",fullSpan.meanCode==2047.5&&Near(fullSpan.sigmaCodes,4095/std::sqrt(2.0)));
    const auto windowWave=FromCodes({0,2,4},{100,2,4,200},adc);
    const auto selected=Estimate(windowWave,3,7);
    Check("baseline left included right excluded by centers",selected.sampleCount==2&&selected.meanCode==3&&Near(selected.sigmaCodes,std::sqrt(2.0)));
    const auto partial=Estimate(windowWave,-10,4);
    Check("baseline partial window overlap supported",partial.sampleCount==2&&partial.meanCode==51&&Near(partial.sigmaCodes,98/std::sqrt(2.0)));
    const auto negativeTime=Estimate(FromCodes({-4,2,4},{2,4,100,200},adc),-4,0);
    Check("baseline negative time noise window allowed",negativeTime.sampleCount==2&&negativeTime.meanCode==3);
    Check("baseline empty window rejected",Throws<std::invalid_argument>([&]{Estimate(windowWave,20,30);}));
    Check("baseline one selected center rejected",Throws<std::invalid_argument>([&]{Estimate(windowWave,3,4);}));
    Check("baseline one bin record rejected",Throws<std::invalid_argument>([&]{Estimate(FromCodes({0,1,1},{10},adc),0,1);}));
    Check("baseline zero duration rejected",Throws<std::invalid_argument>([&]{Estimate(windowWave,3,3);}));
    Check("baseline reversed bounds rejected",Throws<std::invalid_argument>([&]{Estimate(windowWave,7,3);}));
    const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
    Check("baseline NaN start rejected",Throws<std::invalid_argument>([&]{Estimate(windowWave,nan,7);}));
    Check("baseline NaN end rejected",Throws<std::invalid_argument>([&]{Estimate(windowWave,0,nan);}));
    Check("baseline infinite start rejected",Throws<std::invalid_argument>([&]{Estimate(windowWave,-inf,7);}));
    Check("baseline infinite end rejected",Throws<std::invalid_argument>([&]{Estimate(windowWave,0,inf);}));
    const IdealUniformAdc unipolar(12,0,1);
    const auto clipped=LidarDigitalWaveform::FromVoltageWaveform(LidarVoltageWaveform({0,2,4},{-.1,.25,.5,1.2}),unipolar);
    const auto cleanPart=Estimate(clipped,3,7);
    Check("baseline clipping outside noise window ignored",clipped.HasOutOfRangeSamples()&&cleanPart.sampleCount==2&&cleanPart.meanCode==1536&&Near(cleanPart.sigmaCodes,1024/std::sqrt(2.0)));
    Check("baseline low clipping inside window rejected",Throws<std::domain_error>([&]{Estimate(clipped,0,7);}));
    Check("baseline high clipping inside window rejected",Throws<std::domain_error>([&]{Estimate(clipped,3,8);}));
    const auto endpoints=LidarDigitalWaveform::FromVoltageWaveform(LidarVoltageWaveform({0,1,2},{0,1}),unipolar);
    Check("baseline endpoint codes alone do not imply clipping",Estimate(endpoints,0,2).meanCode==2047.5);

    constexpr std::size_t count=200000;
    constexpr double sigmaV=1e-3;
    const LidarVoltageWaveform zeroVoltage({0,1e-9,count},std::vector<double>(count,0));
    const auto noisy=LidarVoltageNoise(sigmaV).Apply(zeroVoltage,20261008);
    const auto digits=LidarDigitalWaveform::FromVoltageWaveform(noisy,adc);
    const double endTime=count*1e-9;
    const auto result=Estimate(digits,0,endTime);
    Check("baseline noisy record sample count retained",result.sampleCount==count);
    long double sum=0,squares=0;
    for(const auto& s:digits.Samples())sum+=s.code;
    const long double directMean=sum/count;
    for(const auto& s:digits.Samples()){const long double delta=s.code-directMean;squares+=delta*delta;}
    const double directSigma=std::sqrt(double(squares/(count-1)));
    Check("baseline noisy mean matches independent direct sum",std::abs(result.meanCode-double(directMean))<1e-8);
    Check("baseline noisy sigma matches independent centered sum",std::abs(result.sigmaCodes-directSigma)<1e-9);
    // Model the actual ADC intervals, not sigmaV/LSB as an exact prediction.
    // Saturating outer intervals extend to infinity; at this sigma their mass is negligible.
    long double predictedVariance=0;
    const double lsb=adc.LsbVoltageV();
    for(std::uint32_t code=0;code<=adc.MaximumCode();++code){
        const double lo=code==0?-inf:(-1+code*lsb)/sigmaV;
        const double hi=code==adc.MaximumCode()?inf:(-1+(code+1)*lsb)/sigmaV;
        const double probability=.5*(std::erf(hi/std::sqrt(2.0))-std::erf(lo/std::sqrt(2.0)));
        const long double delta=double(code)-2047.5;
        predictedVariance+=probability*delta*delta;
    }
    const double predictedSigma=std::sqrt(double(predictedVariance));
    Check("baseline quantized noise mean near 2047 point five",std::abs(result.meanCode-2047.5)<6*predictedSigma/std::sqrt(double(count)));
    Check("baseline quantized variance agrees with ADC interval model",std::abs(result.sigmaCodes*result.sigmaCodes/double(predictedVariance)-1)<6*std::sqrt(2.0/(count-1)));
    const auto shiftedDigits=LidarDigitalWaveform::FromVoltageWaveform(LidarVoltageNoise(sigmaV).Apply(LidarVoltageWaveform({0,1e-9,count},std::vector<double>(count,.125)),20261008),adc);
    const auto shifted=Estimate(shiftedDigits,0,endTime);
    Check("baseline voltage offset shifts mean by 256 codes",std::abs(shifted.meanCode-result.meanCode-256)<1e-8);
    Check("baseline constant offset preserves noise sigma",std::abs(shifted.sigmaCodes-result.sigmaCodes)<1e-9);
    const auto exactZero=Estimate(LidarDigitalWaveform::FromVoltageWaveform(zeroVoltage,adc),0,endTime);
    Check("baseline zero analog noise has exact zero digital sigma",exactZero.meanCode==2048&&exactZero.sigmaCodes==0);
    const auto repeated=Estimate(digits,0,endTime);
    Check("baseline repeated call deterministic",repeated.meanCode==result.meanCode&&repeated.sigmaCodes==result.sigmaCodes&&repeated.sampleCount==result.sampleCount);
    Check("baseline estimator does not alter digital samples",digits.Samples()[0].code==adc.Quantize(noisy.AverageVoltageBinsV()[0]).code&&digits.Samples().size()==count);
    const auto clippedNoise=LidarDigitalWaveform::FromVoltageWaveform(noisy,unipolar);
    Check("baseline noisy unipolar clipping rejected",Throws<std::domain_error>([&]{Estimate(clippedNoise,0,endTime);}));
    std::cout<<"[INFO] baseline samples="<<result.sampleCount<<" mean code="<<result.meanCode
        <<" sigma codes="<<result.sigmaCodes<<" predicted quantized sigma="<<predictedSigma
        <<" sigma times LSB mV="<<result.sigmaCodes*lsb*1000<<'\n';
}
