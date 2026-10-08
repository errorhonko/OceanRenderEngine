#include "LidarVoltageWaveform.h"
#include "FirstOrderLidarResponse.h"
#include "GaussianPulseProfile.h"
#include <algorithm>
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
bool Near(double a,double b) {
    return std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=1e-12*std::max(std::abs(a),std::abs(b));
}
template<class E,class F>bool Throws(F f) {
    try{f();}catch(const E&){return true;}return false;
}
}

void RunLidarVoltageAcceptanceTests() {
    IdealTransimpedanceAmplifier gain(10e3),inverse(-10e3);
    Check("TIA stores gain V per A",gain.GainVoltsPerAmp()==10e3);
    Check("TIA 25 microamp gives 0.25 V",Near(gain.CurrentToVoltageV(25e-6),.25));
    Check("TIA zero current zero voltage",gain.CurrentToVoltageV(0)==0);
    Check("TIA doubled current doubled voltage",Near(gain.CurrentToVoltageV(50e-6),.5));
    Check("TIA negative current retained",Near(gain.CurrentToVoltageV(-25e-6),-.25));
    Check("TIA negative gain reverses polarity",Near(inverse.CurrentToVoltageV(25e-6),-.25));
    const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity(),maximum=std::numeric_limits<double>::max();
    Check("TIA rejects zero gain",Throws<std::invalid_argument>([]{IdealTransimpedanceAmplifier bad(0);}));
    Check("TIA rejects NaN gain",Throws<std::invalid_argument>([&]{IdealTransimpedanceAmplifier bad(nan);}));
    Check("TIA rejects infinite gain",Throws<std::invalid_argument>([&]{IdealTransimpedanceAmplifier bad(inf);}));
    Check("TIA rejects NaN current",Throws<std::invalid_argument>([&]{gain.CurrentToVoltageV(nan);}));
    Check("TIA rejects infinite current",Throws<std::invalid_argument>([&]{gain.CurrentToVoltageV(inf);}));
    Check("TIA detects voltage overflow",Throws<std::overflow_error>([&]{gain.CurrentToVoltageV(maximum);}));
    LidarAnalogWaveform input({10e-9,2e-9,4},{0,25e-6,-25e-6,50e-6});
    const auto before=input.AverageCurrentBinsA();
    const auto voltage=LidarVoltageWaveform::FromCurrentWaveform(input,gain);
    const auto reversed=LidarVoltageWaveform::FromCurrentWaveform(input,inverse);
    bool values=true,polarity=true,times=true;
    for(std::size_t i=0;i<4;++i){
        values=values&&Near(voltage.AverageVoltageBinsV()[i],before[i]*1e4);
        polarity=polarity&&Near(reversed.AverageVoltageBinsV()[i],-before[i]*1e4);
        times=times&&voltage.BinCenterTimeSeconds(i)==input.BinCenterTimeSeconds(i);
    }
    Check("voltage waveform every bin analytic",values);
    Check("voltage waveform signed polarity conversion",polarity);
    Check("voltage waveform time configuration retained",voltage.Config().startTimeSeconds==10e-9&&voltage.Config().binWidthSeconds==2e-9&&voltage.Config().binCount==4);
    Check("voltage waveform every bin time retained",times);
    Check("voltage conversion does not modify input",input.AverageCurrentBinsA()==before);
    input=LidarAnalogWaveform({10e-9,2e-9,4},{0,0,0,0});
    Check("voltage waveform owns independent snapshot",Near(voltage.AverageVoltageBinsV()[1],.25)&&input.AverageCurrentBinsA()[1]==0);
    const auto zero=LidarVoltageWaveform::FromCurrentWaveform(input,gain);
    Check("voltage zero signal remains zero",std::all_of(zero.AverageVoltageBinsV().begin(),zero.AverageVoltageBinsV().end(),[](double v){return v==0;}));
    Check("voltage rejects length mismatch",Throws<std::invalid_argument>([]{LidarVoltageWaveform bad({0,1,2},{1});}));
    Check("voltage rejects empty grid",Throws<std::invalid_argument>([]{LidarVoltageWaveform bad({0,1,0},{});}));
    Check("voltage rejects invalid start",Throws<std::invalid_argument>([&]{LidarVoltageWaveform bad({nan,1,1},{0});}));
    Check("voltage rejects zero bin width",Throws<std::invalid_argument>([]{LidarVoltageWaveform bad({0,0,1},{0});}));
    Check("voltage rejects negative bin width",Throws<std::invalid_argument>([]{LidarVoltageWaveform bad({0,-1,1},{0});}));
    Check("voltage rejects infinite bin width",Throws<std::invalid_argument>([&]{LidarVoltageWaveform bad({0,inf,1},{0});}));
    Check("voltage rejects nonfinite value",Throws<std::invalid_argument>([&]{LidarVoltageWaveform bad({0,1,1},{inf});}));
    Check("voltage rejects duration overflow",Throws<std::invalid_argument>([&]{LidarVoltageWaveform bad({0,maximum,2},{0,0});}));
    Check("voltage rejects window end overflow",Throws<std::invalid_argument>([&]{LidarVoltageWaveform bad({maximum,maximum,1},{0});}));
    Check("voltage bin index bounds checked",Throws<std::out_of_range>([&]{voltage.BinCenterTimeSeconds(4);}));
    Check("voltage waveform conversion detects overflow",Throws<std::overflow_error>([&]{LidarVoltageWaveform::FromCurrentWaveform(LidarAnalogWaveform({0,1,1},{maximum}),gain);}));
    // 物理链路：光学能量 -> 光电流 -> 接收响应 -> 跨阻电压。
    LidarWaveform optical({0,.25e-9,1024});
    optical.AccumulateReturn({1e-12,100.2e-9,532},GaussianPulseProfile(4e-9));
    LinearPhotodetector detector(532,.6);
    const auto ideal=LidarAnalogWaveform::FromOpticalWaveform(optical,detector);
    const auto received=FirstOrderLidarResponse(2e-9).Apply(ideal);
    const auto readout=LidarVoltageWaveform::FromCurrentWaveform(received,gain);
    const auto& currents=received.AverageCurrentBinsA();const auto& volts=readout.AverageVoltageBinsV();
    bool chain=true;long double iSum=0,vSum=0,iMoment=0,vMoment=0;
    for(std::size_t i=0;i<currents.size();++i){
        chain=chain&&Near(volts[i],currents[i]*1e4);
        iSum+=currents[i];vSum+=volts[i];
        iMoment+=static_cast<long double>(currents[i])*received.BinCenterTimeSeconds(i);
        vMoment+=static_cast<long double>(volts[i])*readout.BinCenterTimeSeconds(i);
    }
    Check("voltage optical response readout chain analytic",chain);
    Check("voltage positive gain preserves peak index",std::max_element(currents.begin(),currents.end())-currents.begin()==std::max_element(volts.begin(),volts.end())-volts.begin());
    Check("voltage positive gain preserves centroid time",std::abs(double(iMoment/iSum-vMoment/vSum))<1e-18);
    // 积分电压单位为V*s，只用于验证增益，不称为能量。
    Check("voltage time integral equals gain times charge",Near(double(vSum)*readout.Config().binWidthSeconds,1e4*received.TotalChargeC()));
}
