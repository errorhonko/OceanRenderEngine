#include "FirstOrderLidarResponse.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
void Check(const char* name,bool condition) {
    if(!condition) throw std::runtime_error(std::string("[FAIL] ")+name);
    std::cout << "[PASS] " << name << '\n';
}
bool Near(double a,double b,double relative=1e-11,double absolute=1e-15) {
    return std::isfinite(a) && std::isfinite(b) &&
        std::abs(a-b)<=absolute+relative*std::max(std::abs(a),std::abs(b));
}
template<class F> bool Invalid(F f) {
    try {f();} catch(const std::invalid_argument&){return true;}
    return false;
}
double Centroid(const LidarAnalogWaveform& waveform) {
    long double moment=0,total=0;
    const auto& bins=waveform.AverageCurrentBinsA();
    for(std::size_t i=0;i<bins.size();++i){
        total+=bins[i]; moment+=static_cast<long double>(bins[i])*waveform.BinCenterTimeSeconds(i);
    }
    if(total<=0) throw std::runtime_error("Invalid diagnostic centroid");
    return double(moment/total);
}
std::size_t Peak(const LidarAnalogWaveform& waveform) {
    const auto& bins=waveform.AverageCurrentBinsA();
    return std::size_t(std::max_element(bins.begin(),bins.end())-bins.begin());
}
}

void RunFirstOrderLidarResponseAcceptanceTests() {
    constexpr double dt=1e-9,tau=2e-9;
    FirstOrderLidarResponse response(tau),identity(0);
    Check("response exposes time constant",response.TimeConstantSeconds()==tau);
    Check("response rejects negative time constant",Invalid([]{FirstOrderLidarResponse bad(-1);}));
    Check("response rejects NaN time constant",Invalid([]{FirstOrderLidarResponse bad(std::numeric_limits<double>::quiet_NaN());}));
    Check("response rejects infinite time constant",Invalid([]{FirstOrderLidarResponse bad(std::numeric_limits<double>::infinity());}));
    LidarAnalogWaveform mixed({1e-3,dt,4},{0,1,-2,3});
    const auto direct=identity.Apply(mixed);
    Check("response zero tau exact bypass",direct.AverageCurrentBinsA()==mixed.AverageCurrentBinsA());
    Check("response bypass preserves charge",direct.TotalChargeC()==mixed.TotalChargeC());
    Check("response preserves time grid",response.Apply(mixed).Config().startTimeSeconds==1e-3 && response.Apply(mixed).Config().binWidthSeconds==dt && response.Apply(mixed).Config().binCount==4);
    LidarAnalogWaveform zeros({0,dt,32},std::vector<double>(32,0));
    Check("response zero input zero output",response.Apply(zeros).AverageCurrentBinsA()==zeros.AverageCurrentBinsA());
    LidarAnalogWaveform step({0,dt,16},std::vector<double>(16,1));
    const auto stepOutput=response.Apply(step);
    // 独立解析积分：每箱平均值 = 1 - tau/dt*(exp(-t0/tau)-exp(-t1/tau))。
    bool analytic=true;
    for(std::size_t i=0;i<16;++i){
        const long double t0=static_cast<long double>(i)*dt,t1=static_cast<long double>(i+1)*dt;
        const double expected=double(1-static_cast<long double>(tau)/dt*(std::exp(-t0/tau)-std::exp(-t1/tau)));
        analytic=analytic && Near(stepOutput.AverageCurrentBinsA()[i],expected);
    }
    Check("response constant input matches analytic bin integral",analytic);
    Check("response output is average not end sample",std::abs(stepOutput.AverageCurrentBinsA()[0]-(1-std::exp(-dt/tau)))>.1);
    Check("response step output monotone",std::is_sorted(stepOutput.AverageCurrentBinsA().begin(),stepOutput.AverageCurrentBinsA().end()));
    Check("response step approaches unity DC gain",stepOutput.AverageCurrentBinsA().back()>.999 && stepOutput.AverageCurrentBinsA().back()<1);
    constexpr std::size_t pulseIndex=3;
    std::vector<double> pulse(200,0);pulse[pulseIndex]=1;
    LidarAnalogWaveform input({0,dt,pulse.size()},pulse);
    const auto before=input.AverageCurrentBinsA();
    const auto output=response.Apply(input);
    const auto& values=output.AverageCurrentBinsA();
    Check("response input snapshot unchanged",input.AverageCurrentBinsA()==before);
    bool causal=true;
    for(std::size_t i=0;i<pulseIndex;++i) causal=causal && values[i]==0;
    Check("response causal no output before pulse",causal);
    Check("response positive input stays nonnegative",std::all_of(values.begin(),values.end(),[](double x){return std::isfinite(x) && x>=0;}));
    const long double a=std::exp(-static_cast<long double>(dt)/tau);
    const long double b=static_cast<long double>(tau)/dt*(1-a);
    Check("response pulse first box analytic",Near(values[pulseIndex],double(1-b)));
    Check("response pulse first tail box analytic",Near(values[pulseIndex+1],double(b*(1-a))));
    bool tail=true;
    for(std::size_t i=pulseIndex+1;i<values.size();++i){
        const double expected=double(b*(1-a)*std::exp(-static_cast<long double>(i-pulseIndex-1)*dt/tau));
        tail=tail && Near(values[i],expected,1e-11,1e-25);
    }
    Check("response entire pulse tail matches exponential integral",tail);
    Check("response tail successive ratio analytic",Near(values[pulseIndex+2]/values[pulseIndex+1],double(a)));
    Check("response sufficiently long window preserves total charge",Near(output.TotalChargeC(),input.TotalChargeC(),1e-12,1e-24));
    std::vector<double> shortPulse(8,0);shortPulse[pulseIndex]=1;
    LidarAnalogWaveform shortInput({0,dt,8},shortPulse);
    const auto shortOutput=response.Apply(shortInput);
    const double residual=double(static_cast<long double>(tau)*(1-a)*std::exp(-static_cast<long double>(8-pulseIndex-1)*dt/tau));
    Check("response finite window charge matches residual state",Near(shortOutput.TotalChargeC(),dt-residual,1e-12,1e-24));
    Check("response truncated tail not renormalized",shortOutput.TotalChargeC()<shortInput.TotalChargeC());
    Check("response longer window recovers more charge",output.TotalChargeC()>shortOutput.TotalChargeC());
    Check("response independently reset apply repeatable",response.Apply(input).AverageCurrentBinsA()==values);
    auto doubled=pulse;for(auto& x:doubled)x*=2;
    auto negative=pulse;for(auto& x:negative)x=-x;
    const auto twice=response.Apply(LidarAnalogWaveform({0,dt,200},doubled));
    const auto sign=response.Apply(LidarAnalogWaveform({0,dt,200},negative));
    bool linear=true,signedLinear=true;
    for(std::size_t i=0;i<values.size();++i){
        linear=linear && Near(twice.AverageCurrentBinsA()[i],2*values[i]);
        signedLinear=signedLinear && Near(sign.AverageCurrentBinsA()[i],-values[i]);
    }
    Check("response amplitude linearity",linear);
    Check("response signed signal linearity",signedLinear);
    const auto fast=FirstOrderLidarResponse(1e-15).Apply(input);
    Check("response fast finite tau approaches bypass",std::abs(fast.AverageCurrentBinsA()[pulseIndex]-1)<2e-6);
    const auto tinyRatio=FirstOrderLidarResponse(1).Apply(LidarAnalogWaveform({0,1e-12,1},{1}));
    Check("response small ratio series retains nonzero change",Near(tinyRatio.AverageCurrentBinsA()[0],5e-13,1e-11,1e-25));
    const auto infiniteRatio=FirstOrderLidarResponse(std::numeric_limits<double>::min()).Apply(LidarAnalogWaveform({0,10,1},{1}));
    Check("response ratio overflow uses fast response limit",infiniteRatio.AverageCurrentBinsA()[0]==1);
    const double centroidShift=Centroid(output)-Centroid(input);
    const double peakShift=(double(Peak(output))-double(Peak(input)))*dt;
    // 完整拖尾、零状态、无背景的本例；不是所有测距算法的通用校正。
    Check("response full pulse centroid delay equals tau in this case",std::abs(centroidShift-tau)<1e-18);
    Check("response peak delay differs from tau",std::abs(peakShift-dt)<1e-18 && std::abs(peakShift-tau)>.5e-9);
    Check("response truncated centroid differs from full tail",Centroid(shortOutput)<Centroid(output));
    std::cout << "[INFO] response tau_ns=" << tau*1e9 << " centroid_shift_ns=" << centroidShift*1e9 << " peak_shift_ns=" << peakShift*1e9 << '\n';
}
