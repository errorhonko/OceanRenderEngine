#include "LinearPhotodetector.h"
#include "LidarAnalogWaveform.h"
#include "GaussianPulseProfile.h"
#include "LidarRangeEstimator.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
void Check(const char* name, bool condition) {
    if (!condition) throw std::runtime_error(std::string("[FAIL] ")+name);
    std::cout << "[PASS] " << name << '\n';
}
bool RelativeNear(double a,double b,double tolerance=1e-12) {
    return std::isfinite(a) && std::isfinite(b) &&
        std::abs(a-b)<=tolerance*std::max(std::abs(a),std::abs(b));
}
template<class Exception,class F> bool Throws(F f) {
    try { f(); } catch(const Exception&) { return true; }
    return false;
}
}

void RunLidarAnalogAcceptanceTests() {
    constexpr double q=1.602176634e-19,h=6.62607015e-34,c=299792458;
    const LinearPhotodetector detector(532,.6);
    const double response=.6*q*532e-9/(h*c);
    const double onePhotonEnergy=h*c/532e-9;
    Check("detector analytic responsivity A per W",RelativeNear(detector.ResponsivityAmpsPerWatt(),response));
    Check("detector one photon mean charge eta times q",RelativeNear(detector.EnergyToChargeC(onePhotonEnergy),.6*q));
    Check("detector one pJ analytic charge",RelativeNear(detector.EnergyToChargeC(1e-12),response*1e-12));
    Check("detector ten ns average current",RelativeNear(detector.EnergyToAverageCurrentA(1e-12,10e-9),response*1e-4));
    Check("detector zero energy zero charge",detector.EnergyToChargeC(0)==0);
    Check("detector zero energy zero current",detector.EnergyToAverageCurrentA(0,1e-9)==0);
    const LinearPhotodetector zero(532,0),full(532,1),red(1064,.6);
    Check("detector zero QE zero response",zero.ResponsivityAmpsPerWatt()==0);
    Check("detector zero QE zero signal",zero.EnergyToAverageCurrentA(1e-12,1e-9)==0);
    Check("detector full QE one electron mean per photon",RelativeNear(full.EnergyToChargeC(onePhotonEnergy),q));
    Check("detector doubled energy doubled charge",RelativeNear(detector.EnergyToChargeC(2e-12),2*detector.EnergyToChargeC(1e-12)));
    Check("detector doubled bin width halves current",RelativeNear(detector.EnergyToAverageCurrentA(1e-12,2e-9),.5*detector.EnergyToAverageCurrentA(1e-12,1e-9)));
    // 固定QE的受控比较；不宣称真实器件在532和1064nm的QE相同。
    Check("detector fixed QE doubled wavelength doubles response",RelativeNear(red.ResponsivityAmpsPerWatt(),2*response));
    const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
    Check("detector rejects zero wavelength",Throws<std::invalid_argument>([]{LinearPhotodetector bad(0,.6);}));
    Check("detector rejects nonfinite wavelength",Throws<std::invalid_argument>([]{LinearPhotodetector bad(std::numeric_limits<float>::infinity(),.6);}));
    Check("detector rejects negative QE",Throws<std::invalid_argument>([]{LinearPhotodetector bad(532,-.1);}));
    Check("detector rejects QE above one",Throws<std::invalid_argument>([]{LinearPhotodetector bad(532,1.1);}));
    Check("detector rejects NaN QE",Throws<std::invalid_argument>([&]{LinearPhotodetector bad(532,nan);}));
    Check("detector rejects negative energy",Throws<std::invalid_argument>([&]{detector.EnergyToChargeC(-1);}));
    Check("detector rejects NaN energy",Throws<std::invalid_argument>([&]{detector.EnergyToChargeC(nan);}));
    Check("detector rejects infinite energy",Throws<std::invalid_argument>([&]{detector.EnergyToChargeC(inf);}));
    Check("detector rejects zero bin width",Throws<std::invalid_argument>([&]{detector.EnergyToAverageCurrentA(1e-12,0);}));
    Check("detector rejects negative bin width",Throws<std::invalid_argument>([&]{detector.EnergyToAverageCurrentA(1e-12,-1);}));
    Check("detector rejects infinite bin width",Throws<std::invalid_argument>([&]{detector.EnergyToAverageCurrentA(1e-12,inf);}));
    const double maximum=std::numeric_limits<double>::max();
    // 极端波长仅检查算术溢出防护，不代表物理器件工作范围。
    Check("detector charge overflow rejected",Throws<std::overflow_error>([&]{LinearPhotodetector huge(1e10f,1);huge.EnergyToChargeC(maximum);}));
    Check("detector current overflow rejected",Throws<std::overflow_error>([&]{detector.EnergyToAverageCurrentA(100,std::numeric_limits<double>::min());}));

    LidarWaveform optical({10e-9,2e-9,4});
    optical.AccumulateReturn({1e-12,11e-9,532});
    optical.AccumulateReturn({2e-12,13e-9,532});
    optical.AccumulateReturn({3e-12,15e-9,532});
    const auto before=optical.EnergyBinsJ();
    const auto analog=LidarAnalogWaveform::FromOpticalWaveform(optical,detector);
    Check("analog preserves time configuration",analog.Config().startTimeSeconds==optical.Config().startTimeSeconds && analog.Config().binWidthSeconds==2e-9 && analog.Config().binCount==4);
    bool times=true,conversion=true;
    for(std::size_t i=0;i<before.size();++i) {
        times=times && analog.BinCenterTimeSeconds(i)==optical.BinCenterTimeSeconds(i);
        conversion=conversion && RelativeNear(analog.AverageCurrentBinsA()[i],response*before[i]/2e-9);
    }
    Check("analog preserves every bin center",times);
    Check("analog every current bin matches analytic conversion",conversion);
    Check("analog total charge equals response times energy",RelativeNear(analog.TotalChargeC(),response*6e-12));
    Check("analog conversion leaves optical truth unchanged",optical.EnergyBinsJ()==before);
    optical.Clear();
    Check("analog owns converted snapshot",analog.AverageCurrentBinsA()[0]>0 && optical.TotalEnergyJ()==0);
    const auto zeroSignal=LidarAnalogWaveform::FromOpticalWaveform(optical,detector);
    Check("analog zero waveform remains zero",zeroSignal.TotalChargeC()==0);
    LidarWaveform nonzero({0,1e-9,2});nonzero.AccumulateReturn({1e-12,.5e-9,532});
    Check("analog zero QE produces zero charge",LidarAnalogWaveform::FromOpticalWaveform(nonzero,zero).TotalChargeC()==0);
    LidarAnalogWaveform signedSignal({0,.5,3},{1,-2,3});
    Check("analog signed signal container allowed",signedSignal.AverageCurrentBinsA()[1]==-2 && signedSignal.TotalChargeC()==1);
    Check("analog bin center bounds checked",Throws<std::out_of_range>([&]{analog.BinCenterTimeSeconds(4);}));
    Check("analog rejects mismatched vector length",Throws<std::invalid_argument>([]{LidarAnalogWaveform bad({0,1,2},{1});}));
    Check("analog rejects empty configuration",Throws<std::invalid_argument>([]{LidarAnalogWaveform bad({0,1,0},{});}));
    Check("analog rejects nonfinite start time",Throws<std::invalid_argument>([&]{LidarAnalogWaveform bad({nan,1,1},{0});}));
    Check("analog rejects zero bin width",Throws<std::invalid_argument>([]{LidarAnalogWaveform bad({0,0,1},{0});}));
    Check("analog rejects NaN current",Throws<std::invalid_argument>([&]{LidarAnalogWaveform bad({0,1,1},{nan});}));
    Check("analog rejects duration overflow",Throws<std::invalid_argument>([&]{LidarAnalogWaveform bad({0,maximum,2},{0,0});}));
    Check("analog rejects window end overflow",Throws<std::invalid_argument>([&]{LidarAnalogWaveform bad({maximum,maximum,1},{0});}));
    Check("analog detects per bin charge overflow",Throws<std::overflow_error>([&]{LidarAnalogWaveform signal({0,2,1},{maximum});signal.TotalChargeC();}));
    Check("analog detects total charge overflow",Throws<std::overflow_error>([&]{LidarAnalogWaveform signal({0,1,2},{maximum,maximum});signal.TotalChargeC();}));
    // 用独立电流加权计算验证线性转换不改变时间，不将A冒充J。
    GaussianPulseProfile profile(4e-9);
    LidarWaveform shaped({0,.5e-9,512});shaped.AccumulateReturn({1e-12,100.2e-9,532},profile);
    const auto current=LidarAnalogWaveform::FromOpticalWaveform(shaped,detector);
    const auto& values=current.AverageCurrentBinsA();
    const auto index=std::size_t(std::max_element(values.begin(),values.end())-values.begin());
    LidarRangeEstimator estimator;
    Check("analog positive linear conversion preserves peak index",index==estimator.Estimate(shaped,0)->peakBinIndex);
    long double moment=0,total=0;
    for(std::size_t i=0;i<values.size();++i) {total+=values[i];moment+=static_cast<long double>(values[i])*current.BinCenterTimeSeconds(i);}
    Check("analog positive linear conversion preserves centroid",std::abs(double(moment/total)-estimator.EstimateCentroid(shaped,0)->arrivalTimeSeconds)<1e-18);
    Check("analog shaped signal charge conservation",RelativeNear(current.TotalChargeC(),response*shaped.TotalEnergyJ()));
}
