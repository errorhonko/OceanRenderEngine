#include "LidarRangeEstimator.h"
#include "GaussianPulseProfile.h"
#include "Sphere.h"
#include "HittableList.h"
#include "IndependentSampler.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
void Check(const char* name, bool ok) {
    if (!ok) throw std::runtime_error(std::string("[FAIL] ") + name);
    std::cout << "[PASS] " << name << '\n';
}
bool Near(double a, double b, double tolerance=1e-18) {
    return std::isfinite(a) && std::isfinite(b) && std::abs(a-b)<=tolerance;
}
template<class F> bool Invalid(F f) {
    try { f(); } catch(const std::invalid_argument&) { return true; }
    return false;
}
}

void RunLidarRangeEstimatorAcceptanceTests() {
    constexpr double c=299792458.0;
    LidarRangeEstimator estimator;
    LidarWaveform empty({0,1e-9,256});
    Check("range empty peak not detected", !estimator.Estimate(empty,0));
    Check("range empty centroid not detected", !estimator.EstimateCentroid(empty,0));
    LidarWaveform single({0,1e-9,256});
    single.AccumulateReturn({1e-12,100.2e-9,532});
    auto peak=estimator.Estimate(single,0), centroid=estimator.EstimateCentroid(single,0);
    Check("range single box detected",peak && centroid);
    Check("range peak correct index and energy",peak->peakBinIndex==100 && peak->peakBinEnergyJ==1e-12);
    Check("range peak uses bin center",Near(peak->arrivalTimeSeconds,100.5e-9));
    Check("range single station conversion",Near(peak->rangeMeters,.5*c*100.5e-9,1e-12));
    Check("range single box peak equals centroid",Near(peak->rangeMeters,centroid->rangeMeters,1e-12));
    Check("range impulse half box quantization bound",std::abs(peak->rangeMeters-.5*c*100.2e-9)<=c*1e-9/4);
    LidarWaveform pair({0,1e-9,256});
    pair.AccumulateReturn({8e-12,100.2e-9,532});
    pair.AccumulateReturn({2e-12,101.2e-9,532});
    auto cp=estimator.EstimateCentroid(pair,0);
    Check("range centroid weighted hand calculation",cp && Near(cp->arrivalTimeSeconds,100.7e-9));
    Check("range centroid range hand calculation",Near(cp->rangeMeters,.5*c*100.7e-9,1e-12));
    Check("range centroid retains peak diagnostics",cp->peakBinIndex==100 && cp->peakBinEnergyJ==8e-12);
    LidarWaveform equal({0,1e-9,256});
    equal.AccumulateReturn({1e-12,100.2e-9,532}); equal.AccumulateReturn({1e-12,101.2e-9,532});
    Check("range equal peaks select earlier box",estimator.Estimate(equal,0)->peakBinIndex==100);
    Check("range equal weights centroid midpoint",Near(estimator.EstimateCentroid(equal,0)->arrivalTimeSeconds,101e-9));
    LidarWaveform scaled({0,1e-9,256});
    scaled.AccumulateReturn({8e-9,100.2e-9,532}); scaled.AccumulateReturn({2e-9,101.2e-9,532});
    Check("range centroid uniform energy scale invariant",Near(estimator.EstimateCentroid(scaled,0)->arrivalTimeSeconds,cp->arrivalTimeSeconds));
    LidarRangeEstimator threshold(1e-12);
    Check("range peak threshold equality not detected",!threshold.Estimate(single,0));
    Check("range centroid shares detection threshold",!threshold.EstimateCentroid(single,0));
    Check("range rejects negative threshold",Invalid([]{LidarRangeEstimator bad(-1);}));
    Check("range rejects NaN threshold",Invalid([]{LidarRangeEstimator bad(std::numeric_limits<double>::quiet_NaN());}));
    Check("range rejects negative emission",Invalid([&]{estimator.Estimate(pair,-1);}));
    Check("range centroid rejects nonfinite emission",Invalid([&]{estimator.EstimateCentroid(pair,std::numeric_limits<double>::infinity());}));
    // 使用非零绝对发射时刻，检验相对时延而非绝对时刻参与测距。
    const double emission=1e-3;
    LidarWaveform shifted({emission,1e-9,256});
    shifted.AccumulateReturn({8e-12,emission+100.2e-9,532});
    shifted.AccumulateReturn({2e-12,emission+101.2e-9,532});
    auto sp=estimator.Estimate(shifted,emission),sc=estimator.EstimateCentroid(shifted,emission);
    Check("range nonzero emission peak delay",Near(sp->timeOfFlightSeconds,100.5e-9,1e-15));
    Check("range nonzero emission centroid delay",Near(sc->timeOfFlightSeconds,100.7e-9,1e-15));
    Check("range nonzero emission centroid distance",Near(sc->rangeMeters,cp->rangeMeters,1e-7));
    LidarWaveform gated({0,1e-9,256});
    gated.AccumulateReturn({1e-9,10.2e-9,532}); gated.AccumulateReturn({1e-12,100.2e-9,532});
    Check("range ignores pre emission peak",estimator.Estimate(gated,50e-9)->peakBinIndex==100);
    Check("range centroid ignores pre emission energy",Near(estimator.EstimateCentroid(gated,50e-9)->timeOfFlightSeconds,50.5e-9));
    Check("range all energy before emission not detected",!estimator.EstimateCentroid(single,200e-9));
    // 完整对称高斯回波，中心不位于箱中心；这里不声称一般波形均无偏。
    GaussianPulseProfile profile(4e-9);
    LidarWaveform gaussian({0,.5e-9,512});
    gaussian.AccumulateReturn({1e-12,100.2e-9,532},profile);
    auto gp=estimator.Estimate(gaussian,0),gc=estimator.EstimateCentroid(gaussian,0);
    Check("range Gaussian pulse centroid recovers center",gc && Near(gc->arrivalTimeSeconds,100.2e-9,1e-14));
    Check("range Gaussian centroid beats box peak in this case",std::abs(gc->arrivalTimeSeconds-100.2e-9)<std::abs(gp->arrivalTimeSeconds-100.2e-9));
    // 有限窗口只保留右半波：展示截断会改变重心，并非算法保证恢复中心。
    LidarWaveform clipped({100.2e-9,.5e-9,100});
    clipped.AccumulateReturn({1e-12,100.2e-9,532},profile);
    Check("range truncated waveform centroid shifts late",estimator.EstimateCentroid(clipped,0)->arrivalTimeSeconds>100.2e-9);
    // 真正经过发射、求交、接收、脉冲形状及波形的单站链路。
    const Vector3f up(0,1,0),sensor(0,15,0);
    HittableList world(std::make_shared<Sphere>(Vector3f(0,-1,0),1,nullptr));
    LaserEmitter emitter(sensor,-up,532,1e-3f,0);
    LidarReceiver receiver(sensor,-up,.25f,.01f,.8f);
    ConstantLidarScattering scattering(.2);
    LidarIntegrator lidar(world,emitter,receiver,16,scattering);
    IndependentSampler sampler(42);
    const auto pulse=lidar.SimulatePulse(sampler);
    LidarWaveform received({0,.5e-9,512}); received.AccumulatePulse(pulse,profile);
    auto measured=estimator.EstimateCentroid(received,0);
    Check("range lidar geometry waveform chain detected",pulse.returns.size()==16 && measured);
    Check("range lidar waveform centroid matches 15 m truth",Near(measured->rangeMeters,15,1e-6));
    // 有限合法输入，归一化后先遇到零权重：必须跳过而非计算 0/0。
    LidarWaveform extreme({0,1e-9,256});
    extreme.AccumulateReturn({1e-300,10.2e-9,532});
    extreme.AccumulateReturn({1e100,100.2e-9,532});
    const auto extremeEstimate=estimator.EstimateCentroid(extreme,0);
    Check("range centroid leading weight underflow remains finite",
        extremeEstimate && std::isfinite(extremeEstimate->rangeMeters));
    Check("range centroid underflow retains dominant arrival time",
        Near(extremeEstimate->arrivalTimeSeconds,100.5e-9));
    Check("range centroid underflow retains dominant range",
        Near(extremeEstimate->rangeMeters,.5*c*100.5e-9,1e-12));
    // 峰值在最前面，后续零权重同样不改变结果。
    LidarWaveform trailingTiny({0,1e-9,256});
    trailingTiny.AccumulateReturn({1e100,10.2e-9,532});
    trailingTiny.AccumulateReturn({1e-300,100.2e-9,532});
    Check("range centroid trailing weight underflow ignored",
        Near(estimator.EstimateCentroid(trailingTiny,0)->arrivalTimeSeconds,10.5e-9));
}
