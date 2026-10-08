#include "OceanSlopeNormalDistribution.h"
#include "OceanNormalDistribution.h"
#include "ElfouhailySpectrum.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>

namespace {
void Expect(const char* name, bool ok) {
    if (!ok) throw std::runtime_error(std::string(name)+" failed");
    std::cout << "[PASS] " << name << '\n';
}
bool Near(double a, double b, double tolerance=2e-5) {
    return std::abs(a-b)<=tolerance*(1+std::abs(b));
}
template<class F> void Reject(const char* name, F f) {
    bool rejected=false;
    try { f(); } catch (const std::invalid_argument&) { rejected=true; }
    Expect(name,rejected);
}
}

void RunOceanSlopeNormalDistributionAcceptanceTests()
{
    constexpr double pi=std::numbers::pi_v<double>;
    const OceanSlopeVariance c{.64,.25,.12};
    const Vector3f n0=Vector3f(-1,1,-.25f).normalize();
    const OceanSlopeNormalDistribution tilted(c,Vector3f(-1,1,-.25f));
    const OceanSlopeNormalDistribution flat(c,Vector3f(0,1,0));
    const double rho=c.covarianceXZ/std::sqrt(c.varianceX*c.varianceZ);
    const double dp=.8*.3,dq=.5*(rho*.3+std::sqrt(1-rho*rho)*(-.7));
    const Vector3f sampled=tilted.NormalFromStandardNormal(.3,-.7);
    const Vector3f expected=Vector3f(float(-1-dp),1,float(-.25-dq)).normalize();
    Expect("exact ocean correlated Gaussian sample adds world slopes",
        Near(sampled.x,expected.x,1e-7) && Near(sampled.y,expected.y,1e-7) && Near(sampled.z,expected.z,1e-7));
    const Vector3f zero=tilted.SampleNormalWorld(0,.8);
    Expect("exact ocean zero uniform endpoint returns macro normal",
        Near(zero.x,n0.x,1e-7) && Near(zero.y,n0.y,1e-7) && Near(zero.z,n0.z,1e-7));
    const Vector3f endpoint=tilted.SampleNormalWorld(std::nextafter(1.0,0.0),0);
    Expect("exact ocean uniform upper interior endpoint is finite", endpoint.y>0 && Near(endpoint.dot(endpoint),1));

    // Independent inverse-covariance expression (implementation uses Cholesky coordinates).
    const Vector3f m=Vector3f(-.7f,1,-.5f).normalize();
    const double x=-double(m.x)/m.y-1, z=-double(m.z)/m.y-.25;
    const double det=c.varianceX*c.varianceZ-c.covarianceXZ*c.covarianceXZ;
    const double P=std::exp(-.5*(c.varianceZ*x*x-2*c.covarianceXZ*x*z+c.varianceX*z*z)/det)/(2*pi*std::sqrt(det));
    const double my=double(m.y)/std::hypot(double(m.x),double(m.y),double(m.z));
    const double macroY=1/std::sqrt(2.0625);
    Expect("exact ocean PDF matches analytic change of variables",Near(tilted.PdfNormalWorld(m),P/std::pow(my,3)));
    Expect("exact ocean area density uses macro-to-horizontal area ratio",Near(tilted.AreaDensityWorld(m),macroY*P/std::pow(my,4)));
    Expect("exact ocean direction scaling leaves densities unchanged",
        Near(tilted.PdfNormalWorld(m*7),tilted.PdfNormalWorld(m)) &&
        Near(tilted.AreaDensityWorld(m*7),tilted.AreaDensityWorld(m)));
    const OceanSlopeNormalDistribution scaled(c,Vector3f(-7,7,-1.75f));
    Expect("exact ocean macro-normal scaling leaves densities unchanged",
        Near(scaled.AreaDensityWorld(m),tilted.AreaDensityWorld(m),1e-12));
    const auto flatLinear=OceanNormalDistribution::FromWorldSlopeVarianceLinearized(c,Vector3f(0,1,0));
    Expect("exact ocean flat area density equals Beckmann baseline",
        Near(flat.AreaDensityWorld(m),flatLinear.DWorld(m)) && Near(flat.PdfNormalWorld(m),flat.AreaDensityWorld(m)*my));

    // Deliberately broad slopes: verify full world support, including macro-lower normals.
    const Vector3f lower=tilted.NormalFromStandardNormal(-4,0);
    Expect("exact ocean does not discard macro-lower normals",
        lower.y>0 && lower.dot(n0)<0 && tilted.PdfNormalWorld(lower)>0 && tilted.AreaDensityWorld(lower)>0);
    Expect("exact ocean world-lower and horizon densities vanish",
        tilted.PdfNormalWorld(Vector3f(0,-1,0))==0 && tilted.AreaDensityWorld(Vector3f(1,0,0))==0);
    Expect("exact ocean grazing Gaussian tail is finite",
        tilted.PdfNormalWorld(Vector3f(1,1e-30f,0))==0 && tilted.AreaDensityWorld(Vector3f(1,1e-30f,0))==0);

    // Independent world-hemisphere midpoint quadrature: dOmega=sin(theta)dtheta dphi.
    constexpr int nt=1024,np=512;
    const double dt=.5*pi/nt, df=2*pi/np, coneCos=std::cos(20*pi/180);
    double mass=0,meanX=0,meanZ=0,xx=0,zz=0,xz=0;
    double areaX=0,areaY=0,areaZ=0,positiveProjection=0,negativeProjection=0,coneMass=0;
    for (int i=0;i<nt;++i) {
        const double theta=(i+.5)*dt, st=std::sin(theta),ct=std::cos(theta);
        for (int j=0;j<np;++j) {
            const double phi=(j+.5)*df;
            const Vector3f direction(float(st*std::cos(phi)),float(ct),float(st*std::sin(phi)));
            const double dw=st*dt*df;
            const double w=tilted.PdfNormalWorld(direction)*dw;
            const double a=tilted.AreaDensityWorld(direction)*dw;
            const double sx=-double(direction.x)/direction.y-1;
            const double sz=-double(direction.z)/direction.y-.25;
            mass+=w; meanX+=sx*w; meanZ+=sz*w; xx+=sx*sx*w; zz+=sz*sz*w; xz+=sx*sz*w;
            areaX+=a*direction.x; areaY+=a*direction.y; areaZ+=a*direction.z;
            const double cosine=double(direction.x)*n0.x+double(direction.y)*n0.y+double(direction.z)*n0.z;
            if (cosine>=0) positiveProjection+=a*cosine; else negativeProjection+=a*cosine;
            if (cosine>=coneCos) coneMass+=w;
        }
    }
    Expect("exact ocean solid-angle PDF integrates to one",Near(mass,1));
    Expect("exact ocean PDF recovers zero short-slope mean",Near(meanX,0) && Near(meanZ,0));
    Expect("exact ocean PDF recovers input covariance",Near(xx,c.varianceX) && Near(zz,c.varianceZ) && Near(xz,c.covarianceXZ));
    Expect("exact ocean vector projected area equals macro normal",Near(areaX,n0.x) && Near(areaY,n0.y) && Near(areaZ,n0.z));
    Expect("exact ocean signed projected area is normalized",Near(positiveProjection+negativeProjection,1));
    // The signed projected-area weight is (r^2 + p*dp + q*dq)/r^2.
    // Its negative part has an analytic Gaussian first moment; avoid an arbitrary cutoff.
    const double r2=2.0625;
    const double sd=std::sqrt(c.varianceX+.5*c.covarianceXZ+.0625*c.varianceZ);
    const double a=r2/sd;
    const double expectedNegative=(r2*.5*std::erfc(a/std::sqrt(2.0))-
        sd*std::exp(-.5*a*a)/std::sqrt(2*pi))/r2;
    Expect("exact ocean clipping macro-lower support changes projected area",
        expectedNegative<0 && negativeProjection<0 && positiveProjection>1 &&
        Near(negativeProjection,expectedNegative,2e-6));

    // Fixed-seed iid uniforms test the public sampler against independent PDF quadrature.
    std::mt19937_64 rng(20260926);
    auto uniform=[&] {return double(rng()>>11)/9007199254740992.0;};
    constexpr int count=200000;
    double sumX=0,sumZ=0,sumXX=0,sumZZ=0,sumXZ=0;
    int coneCount=0;
    bool validSamples=true;
    for (int i=0;i<count;++i) {
        const double u1=uniform(),u2=uniform();
        const Vector3f sample=tilted.SampleNormalWorld(u1,u2);
        validSamples=validSamples && sample.y>0 && Near(sample.dot(sample),1);
        const double sx=-double(sample.x)/sample.y-1,sz=-double(sample.z)/sample.y-.25;
        sumX+=sx; sumZ+=sz; sumXX+=sx*sx; sumZZ+=sz*sz; sumXZ+=sx*sz;
        if (double(sample.x)*n0.x+double(sample.y)*n0.y+double(sample.z)*n0.z>=coneCos) ++coneCount;
    }
    Expect("exact ocean sampled normals are finite unit world-upper directions",validSamples);
    Expect("exact ocean sampled slope means match Gaussian baseline",std::abs(sumX/count)<.006 && std::abs(sumZ/count)<.006);
    Expect("exact ocean sampled covariance matches Gaussian baseline",
        std::abs(sumXX/count-c.varianceX)<.01 && std::abs(sumZZ/count-c.varianceZ)<.005 && std::abs(sumXZ/count-c.covarianceXZ)<.005);
    const double se=std::sqrt(coneMass*(1-coneMass)/count);
    Expect("exact ocean sampled cone probability matches integrated PDF",std::abs(double(coneCount)/count-coneMass)<6*se+.001);

    ElfouhailySpectrum sea(ElfouhailyConfig{});
    const auto seaStats=IntegrateSlopeVariance([&](double kx,double kz){return sea.CartesianSpectrum(float(kx),float(kz));},4,10000);
    const OceanSlopeNormalDistribution seaNormals(seaStats,Vector3f(-.3f,1,.1f));
    const auto seaSample=seaNormals.SampleNormalWorld(.3,.7);
    Expect("Elfouhaily statistics feed exact ocean normal model",
        seaSample.y>0 && seaNormals.PdfNormalWorld(seaSample)>0 && std::isfinite(seaNormals.AreaDensityWorld(seaSample)));

    const double nan=std::numeric_limits<double>::quiet_NaN();
    Reject("exact ocean rejects zero covariance",[&]{OceanSlopeNormalDistribution d({},n0);});
    Reject("exact ocean rejects singular covariance",[&]{OceanSlopeNormalDistribution d({1,1,1},n0);});
    Reject("exact ocean rejects indefinite covariance",[&]{OceanSlopeNormalDistribution d({1,1,2},n0);});
    Reject("exact ocean rejects nonfinite covariance",[&]{OceanSlopeNormalDistribution d({1,1,nan},n0);});
    Reject("exact ocean rejects downward macro normal",[&]{OceanSlopeNormalDistribution d(c,Vector3f(0,-1,0));});
    Reject("exact ocean rejects horizontal macro normal",[&]{OceanSlopeNormalDistribution d(c,Vector3f(1,0,0));});
    Reject("exact ocean rejects zero query normal",[&]{tilted.PdfNormalWorld(Vector3f());});
    Reject("exact ocean rejects nonfinite query normal",[&]{tilted.AreaDensityWorld(Vector3f(float(nan),1,0));});
    Reject("exact ocean rejects uniform value one",[&]{tilted.SampleNormalWorld(1,0);});
    Reject("exact ocean rejects negative uniform",[&]{tilted.SampleNormalWorld(0,-.1);});
    Reject("exact ocean rejects nonfinite uniform",[&]{tilted.SampleNormalWorld(nan,0);});
    Reject("exact ocean rejects nonfinite Gaussian variate",[&]{tilted.NormalFromStandardNormal(0,nan);});
}
