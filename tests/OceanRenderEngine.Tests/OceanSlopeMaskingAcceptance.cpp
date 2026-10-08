#include "OceanSlopeNormalDistribution.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>

// Test-only prototype: normal-independent masking except for backface support.
// This checks the projected-area closure, NOT visibility of an explicit microsurface.
namespace {
constexpr double pi=std::numbers::pi_v<double>;
struct V {
    double x,y,z;
    V Unit() const { const double l=std::hypot(x,y,z); return {x/l,y/l,z/l}; }
    double Dot(V b) const { return x*b.x+y*b.y+z*b.z; }
    V operator+(V b) const { return {x+b.x,y+b.y,z+b.z}; }
    V operator*(double s) const { return {x*s,y*s,z*s}; }
};
void Expect(const std::string& name, bool ok) {
    if (!ok) throw std::runtime_error(name+" failed");
    std::cout << "[PASS] " << name << '\n';
}
bool Near(double x,double y,double abs=2e-6,double rel=2e-5) {
    return std::abs(x-y)<=abs+rel*std::abs(y);
}
double PositiveGaussianMoment(double mu,double sigma) {
    if (sigma==0) return std::max(0.0,mu);
    const double a=mu/sigma;
    return sigma*std::exp(-.5*a*a)/std::sqrt(2*pi)+mu*.5*std::erfc(-a/std::sqrt(2.0));
}
// Independent quadrature of integral max(0,mu+sigma*z) phi(z) dz.
// Split exactly at the kink and truncate the standard normal at +/-12.
double NumericalMoment(double mu,double sigma) {
    if (sigma==0) return std::max(0.0,mu);
    const double lo=std::max(-12.0,-mu/sigma),hi=12;
    if (lo>=hi) return 0;
    constexpr int count=100000;
    const double dz=(hi-lo)/count;
    double sum=0;
    for (int i=0;i<count;++i) {
        const double z=lo+(i+.5)*dz;
        sum+=(mu+sigma*z)*std::exp(-.5*z*z)/std::sqrt(2*pi)*dz;
    }
    return sum;
}
struct Candidate {
    OceanSlopeVariance cov;
    double p,q;
    V n;
    Candidate(OceanSlopeVariance c,double p,double q):cov(c),p(p),q(q),n(V{-p,1,-q}.Unit()) {}
    double Mu(V w) const { return w.y-p*w.x-q*w.z; }
    double Sigma(V w) const {
        return std::sqrt(std::max(0.0,w.x*w.x*cov.varianceX+
            2*w.x*w.z*cov.covarianceXZ+w.z*w.z*cov.varianceZ));
    }
    double Area(V w) const { return n.y*PositiveGaussianMoment(Mu(w),Sigma(w)); }
    double Factor(V w) const {
        const double mu=Mu(w);
        if (mu<=0) return 0; // one-sided macro domain; no abs(mu) extension
        return mu/PositiveGaussianMoment(mu,Sigma(w));
    }
    double G1(V w,V m) const {
        if (m.y<=0 || w.Dot(m)<=0) return 0;
        return Factor(w);
    }
};

void VerifySphere(const std::string& label,const Candidate& model) {
    const OceanSlopeNormalDistribution distribution(model.cov,
        Vector3f(float(-model.p),1,float(-model.q)));
    const V tangent=V{1,model.p,0}.Unit();
    const std::array<V,9> directions{{
        {0,1,0},model.n,V{.2,1,.3}.Unit(),V{-1,.2,.3}.Unit(),
        V{.7,.6,-.4}.Unit(),(tangent+model.n*.001).Unit(),
        {-1,0,0},{1,0,0},{0,-1,0}
    }};
    std::array<double,9> area{},visible{},signedArea{},vndfMass{};
    constexpr int nt=1024,np=512;
    const double dt=.5*pi/nt,dp=2*pi/np;
    for (int i=0;i<nt;++i) {
        const double theta=(i+.5)*dt,st=std::sin(theta),ct=std::cos(theta);
        for (int j=0;j<np;++j) {
            const double phi=(j+.5)*dp;
            const V m{st*std::cos(phi),ct,st*std::sin(phi)};
            const double dA=distribution.AreaDensityWorld(Vector3f(float(m.x),float(m.y),float(m.z)))*st*dt*dp;
            for (std::size_t k=0;k<directions.size();++k) {
                const V w=directions[k];
                const double cosine=w.Dot(m),positive=std::max(0.0,cosine);
                area[k]+=dA*positive;
                signedArea[k]+=dA*cosine;
                const double weighted=dA*positive*model.G1(w,m);
                visible[k]+=weighted;
                if (model.Mu(w)>0) vndfMass[k]+=weighted/(model.n.y*model.Mu(w));
            }
        }
    }
    bool areaOk=true,closureOk=true,signedOk=true,probabilityOk=true,vndfOk=true;
    double maxAreaError=0,maxClosureError=0;
    for (std::size_t k=0;k<directions.size();++k) {
        const V w=directions[k];
        const double analytic=model.Area(w),target=model.n.y*model.Mu(w);
        areaOk=areaOk && Near(area[k],analytic);
        signedOk=signedOk && Near(signedArea[k],target);
        maxAreaError=std::max(maxAreaError,std::abs(area[k]-analytic));
        if (target>0) {
            closureOk=closureOk && Near(visible[k],target,2e-8,4e-5);
            vndfOk=vndfOk && Near(vndfMass[k],1,1e-6,4e-5);
            maxClosureError=std::max(maxClosureError,std::abs(visible[k]-target));
        } else closureOk=closureOk && visible[k]==0;
        const double factor=model.Factor(w);
        probabilityOk=probabilityOk && std::isfinite(factor) && factor>=0 && factor<=1+1e-14;
    }
    Expect("ocean G1 "+label+" analytic A+ matches independent sphere integral",areaOk);
    Expect("ocean G1 "+label+" signed projected area identity",signedOk);
    Expect("ocean G1 "+label+" visible projected area closure",closureOk);
    Expect("ocean G1 "+label+" visible-normal density integrates to one",vndfOk);
    Expect("ocean G1 "+label+" factors stay in [0,1]",probabilityOk);
    std::cout << "[INFO] G1 " << label << " max_abs_A_error=" << maxAreaError
        << " max_abs_closure_error=" << maxClosureError << '\n';
}
}

void RunOceanSlopeMaskingAcceptanceTests() {
    bool momentsOk=true;
    for (double sigma:{.05,.5,2.0})
        for (double a:{-4.0,-1.0,0.0,.2,1.0,4.0,12.0}) {
            const double analytic=PositiveGaussianMoment(a*sigma,sigma);
            momentsOk=momentsOk && Near(analytic,NumericalMoment(a*sigma,sigma),1e-9,1e-7);
        }
    Expect("ocean G1 Gaussian positive moment matches independent 1D integral",momentsOk);
    Expect("ocean G1 zero directional variance has deterministic limit",
        PositiveGaussianMoment(2,0)==2 && PositiveGaussianMoment(-2,0)==0 && PositiveGaussianMoment(0,0)==0);
    const Candidate flat({.04,.01,.006},0,0);
    const Candidate tilted({.64,.25,.12},1,.25);
    VerifySphere("flat anisotropic",flat);
    VerifySphere("tilted correlated",tilted);
    VerifySphere("tilted negative correlation",Candidate({.36,.16,-.1},-.5,.75));

    const V up{0,1,0};
    Expect("ocean G1 world-up factor is exactly one",flat.Factor(up)==1 && tilted.Factor(up)==1);
    Expect("ocean G1 world-up projected area equals macroY",Near(tilted.Area(up),tilted.n.y,1e-14,0));
    Expect("ocean G1 world-up sees macro-lower micro-normal",
        tilted.n.Dot(V{3,1,0}.Unit())<0 && tilted.G1(up,V{3,1,0}.Unit())==1);
    const V w=V{-1,.2,0}.Unit();
    Expect("ocean G1 rejects view-backfacing micro-normal",tilted.G1(w,V{1,.1,0}.Unit())==0);
    Expect("ocean G1 macro-backside is not extended by absolute cosine",tilted.Factor({1,0,0})==0);

    // Flat case independently agrees with the Gaussian/Beckmann Smith formula.
    bool beckmannOk=true;
    for (double degrees:{30.0,60.0,80.0,89.0}) {
        const double angle=degrees*pi/180;
        const V direction{std::sin(angle),std::cos(angle),0};
        const double a=std::cos(angle)/(std::sqrt(2.0)*.2*std::sin(angle));
        const double lambda=.5*(std::erf(a)-1)+std::exp(-a*a)/(2*a*std::sqrt(pi));
        beckmannOk=beckmannOk && Near(flat.Factor(direction),1/(1+lambda),1e-13,1e-13);
    }
    Expect("ocean G1 flat limit agrees with Beckmann Smith expression",beckmannOk);

    const V tangent=V{1,tilted.p,0}.Unit();
    bool grazingOk=true;
    double previous=1;
    for (double eps:{1e-1,1e-2,1e-3,1e-4,1e-6,1e-8}) {
        const V direction=(tangent+tilted.n*eps).Unit();
        const double g=tilted.Factor(direction);
        grazingOk=grazingOk && std::isfinite(g) && g>0 && g<previous;
        previous=g;
        if (eps==1e-8) {
            const double asymptotic=std::sqrt(2*pi)*tilted.Mu(direction)/tilted.Sigma(direction);
            grazingOk=grazingOk && Near(g/asymptotic,1,1e-6,0);
        }
        std::cout << "[INFO] G1 macro_grazing epsilon=" << eps << " factor=" << g << '\n';
    }
    Expect("ocean G1 macro-grazing tends to zero with correct asymptote",grazingOk);
    Expect("ocean G1 exact macro tangent is zero",tilted.Factor(tangent)==0);
    const double horizon=tilted.Factor({-1,0,0});
    Expect("ocean G1 world horizon need not be macro grazing",horizon>.9 && horizon<1);
    std::cout << "[INFO] G1 tilted world_horizon=" << horizon << " world_up=" << tilted.Factor(up) << '\n';
}
