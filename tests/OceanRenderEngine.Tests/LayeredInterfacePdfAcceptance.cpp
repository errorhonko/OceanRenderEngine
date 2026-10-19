#include "reflection/layered/CoatedDiffuseReflection.h" // Self-contained interface header.
#include <array>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numbers>
#include <string>

namespace {
using namespace LayeredReflectionDetail;
constexpr double pi=std::numbers::pi_v<double>;
constexpr int muBins=4,phiBins=8,binCount=muBins*phiBins;
using Bins=std::array<double,binCount>;
void Check(const std::string& name,bool ok)
{
    if(!ok){std::cerr<<"[FAIL] layered conditional PDF "<<name<<std::endl;std::exit(EXIT_FAILURE);}
    std::cout<<"[PASS] layered conditional PDF "<<name<<std::endl;
}
template<class E,class F>bool Rejects(F f){try{f();}catch(const E&){return true;}return false;}
double RelativeError(double a,double b)
{return std::abs(a-b)/std::max({std::abs(a),std::abs(b),1e-30});}
int Bin(Vector3f w)
{
    double phi=std::atan2(double(w.y),double(w.x));if(phi<0)phi+=2*pi;
    const int i=std::clamp(int(std::abs(double(w.z))*muBins),0,muBins-1);
    const int j=std::clamp(int(phi/(2*pi)*phiBins),0,phiBins-1);
    return i*phiBins+j;
}
struct SamplingResult
{
    Bins probability{};
    std::size_t accepted=0;
    double maxPdfError=0,maxFError=0;
    double maxRegularFError=0,maxCriticalFAbsoluteError=0;
    std::size_t nearCriticalSamples=0;
    double sampleFAtMaxError=0,queryFAtMaxError=0;
    Vector3f wiAtMaxFError;
    bool valid=true;
};
template<class Interface>
SamplingResult Measure(const Interface& interface,Vector3f wo,BxDFReflectionType flags,
    std::size_t n,std::uint64_t seed=42,float relativeEta=1.5f)
{
    SamplingResult result;RandomStream rng(seed);
    for(std::size_t i=0;i<n;++i)
    {
        const auto s=interface.Sample_f(wo,rng.Next2D(),TransportMode::Radiance,flags);
        if(!s)continue;
        ++result.accepted;
        const double length2=double(s->wi.x)*s->wi.x+double(s->wi.y)*s->wi.y+double(s->wi.z)*s->wi.z;
        const bool reflected=wo.z*s->wi.z>0;
        result.valid=result.valid&&std::isfinite(s->pdf)&&s->pdf>0&&std::isfinite(s->f)&&s->f>=0&&
            std::abs(length2-1)<1e-3&&s->wi.z!=0&&(reflected==(flags==BxDFReflectionType::Reflection));
        const double p=interface.Pdf(wo,s->wi,TransportMode::Radiance,flags);
        const double f=interface.f(wo,s->wi,TransportMode::Radiance);
        result.maxPdfError=std::max(result.maxPdfError,RelativeError(p,s->pdf));
        const double fError=RelativeError(f,s->f);
        // Classify the critical subset for reporting, but assert f/PDF
        // consistency on ALL accepted samples, including this subset.
        bool nearCritical=false;
        if(flags==BxDFReflectionType::Transmission && wo.z<0)
        {
            const auto wm=MicrofacetReflectionDetail::NormalizeDirection(wo+s->wi*(1.f/relativeEta));
            nearCritical=std::abs(s->wi.dot(wm))<.02f;
        }
        if(nearCritical)
        {
            ++result.nearCriticalSamples;
            result.maxCriticalFAbsoluteError=std::max(result.maxCriticalFAbsoluteError,std::abs(f-s->f));
        }
        else result.maxRegularFError=std::max(result.maxRegularFError,fError);
        if(fError>result.maxFError)
        {
            result.maxFError=fError;result.sampleFAtMaxError=s->f;result.queryFAtMaxError=f;
            result.wiAtMaxFError=s->wi;
        }
        result.probability[Bin(s->wi)]+=1.0/double(n); // Original N, NOT accepted count.
    }
    return result;
}
template<class Interface>
Bins Integrate(const Interface& interface,Vector3f wo,BxDFReflectionType flags,int nMu,int nPhi)
{
    Bins result{};
    const float sign=(wo.z>0)==(flags==BxDFReflectionType::Reflection)?1.f:-1.f;
    for(int i=0;i<nMu;++i)for(int j=0;j<nPhi;++j)
    {
        const double mu=(i+.5)/nMu,phi=2*pi*(j+.5)/nPhi,s=std::sqrt(1-mu*mu);
        const Vector3f wi(float(s*std::cos(phi)),float(s*std::sin(phi)),sign*float(mu));
        const int bin=(i/(nMu/muBins))*phiBins+j/(nPhi/phiBins);
        // Direct independent direction-space midpoint quadrature; no cosine:
        // Pdf is already a density per solid angle, dOmega=dMu*dPhi.
        result[bin]+=interface.Pdf(wo,wi,TransportMode::Radiance,flags)*2*pi/(double(nMu)*nPhi);
    }
    return result;
}
double Mass(const Bins& bins){double sum=0;for(double p:bins)sum+=p;return sum;}
double L1(const Bins& a,const Bins& b){double sum=0;for(int i=0;i<binCount;++i)sum+=std::abs(a[i]-b[i]);return sum;}
void CheckHistogram(const std::string& name,const SamplingResult& sampled,const Bins& expected,std::size_t n,double quadratureAllowance=0)
{
    double largestZ=0;
    bool finite=true;
    for(int i=0;i<binCount;++i)
    {
        const double p=expected[i];finite=finite&&std::isfinite(p)&&p>=0&&p<=1;
        const double sigma=std::sqrt(std::max(0.,p*(1-p))/double(n));
        largestZ=std::max(largestZ,std::abs(sampled.probability[i]-p)/(sigma+quadratureAllowance+1./n));
    }
    const double mass=Mass(expected),observed=double(sampled.accepted)/n;
    const double sigma=std::sqrt(std::max(0.,mass*(1-mass))/double(n));
    std::cout<<std::setprecision(9)<<"[INFO] layered conditional PDF "<<name<<" accepted="<<observed<<
        " integral="<<mass<<" null="<<1-observed<<" maxBinZ="<<largestZ<<
        " maxPdfRel="<<sampled.maxPdfError<<" maxFRel="<<sampled.maxFError<<
        " regularFRel="<<sampled.maxRegularFError<<" nearCritical="<<sampled.nearCriticalSamples<<
        " criticalFAbs="<<sampled.maxCriticalFAbsoluteError<<std::endl;
    if(sampled.maxFError>=.005)
        std::cout<<"[INFO] layered conditional PDF worst f: sample="<<sampled.sampleFAtMaxError<<
            " query="<<sampled.queryFAtMaxError<<" wi="<<sampled.wiAtMaxFError.x<<','<<
            sampled.wiAtMaxFError.y<<','<<sampled.wiAtMaxFError.z<<std::endl;
    Check(name+" queried probabilities finite",finite&&mass<=1.005);
    Check(name+" returned samples valid",sampled.valid);
    Check(name+" sampled PDF agrees with query",sampled.maxPdfError<5e-5);
    Check(name+" sampled f agrees including critical angle",sampled.maxFError<5e-5);
    // Six-sigma acceptance with explicit quadrature error, not renormalization.
    Check(name+" mass agrees with acceptance plus null event",std::abs(mass-observed)<6*sigma+quadratureAllowance*binCount+2./n);
    Check(name+" angular histogram agrees with PDF",largestZ<6);
}
}

void RunLayeredInterfacePdfAcceptanceTests()
{
    const Vector3f n(0,0,1),down(0,0,-1),tangent(1,0,0);
    const auto radiance=TransportMode::Radiance,importance=TransportMode::Importance;
    const auto reflection=BxDFReflectionType::Reflection,transmission=BxDFReflectionType::Transmission;
    const DielectricInterface rough(1.5f,TrowbridgeReitzDistribution(.6f,.85f));
    const DiffuseInterface diffuse(.4),black(0);
    Check("rough and diffuse continuous",!rough.IsDelta()&&!diffuse.IsDelta());
    for(auto bad:{BxDFReflectionType::All,BxDFReflectionType::Unset,static_cast<BxDFReflectionType>(8)})
    {
        Check("dielectric rejects mixed or invalid query flags",Rejects<std::invalid_argument>([&]{rough.Pdf(n,n,radiance,bad);}));
        Check("dielectric rejects mixed or invalid sample flags",Rejects<std::invalid_argument>([&]{rough.Sample_f(n,{.3f,.7f},radiance,bad);}));
        Check("diffuse rejects mixed or invalid query flags",Rejects<std::invalid_argument>([&]{diffuse.Pdf(n,n,radiance,bad);}));
        Check("diffuse rejects mixed or invalid sample flags",Rejects<std::invalid_argument>([&]{diffuse.Sample_f(n,{.3f,.7f},radiance,bad);}));
    }
    for(Vector3f invalid:{Vector3f(0,0,0),Vector3f(0,0,2),Vector3f(std::numeric_limits<float>::quiet_NaN(),0,1)})
    {
        Check("reject invalid dielectric query directions",Rejects<std::invalid_argument>([&]{rough.Pdf(n,invalid,radiance,reflection);}));
        Check("reject invalid diffuse query directions",Rejects<std::invalid_argument>([&]{diffuse.Pdf(invalid,n,radiance,reflection);}));
    }
    Check("dielectric horizon query zero",rough.Pdf(n,tangent,radiance,reflection)==0&&rough.Pdf(tangent,n,radiance,reflection)==0);
    Check("diffuse horizon sample rejected",!diffuse.Sample_f(tangent,{.3f,.7f},radiance,reflection));
    Check("dielectric horizon sample rejected",!rough.Sample_f(tangent,{.3f,.7f},radiance,reflection));
    Check("diffuse rejects transmission",diffuse.Pdf(n,down,radiance,transmission)==0&&!diffuse.Sample_f(n,{.3f,.7f},radiance,transmission));
    Check("wrong reflection hemisphere density zero",rough.Pdf(n,down,radiance,reflection)==0&&diffuse.Pdf(n,down,radiance,reflection)==0);
    Check("wrong transmission hemisphere density zero",rough.Pdf(n,n,radiance,transmission)==0);
    Check("diffuse analytic normal PDF",std::abs(diffuse.Pdf(n,n,radiance,reflection)-1/pi)<1e-7);
    Check("black base has same directional PDF",black.Pdf(n,n,radiance,reflection)==diffuse.Pdf(n,n,radiance,reflection));
    // Rough microfacets can transmit even when the MACRO normal predicts TIR.
    // Reject only when the actual sampled microfacet has TIR.
    const Vector3f inside=MicrofacetReflectionDetail::NormalizeDirection({.8f,.1f,-.6f});
    Check("chosen inside direction has macro TIR",FrDielectric(inside.z,1.f,1.5f)==1);
    const auto macroTir=Measure(rough,inside,transmission,8192);
    Check("rough transmission survives macro TIR",macroTir.accepted>100);

    // Local mixed precision: regression on the former worst critical sample.
    const Point2f criticalU(.31760507822036743f,.95412570238113403f);
    const auto critical=rough.Sample_f(inside,criticalU,radiance,transmission);
    Check("critical sample remains accepted",bool(critical));
    // Decimal 120-digit Cartesian GGX/Smith reference for the normalized NEW
    // stored wo/wi pair (not the old float direction): see experiment report.
    constexpr double criticalReference=.00018532416562358273364048767855383253;
    Check("critical query agrees with independent reference",critical&&RelativeError(
        rough.f(inside,critical->wi,radiance),criticalReference)<1e-6);
    Check("critical sample f consistency",critical&&RelativeError(critical->f,rough.f(inside,critical->wi,radiance))<1e-5);
    Check("critical sample PDF consistency",critical&&RelativeError(critical->pdf,rough.Pdf(inside,critical->wi,radiance,transmission))<1e-5);
    const auto promoted=DielectricDetail::Promote(inside);
    Check("promoted direction normalized in double",std::abs(DielectricDetail::Dot(promoted,promoted)-1)<1e-15);
    const auto optics=DielectricDetail::EvaluateInterface(-.8,1,1.5);
    Check("inside reciprocal index retained in double",optics.eta==2.0/3.0&&optics.eta!=double(1.f/1.5f));
    Check("lossless Fresnel power sum",std::abs(optics.R+optics.T-1)<2e-15);
    Check("normal power transmission",std::abs(DielectricDetail::EvaluateInterface(1,1,1.5).T-.96)<1e-15);
    const auto tir=DielectricDetail::EvaluateInterface(-.5,1,1.5);
    Check("shared TIR branch",tir.tir&&tir.R==1&&tir.T==0);
    const auto matchedOptics=DielectricDetail::EvaluateInterface(0,1,1);
    Check("matched grazing transmits",!matchedOptics.tir&&matchedOptics.R==0&&matchedOptics.T==1);
    const double nearC=std::sqrt(1-4.0/9.0*(1-1e-12));
    const auto near=DielectricDetail::EvaluateInterface(-nearC,1,1.5);
    Check("near critical positive transmission",!near.tir&&near.T>0&&near.T<1e-4&&std::abs(near.R+near.T-1)<2e-15);
    Vector3f bent;float bentEta=0;
    Check("public Refract inside survives",Refract(Vector3f(.6f,0,-.8f),n,1.5f,&bentEta,&bent));
    const auto bentD=DielectricDetail::Promote(bent);
    Check("public Refract Snell law",std::abs(bentD.x+.9)<1e-7&&bentEta==float(2.0/3.0));
    Check("public Refract unit output",std::abs(double(bent.x)*bent.x+double(bent.y)*bent.y+double(bent.z)*bent.z-1)<1e-7);
    Check("public Refract TIR rejects",!Refract(inside,n,1.5f,nullptr,&bent));
    Check("invalid zero Refract rejected",!Refract(Vector3f(0,0,0),n,1.5f,nullptr,&bent));
    // Delta continuous PDF and discrete sampling mass must not be compared.
    for(const auto& top:{DielectricInterface(1.5f,TrowbridgeReitzDistribution(0,0)),
                        DielectricInterface(1.f,TrowbridgeReitzDistribution(.3f,.4f))})
    {
        Check("smooth or matched interface marked Delta",top.IsDelta());
        const auto s=top.Sample_f(n,{.3f,.7f},radiance,transmission);
        Check("Delta transmission has discrete unit mass",s&&s->pdf==1&&s->wi.z<0);
        Check("Delta transmission has zero continuous density",s&&top.Pdf(n,s->wi,radiance,transmission)==0);
    }
    const DielectricInterface smooth(1.5f,TrowbridgeReitzDistribution(0,0));
    Check("smooth actual TIR rejects transmission",!smooth.Sample_f(inside,{.3f,.7f},radiance,transmission));
    const DielectricInterface matched(1.f,TrowbridgeReitzDistribution(.3f,.4f));
    Check("matched interface has no reflection event",!matched.Sample_f(n,{.3f,.7f},radiance,reflection)&&matched.Pdf(n,n,radiance,reflection)==0);
    // Lambert hemisphere density has an independent exact bin integral.
    constexpr std::size_t count=131072;
    Bins lambert{};
    for(int i=0;i<muBins;++i)for(int j=0;j<phiBins;++j)
        lambert[i*phiBins+j]=(double((i+1)*(i+1))-i*i)/(muBins*muBins*phiBins);
    for(Vector3f wo:{n,down})
        CheckHistogram(wo.z>0?"Lambert upper":"Lambert lower",Measure(diffuse,wo,reflection,count),lambert,count);
    CheckHistogram("black Lambert still samples directions",Measure(black,n,reflection,count),lambert,count);
    // Normal isotropic GGX reflection has exact bin masses and null probability.
    constexpr float alpha=.6f;
    const DielectricInterface isotropic(1.5f,TrowbridgeReitzDistribution(alpha,alpha));
    Bins ggx{};
    auto tail=[](double mu){const double s2=(1-mu)/(1+mu);return s2/(double(alpha)*alpha+s2);};
    for(int i=0;i<muBins;++i)for(int j=0;j<phiBins;++j)
        ggx[i*phiBins+j]=(tail(double(i)/muBins)-tail(double(i+1)/muBins))/phiBins;
    const auto ggxSamples=Measure(isotropic,n,reflection,count);
    CheckHistogram("GGX exact normal reflection",ggxSamples,ggx,count);
    Check("GGX rejected samples carry nonzero null mass",ggxSamples.accepted<count&&std::abs(Mass(ggx)-1/(1+double(alpha)*alpha))<1e-12);
    // Anisotropic, oblique outside/inside and grazing cases. Quadrature doubles
    // both angular axes; predicted bins MUST converge before comparison.
    struct Case{const char* name;Vector3f wo;BxDFReflectionType flags;};
    const Vector3f outside=MicrofacetReflectionDetail::NormalizeDirection({.5f,.2f,.8f});
    const Case cases[]={{"oblique reflection",outside,reflection},{"oblique transmission",outside,transmission},
        {"inside reflection",inside,reflection},{"inside macro TIR transmission",inside,transmission},
        {"grazing reflection",MicrofacetReflectionDetail::NormalizeDirection({1,0,.08f}),reflection}};
    for(const auto& c:cases)
    {
        const auto coarse=Integrate(rough,c.wo,c.flags,128,256),fine=Integrate(rough,c.wo,c.flags,256,512);
        const double error=L1(coarse,fine);
        std::cout<<"[INFO] layered conditional PDF "<<c.name<<" coarseMass="<<Mass(coarse)<<" fineMass="<<Mass(fine)<<" binL1="<<error<<std::endl;
        Check(std::string(c.name)+" independent quadrature converges",error<.004);
        CheckHistogram(c.name,Measure(rough,c.wo,c.flags,count),fine,count,.00015);
        // Transport mode changes transmission f, not the conditional density.
        RandomStream rng(43);bool same=true,etaCorrect=true;std::size_t valid=0;
        for(int i=0;i<8192;++i)
        {
            const auto u=rng.Next2D();const auto r=rough.Sample_f(c.wo,u,radiance,c.flags),im=rough.Sample_f(c.wo,u,importance,c.flags);
            same=same&&(bool(r)==bool(im));if(!r||!im)continue;++valid;
            same=same&&r->pdf==im->pdf&&r->wi.x==im->wi.x&&r->wi.y==im->wi.y&&r->wi.z==im->wi.z&&
                rough.Pdf(c.wo,r->wi,radiance,c.flags)==rough.Pdf(c.wo,r->wi,importance,c.flags);
            const double eta=c.wo.z>0?double(1.5f):1.0/double(1.5f);
            const double factor=c.flags==transmission?eta*eta:1.;
            etaCorrect=etaCorrect&&RelativeError(r->f*factor,im->f)<1e-6;
        }
        Check(std::string(c.name)+" mode independent sampled directions and PDF",same&&valid>0);
        Check(std::string(c.name)+" transport eta factor preserved",etaCorrect&&valid>0);
    }
}
