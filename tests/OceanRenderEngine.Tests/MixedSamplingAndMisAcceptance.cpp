#include "reflection/layered/CoatedDiffuseReflection.h"
#include "reflection/bxdfs/BSDF.h"
#include "reflection/bxdfs/ThinDielectricBxDF.h"
#include "reflection/materials/DielectricMaterial.h"
#include "render/integrators/PathIntegrator.h"
#include "render/integrators/SimplePathIntegrator.h"
#include "render/lights/UniformInfiniteLight.h"
#include "core/ray/HittableList.h"
#include "geometry/primitives/Sphere.h"
#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <numbers>
#include <string>
#include <vector>

namespace {
using namespace LayeredReflectionDetail;
constexpr double pi=std::numbers::pi_v<double>;
constexpr int muBins=4,phiBins=8,sideBins=muBins*phiBins;
using Bins=std::array<double,2*sideBins>;
void Check(const std::string& name,bool ok) {
    if(!ok){std::cerr<<"[FAIL] mixed sampling MIS "<<name<<std::endl;std::exit(EXIT_FAILURE);}
    std::cout<<"[PASS] mixed sampling MIS "<<name<<std::endl;
}
double Relative(double a,double b){return std::abs(a-b)/std::max({std::abs(a),std::abs(b),1e-30});}
int Bin(Vector3f v) {
    double phi=std::atan2(double(v.y),double(v.x));if(phi<0)phi+=2*pi;
    return (v.z<0?sideBins:0)+std::clamp(int(std::abs(v.z)*muBins),0,muBins-1)*phiBins+
        std::clamp(int(phi/(2*pi)*phiBins),0,phiBins-1);
}
struct Integral {Bins bins{};double energy=0;};
Integral Integrate(const DielectricBxDF& bxdf,Vector3f wo,int nm,int np) {
    Integral r;const double dw=2*pi/(double(nm)*np);
    for(float sign:{-1.f,1.f})for(int i=0;i<nm;++i)for(int j=0;j<np;++j) {
        const double mu=(i+.5)/nm,phi=2*pi*(j+.5)/np,s=std::sqrt(1-mu*mu);
        Vector3f wi(float(s*std::cos(phi)),float(s*std::sin(phi)),sign*float(mu));
        r.bins[Bin(wi)]+=bxdf.Pdf(wo,wi,TransportMode::Radiance,BxDFReflectionType::All)*dw;
        r.energy+=bxdf.f(wo,wi,TransportMode::Radiance).c[0]*mu*dw;
    }
    return r;
}
void MixedCase(const char* name,Vector3f wo,std::uint64_t seed) {
    const DielectricBxDF bxdf(1.5f,TrowbridgeReitzDistribution(.6f,.85f));
    const auto coarse=Integrate(bxdf,wo,128,256),fine=Integrate(bxdf,wo,256,512);
    constexpr std::size_t N=131072;RandomStream rng(seed);
    Bins counts{};std::size_t accepted=0,transmitted=0;bool valid=true,modes=true;
    double maxPdf=0,maxF=0,sum=0,sum2=0,l1=0,mass=0,maxZ=0;
    for(std::size_t i=0;i<N;++i) {
        const float uc=rng.Next();const auto u=rng.Next2D();
        const auto s=bxdf.Sample_f(wo,uc,u,TransportMode::Radiance,BxDFReflectionType::All);
        if(i<4096) {
            const auto im=bxdf.Sample_f(wo,uc,u,TransportMode::Importance,BxDFReflectionType::All);
            modes=modes&&(bool(s)==bool(im));
            if(s&&im) {
                const double factor=(s->flags&BSDF_TRANSMISSION)?std::pow(wo.z>0?1.5:1/1.5,2):1;
                modes=modes&&s->pdf==im->pdf&&s->wi.x==im->wi.x&&s->wi.y==im->wi.y&&s->wi.z==im->wi.z&&
                    Relative(s->f.c[0]*factor,im->f.c[0])<1e-6;
            }
        }
        if(!s)continue;
        ++accepted;if(s->flags&BSDF_TRANSMISSION)++transmitted;
        const double p=bxdf.Pdf(wo,s->wi,TransportMode::Radiance,BxDFReflectionType::All);
        const double f=bxdf.f(wo,s->wi,TransportMode::Radiance).c[0];
        const double l2=double(s->wi.x)*s->wi.x+double(s->wi.y)*s->wi.y+double(s->wi.z)*s->wi.z;
        valid=valid&&s->pdf>0&&std::isfinite(s->pdf)&&std::isfinite(s->f.c[0])&&s->f.c[0]>=0&&
            std::abs(l2-1)<2e-7&&((s->flags&BSDF_REFLECTION)!=0)==(wo.z*s->wi.z>0);
        maxPdf=std::max(maxPdf,Relative(p,s->pdf));maxF=std::max(maxF,Relative(f,s->f.c[0]));
        counts[Bin(s->wi)]+=1.0/N;
        const double weight=s->f.c[0]*std::abs(s->wi.z)/s->pdf;
        sum+=weight;sum2+=weight*weight;
    }
    for(std::size_t k=0;k<counts.size();++k) {
        l1+=std::abs(coarse.bins[k]-fine.bins[k]);mass+=fine.bins[k];
        const double p=fine.bins[k],sigma=std::sqrt(std::max(0.,p*(1-p))/N);
        maxZ=std::max(maxZ,std::abs(counts[k]-p)/(sigma+.00015+1.0/N));
    }
    const double observed=double(accepted)/N,sigma=std::sqrt(std::max(0.,mass*(1-mass))/N);
    const double mean=sum/N,se=std::sqrt(std::max(0.,sum2/N-mean*mean)/N);
    std::cout<<std::setprecision(10)<<"[INFO] mixed sampling MIS "<<name<<" seed="<<seed<<" accepted="<<accepted
        <<" transmission="<<transmitted<<" integral="<<mass<<" binL1="<<l1<<" maxZ="<<maxZ
        <<" pdfRel="<<maxPdf<<" fRel="<<maxF<<" energy="<<mean<<" oracle="<<fine.energy<<" se="<<se<<std::endl;
    const std::string n(name);
    Check(n+" independent full-sphere quadrature converges",l1<.004&&Relative(coarse.energy,fine.energy)<.005);
    Check(n+" returned samples and flags valid",valid&&accepted>0);
    Check(n+" sample and query PDF agree",maxPdf<5e-5);
    Check(n+" sample and query f agree",maxF<5e-5);
    Check(n+" null mass agrees with independent PDF",std::abs(observed-mass)<6*sigma+.00015*counts.size()+2.0/N);
    Check(n+" full angular histogram agrees",maxZ<6);
    Check(n+" throughput agrees with independent f integral",std::abs(mean-fine.energy)<6*se+std::abs(coarse.energy-fine.energy)+.0005);
    Check(n+" both events have support",transmitted>100&&accepted-transmitted>100);
    Check(n+" transport modes retain direction PDF and eta",modes);
}
// Synthetic interface oracles, not physical material replacements.
struct CosineTop {
    double t=.8;
    bool IsDelta()const{return false;}
    double f(Vector3f wo,Vector3f wi,TransportMode)const{return wo.z>0&&wi.z<0?t/pi:0;}
    double Pdf(Vector3f wo,Vector3f wi,TransportMode,BxDFReflectionType flags)const{
        return flags==BxDFReflectionType::Transmission&&wo.z>0&&wi.z<0?std::abs(wi.z)/pi:0;
    }
    std::optional<LayeredInterfaceSample> Sample_f(Vector3f wo,Point2f u,TransportMode,BxDFReflectionType flags)const {
        if(flags!=BxDFReflectionType::Transmission||wo.z<=0)return {};
        Vector3f wi=BRDFUtils::SampleCosineHemisphere(u);wi.z=-wi.z;
        return LayeredInterfaceSample{t/pi,wi,std::abs(wi.z)/pi};
    }
};
struct UniformTop : CosineTop {
    bool half=false;
    double Pdf(Vector3f wo,Vector3f wi,TransportMode,BxDFReflectionType flags)const {
        return flags==BxDFReflectionType::Transmission&&wo.z>0&&wi.z<0&&(!half||wi.y>=0)?1/(2*pi):0;
    }
    double f(Vector3f wo,Vector3f wi,TransportMode mode)const {
        return Pdf(wo,wi,mode,BxDFReflectionType::Transmission)>0?t/pi:0;
    }
    std::optional<LayeredInterfaceSample> Sample_f(Vector3f wo,Point2f u,TransportMode,BxDFReflectionType flags)const {
        if(flags!=BxDFReflectionType::Transmission||wo.z<=0||(half&&u.y>=.5f))return {};
        const double mu=u.x,phi=2*pi*u.y,r=std::sqrt(1-mu*mu);
        return LayeredInterfaceSample{t/pi,{float(r*std::cos(phi)),float(r*std::sin(phi)),-float(mu)},1/(2*pi)};
    }
};
struct PeakedBase {
    double rho=.5,n=20;
    bool IsDelta()const{return false;}
    double f(Vector3f wo,Vector3f wi,TransportMode)const{
        return wo.z>0&&wi.z>0?rho*(n+2)/(2*pi)*std::pow(double(wi.z),n):0;
    }
    double Pdf(Vector3f wo,Vector3f wi,TransportMode,BxDFReflectionType flags)const{
        return flags==BxDFReflectionType::Reflection&&wo.z>0&&wi.z>0?(n+2)/(2*pi)*std::pow(double(wi.z),n+1):0;
    }
    std::optional<LayeredInterfaceSample> Sample_f(Vector3f wo,Point2f u,TransportMode mode,BxDFReflectionType flags)const{
        if(flags!=BxDFReflectionType::Reflection||wo.z<=0)return {};
        const double mu=std::pow(double(u.x),1/(n+2)),phi=2*pi*u.y,r=std::sqrt(1-mu*mu);
        const Vector3f wi(float(r*std::cos(phi)),float(r*std::sin(phi)),float(mu));
        return LayeredInterfaceSample{f(wo,wi,mode),wi,Pdf(wo,wi,mode,flags)};
    }
};
struct MirrorBase {
    bool IsDelta()const{return true;}
    double f(Vector3f,Vector3f,TransportMode)const{return 0;}
    double Pdf(Vector3f,Vector3f,TransportMode,BxDFReflectionType)const{return 0;}
    std::optional<LayeredInterfaceSample> Sample_f(Vector3f wo,Point2f,TransportMode,BxDFReflectionType flags)const{
        if(flags!=BxDFReflectionType::Reflection||wo.z<=0)return {};
        return LayeredInterfaceSample{.5/std::abs(wo.z),{-wo.x,-wo.y,wo.z},1};
    }
};
struct ComponentProbeSampler : Sampler {
    int one=0,two=0;
    float Get1D() override {++one;return 0;}
    Point2f Get2D() override {++two;return {.3f,.7f};}
};
struct Stats {
    double mean=0,variance=0;
    explicit Stats(const std::vector<double>& x) {
        for(double v:x)mean+=v;mean/=x.size();
        for(double v:x)variance+=(v-mean)*(v-mean);variance/=(x.size()-1);
    }
};
template<class Top,class Bottom>
void Compare(const char* name,const Top& top,const Bottom& bottom,Vector3f wo,Vector3f wi,
    double opticalThickness,std::size_t cap,std::size_t samples,int seeds,double reference=0,bool requireReduction=false,bool rr=false)
{
    std::vector<double> oldValues,newValues;double seconds[2]{};
    for(int mode=0;mode<2;++mode) {
        const auto start=std::chrono::steady_clock::now();
        for(int seed=0;seed<seeds;++seed) {
            LayeredReflectionOptions o;o.useMIS=mode!=0;o.seed=42+seed;o.samplesPerEvaluation=samples;
            o.maxInternalReflections=cap;o.russianRoulette=rr;
            const auto e=EvaluateScalarLayeredReflectionMonteCarlo(top,bottom,wo,wi,opticalThickness,o);
            (mode?newValues:oldValues).push_back(e.internalReflection);
        }
        seconds[mode]=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    }
    const Stats a(oldValues),b(newValues);
    const double combinedSE=std::sqrt((a.variance+b.variance)/seeds);
    std::cout<<"[INFO] mixed sampling MIS "<<name<<" seeds="<<seeds<<" samples="<<samples<<" cap="<<cap
        <<" oldMean="<<a.mean<<" misMean="<<b.mean<<" oldVar="<<a.variance<<" misVar="<<b.variance
        <<" varianceRatio="<<b.variance/a.variance<<" oldSeconds="<<seconds[0]<<" misSeconds="<<seconds[1]<<std::endl;
    Check(std::string(name)+" multi seed mean consistent",std::abs(a.mean-b.mean)<6*combinedSE+1e-7);
    if(reference>0) {
        Check(std::string(name)+" exit only agrees with independent oracle",std::abs(a.mean-reference)<6*std::sqrt(a.variance/seeds)+1e-7);
        Check(std::string(name)+" MIS agrees with independent oracle",std::abs(b.mean-reference)<6*std::sqrt(b.variance/seeds)+1e-7);
    }
    if(requireReduction)Check(std::string(name)+" variance reduced for this case",b.variance<a.variance*.65);
}
}
void RunMixedSamplingAndMisAcceptanceTests()
{
    const Vector3f n(0,0,1),inside=MicrofacetReflectionDetail::NormalizeDirection({.8f,.1f,-.6f});
    const auto rad=TransportMode::Radiance,imp=TransportMode::Importance;
    const auto all=BxDFReflectionType::All,refl=BxDFReflectionType::Reflection,trans=BxDFReflectionType::Transmission;
    const DielectricBxDF rough(1.5f,TrowbridgeReitzDistribution(.6f,.85f));
    bool rejected=false;try{rough.Sample_f(n,{.3f,.7f},rad,all);}catch(const std::invalid_argument&){rejected=true;}
    Check("old 2D rough All cannot silently reuse a VNDF dimension",rejected);
    const BxDF& dynamic=rough;
    const auto viaBase=dynamic.Sample_f(n,.99f,{.3f,.7f},rad,all);
    Check("independent event overload dispatches through BxDF",viaBase&&bool(viaBase->flags&BSDF_TRANSMISSION));
    const BSDF bsdf(n,{1,0,0},std::make_shared<DielectricBxDF>(rough));
    const auto world=bsdf.Sample_f(n,.99f,{.3f,.7f},rad,all);
    Check("BSDF forwards independent event sample",world&&world->wi.z<0&&bsdf.NeedsComponentSample());
    const ThinDielectricBxDF thin(1.5f);
    const auto thinR=thin.Sample_f(n,0.f,{.99f,.3f},rad,all);
    const DiffuseBxDF diffuse(Spectrum(.5f));
    const auto d=diffuse.Sample_f(n,.7f,{.3f,.7f},rad,all);
    Check("single lobe direct 1D plus 2D overload stays available",d&&d->wi.z>0);
    for(bool full:{false,true}) {
        HittableList scene;
        scene.add(std::make_shared<Sphere>(Vector3f(0,0,-1),.5f,
            std::make_shared<DielectricMaterial>(1.5f,0,0,false)));
        const std::vector<std::shared_ptr<Light>> lights{std::make_shared<UniformInfiniteLight>(Spectrum(1.f))};
        auto sampler=std::make_shared<ComponentProbeSampler>();
        Camera camera(8,8);Spectrum L;
        if(full) L=PathIntegrator(1,scene,camera,sampler,lights).Li(Ray({0,0,0},{0,0,-1}),*sampler);
        else L=SimplePathIntegrator(1,false,true,scene,camera,sampler,lights).Li(Ray({0,0,0},{0,0,-1}),*sampler);
        Check(full?"PathIntegrator forwards independent event":"SimplePathIntegrator forwards independent event",
            sampler->one==1&&sampler->two==1&&Relative(L.c[0],1.)<1e-6);
    }
    Check("thin dielectric event uses uc not direction x",thinR&&bool(thinR->flags&BSDF_REFLECTION));
    const DielectricBxDF smooth(1.5f,TrowbridgeReitzDistribution(0,0));
    const auto sr=smooth.Sample_f(n,0.f,{.9f,.7f},rad,all),st=smooth.Sample_f(n,.5f,{.01f,.7f},rad,all);
    Check("smooth All discrete masses preserved",sr&&st&&Relative(sr->pdf,.04)<1e-6&&Relative(st->pdf,.96)<1e-6);
    Check("smooth All continuous PDF remains zero",sr&&st&&smooth.Pdf(n,sr->wi,rad,all)==0&&smooth.Pdf(n,st->wi,rad,all)==0);
    const auto tir=smooth.Sample_f(inside,.99f,{.5f,.7f},rad,all);
    Check("actual smooth TIR All always reflects",tir&&tir->pdf==1&&bool(tir->flags&BSDF_REFLECTION));
    MixedCase("All outside",MicrofacetReflectionDetail::NormalizeDirection({.5f,.2f,.8f}),42);
    MixedCase("All inside macro TIR",inside,43);
    MixedCase("All grazing",MicrofacetReflectionDetail::NormalizeDirection({1,0,.08f}),44);
    // A single fixed facet: changing only uc must switch events at its Fresnel.
    const Point2f u(.31760507822036743f,.95412570238113403f);
    const auto m=DielectricDetail::Promote(TrowbridgeReitzDistribution(.6f,.85f).Sample_Wm(inside,u));
    const auto optics=DielectricDetail::EvaluateInterface(DielectricDetail::Dot(DielectricDetail::Promote(inside),m),1,1.5);
    const auto a=rough.Sample_f(inside,float(optics.R*.5),u,rad,all);
    const auto b=rough.Sample_f(inside,float(optics.R+optics.T*.5),u,rad,all);
    const auto forced=rough.Sample_f(inside,.5f,u,rad,trans);
    const auto forcedR=rough.Sample_f(inside,.5f,u,rad,refl);
    Check("fixed facet selects reflection with the same Fresnel used in PDF",
        a&&forcedR&&bool(a->flags&BSDF_REFLECTION)&&Relative(a->pdf,forcedR->pdf*optics.R/(optics.R+optics.T))<1e-6);
    Check("fixed facet mixed transmission is not rejected by macro TIR",b&&forced&&b->wi.x==forced->wi.x&&b->wi.z==forced->wi.z);
    Check("mixed and conditional transmission densities differ by facet probability",
        b&&forced&&Relative(b->pdf,forced->pdf*optics.T/(optics.R+optics.T))<1e-6);
    Check("mixed and conditional transmission f identical",b&&forced&&b->f.c[0]==forced->f.c[0]);
    bool sumWeights=true;
    for(double x:{0.,1e-300,.1,1.,1e300})for(double y:{0.,1e-300,.2,1.,1e300})
        if(x+y>0)sumWeights=sumWeights&&std::abs(ConnectionWeight(x,y)+ConnectionWeight(y,x)-1)<3e-16;
    Check("power weights complementary and overflow safe",sumWeights);
    Check("power weights correct one proposal zero support",ConnectionWeight(.5,0)==1&&ConnectionWeight(0,.5)==0&&ConnectionWeight(0,0)==0);
    LayeredReflectionOptions o;o.samplesPerEvaluation=256;o.maxInternalReflections=0;o.russianRoulette=false;
    Check("exit only remains default after measured coating variance comparison",!o.useMIS);
    o.useMIS=true;
    const CoatedDiffuseReflection publicMIS(1,1.5,.4f,.6f,0,o);
    Check("public coated model forwards optional MIS",publicMIS.Options().useMIS&&std::isfinite(publicMIS.Evaluate(n,n,.5)));
    const DiffuseInterface base(.5);const CosineTop cosine;
    const double oracle=.5*.8*.8/pi;
    const auto exact=EvaluateScalarLayeredReflectionMonteCarlo(cosine,base,n,n,0,o);
    Check("equal proposals exact TRT no double count or extra half",Relative(exact.internalReflection,oracle)<2e-6);
    Check("final permitted base visit includes both proposals",exact.pathsAtReflectionLimit==o.samplesPerEvaluation);
    o.useMIS=false;
    Check("exit only exact TRT control preserved",Relative(EvaluateScalarLayeredReflectionMonteCarlo(cosine,base,n,n,0,o).internalReflection,oracle)<2e-6);
    o.useMIS=true;
    const auto mirror=EvaluateScalarLayeredReflectionMonteCarlo(cosine,MirrorBase{},n,n,0,o);
    Check("Delta base uses base proposal without continuous competition",Relative(mirror.internalReflection,oracle)<2e-6);
    const DielectricInterface deltaTop(1.5f,TrowbridgeReitzDistribution(0,0));
    const auto delta=EvaluateScalarLayeredReflectionMonteCarlo(deltaTop,base,n,n,0,o);
    Check("Delta top only exit proposal retains eta and mass",Relative(delta.internalReflection,.5/pi*.96*.96/2.25)<2e-6);
    bool bothDelta=false;try{EvaluateScalarLayeredReflectionMonteCarlo(deltaTop,MirrorBase{},n,n,0,o);}
    catch(const std::invalid_argument&){bothDelta=true;}
    Check("two Delta layer is not misreported as finite BRDF",bothDelta);
    UniformTop half;half.half=true;bool exitRejectedButBaseWorks=false;
    o.samplesPerEvaluation=1;
    for(int seed=0;seed<100;++seed) {
        o.seed=seed;o.useMIS=false;const auto e=EvaluateScalarLayeredReflectionMonteCarlo(half,base,n,n,0,o);
        o.useMIS=true;const auto e2=EvaluateScalarLayeredReflectionMonteCarlo(half,base,n,n,0,o);
        if(e.internalReflection==0&&e2.internalReflection>0){exitRejectedButBaseWorks=true;break;}
    }
    Check("rejected exit does not gate independent base contribution",exitRejectedButBaseWorks);
    Compare("half supported exit",half,base,n,n,0,0,4096,24,oracle*.25);
    Compare("peaked base variance",UniformTop{},PeakedBase{},n,n,0,0,512,32,oracle,true);
    const DielectricInterface top(1.5f,TrowbridgeReitzDistribution(.4f,.6f));
    Compare("rough TRT",top,base,n,n,0,0,4096,16);
    Compare("rough multibounce absorbing",top,base,
        MicrofacetReflectionDetail::NormalizeDirection({.5f,.1f,.8f}),
        MicrofacetReflectionDetail::NormalizeDirection({-.2f,.3f,.9f}),.2,4,2048,16);
    Compare("rough multibounce RR",top,base,
        MicrofacetReflectionDetail::NormalizeDirection({.5f,.1f,.8f}),
        MicrofacetReflectionDetail::NormalizeDirection({-.2f,.3f,.9f}),.2,8,2048,16,0,false,true);
}
