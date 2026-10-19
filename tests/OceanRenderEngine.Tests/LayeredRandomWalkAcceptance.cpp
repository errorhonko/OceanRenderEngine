#include "reflection/materials/CoatedDiffuseMaterial.h" // Public header self-contained.
#include "render/integrators/PathIntegrator.h"
#include "render/lights/UniformInfiniteLight.h"
#include "core/sampling/IndependentSampler.h"
#include "core/ray/HittableList.h"
#include "geometry/primitives/Sphere.h"
#include <array>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

namespace {
using namespace LayeredReflectionDetail;
constexpr double pi=std::numbers::pi_v<double>;
void Check(const std::string& name,bool ok) {
    if(!ok){std::cerr<<"[FAIL] layered random walk "<<name<<std::endl;std::exit(EXIT_FAILURE);}
    std::cout<<"[PASS] layered random walk "<<name<<std::endl;
}
template<class E,class F>bool Rejects(F f){try{f();}catch(const E&){return true;}return false;}
double Rel(double a,double b){return std::abs(a-b)/std::max({std::abs(a),std::abs(b),1e-30});}
struct Moments {
    double sum=0,sum2=0;std::size_t n=0,accepted=0,specular=0,survivors=0;
    bool valid=true;std::array<std::size_t,8> muCounts{};
    void Add(std::optional<LayeredPathSample> s) {
        ++n;if(!s)return;++accepted;
        const double l2=double(s->wi.x)*s->wi.x+double(s->wi.y)*s->wi.y+double(s->wi.z)*s->wi.z;
        valid=valid&&s->wi.z>0&&std::abs(l2-1)<3e-7&&s->f>0&&s->pdf>0&&
            std::isfinite(s->f)&&std::isfinite(s->pdf)&&s->pdfIsProportional&&
            (s->flags&BSDF_REFLECTION)&&!(s->flags&BSDF_TRANSMISSION);
        ++muCounts[std::clamp(int(s->wi.z*8),0,7)];
        specular+=bool(s->flags&BSDF_SPECULAR);survivors+=s->rouletteSurvivals;
        const double w=s->f*std::abs(double(s->wi.z))/s->pdf;
        valid=valid&&std::isfinite(w)&&w>=0;sum+=w;sum2+=w*w;
    }
    double Mean()const{return sum/n;}
    double SE()const{return std::sqrt(std::max(0.,sum2/n-Mean()*Mean())/n);}
};
template<class T,class B>
Moments Measure(const T& top,const B& bottom,Vector3f wo,double tau,
    LayeredReflectionOptions o,std::size_t n,std::uint64_t seed=42) {
    Moments m;RandomStream rng(seed);
    for(std::size_t i=0;i<n;++i){const float uc=rng.Next();const auto u=rng.Next2D();
        m.Add(SampleScalarLayeredReflection(top,bottom,wo,uc,u,tau,o));}
    return m;
}
void Oracle(const char* name,const Moments& m,double expected,double allowance=1e-6) {
    std::cout<<std::setprecision(11)<<"[INFO] layered random walk "<<name<<" mean="<<m.Mean()
        <<" oracle="<<expected<<" se="<<m.SE()<<" accepted="<<m.accepted<<"/"<<m.n
        <<" delta="<<m.specular<<" rrSurvivals="<<m.survivors<<std::endl;
    Check(std::string(name)+" valid proportional samples",m.valid&&m.accepted>0);
    Check(std::string(name)+" independent expected transport",std::abs(m.Mean()-expected)<6*m.SE()+allowance);
}
double InsideFresnel(int n) {
    double sum=0;for(int i=0;i<n;++i){const double mu=(i+.5)/n;sum+=2*mu*FrDielectric(-mu,1.,1.5);}
    return sum/n;
}
double AbsorbingAlbedo(double rho,double tau,double muO,int n) {
    double sum=0;for(int i=0;i<n;++i){const double mu=(i+.5)/n;sum+=2*mu*std::exp(-tau/mu);}
    return rho*std::exp(-tau/muO)*sum/n;
}
double RoughAlbedo(const DielectricInterface& top,const DiffuseInterface& base,
    LayeredReflectionOptions o,int nm,int np) {
    double sum=0;const Vector3f wo(0,0,1);
    for(int i=0;i<nm;++i)for(int j=0;j<np;++j) {
        const double mu=(i+.5)/nm,phi=2*pi*(j+.5)/np,s=std::sqrt(1-mu*mu);
        const Vector3f wi(float(s*std::cos(phi)),float(s*std::sin(phi)),float(mu));
        sum+=EvaluateScalarLayeredReflectionMonteCarlo(top,base,wo,wi,0,o).Brdf()*mu;
    }
    return sum*2*pi/(double(nm)*np);
}
std::array<double,8> FirstTrtPdfBins(const DielectricInterface& top,const DiffuseInterface& base,
    LayeredReflectionOptions o,int nm,int np) {
    std::array<double,8> bins{};const Vector3f wo(0,0,1);
    for(int i=0;i<nm;++i)for(int j=0;j<np;++j) {
        const double mu=(i+.5)/nm,phi=2*pi*(j+.5)/np,s=std::sqrt(1-mu*mu);
        const Vector3f wi(float(s*std::cos(phi)),float(s*std::sin(phi)),float(mu));
        const double proxy=EstimateScalarLayeredPdfForMIS(top,base,wo,wi,o);
        // Undo ONLY the documented tail mix to test its direct-R + TRT term.
        bins[i/(nm/8)]+=(proxy-.1/(2*pi))/.9*2*pi/(double(nm)*np);
    }
    return bins;
}
// Synthetic NORMAL-DIRECTION discrete kernel for an independent RR series,
// not a claimed dielectric/reciprocal material.
struct BranchTop {
    bool IsDelta()const{return true;}
    std::optional<LayeredInterfaceSample> SampleAll(Vector3f wo,float uc,Point2f,TransportMode)const {
        if(wo.z>0)return LayeredInterfaceSample{1,{0,0,-1},1};
        return LayeredInterfaceSample{.5,{0,0,uc<.5f?-1.f:1.f},.5};
    }
};
struct MirrorBase {
    double rho=.6;
    bool IsDelta()const{return true;}
    std::optional<LayeredInterfaceSample> Sample_f(Vector3f,Point2f,TransportMode,BxDFReflectionType)const {
        return LayeredInterfaceSample{rho,{0,0,1},1};
    }
};
struct LongTop {
    mutable std::size_t calls=0;
    bool IsDelta()const{return true;}
    std::optional<LayeredInterfaceSample> SampleAll(Vector3f wo,float,Point2f,TransportMode)const {
        if(wo.z>0)return LayeredInterfaceSample{1,{0,0,-1},1};
        return LayeredInterfaceSample{1e-30,{0,0,++calls<=50?-1.f:1.f},1e-30};
    }
};
struct ScaledBase {
    bool IsDelta()const{return true;}
    std::optional<LayeredInterfaceSample> Sample_f(Vector3f,Point2f,TransportMode,BxDFReflectionType)const {
        return LayeredInterfaceSample{1e-30,{0,0,1},1e-30};
    }
};
}
void RunLayeredRandomWalkAcceptanceTests()
{
    const Vector3f n(0,0,1),oblique=MicrofacetReflectionDetail::NormalizeDirection({.4f,.2f,1});
    const auto rad=TransportMode::Radiance,imp=TransportMode::Importance;
    const auto all=BxDFReflectionType::All,refl=BxDFReflectionType::Reflection,trans=BxDFReflectionType::Transmission;
    LayeredReflectionOptions o;o.samplesPerEvaluation=256;o.pdfSamples=16;
    o.maxInternalReflections=0;o.russianRoulette=false;
    Check("independent PDF sample budget default",LayeredReflectionOptions{}.pdfSamples==32);
    const LayeredReflectionOptions legacy{LayeredEvaluationMethod::MonteCarlo,64,32,42,true,false};
    Check("old aggregate option order preserved",legacy.pdfSamples==32&&!legacy.useMIS);
    for(std::size_t count:{std::size_t(0),std::size_t(1048577)}) {
        auto bad=o;bad.pdfSamples=count;
        Check("invalid PDF sample budget rejected",Rejects<std::invalid_argument>([&]{bad.Validate();}));
    }
    const DiffuseInterface base(.5);
    const DielectricInterface matched(1.f,TrowbridgeReitzDistribution(.2f,.3f));
    const DielectricInterface smooth(1.5f,TrowbridgeReitzDistribution(0,0));
    const DielectricInterface rough(1.5f,TrowbridgeReitzDistribution(.4f,.6f));
    Oracle("matched index Lambert",Measure(matched,base,n,0,o,65536),.5,2e-6);
    const double absorb=AbsorbingAlbedo(.5,.2,double(oblique.z),32768);
    Check("absorbing independent quadrature converges",Rel(absorb,AbsorbingAlbedo(.5,.2,double(oblique.z),65536))<1e-7);
    Oracle("matched absorbing",Measure(matched,base,oblique,.2,o,131072),absorb,2e-6);
    // Exact cosine-direction CDF for the matched-index random walk, not Pdf()'s proxy.
    RandomStream rng(43);std::array<std::size_t,8> bins{};bool invariant=true;
    for(int i=0;i<65536;++i) {
        const float uc=rng.Next();const auto u=rng.Next2D();
        const auto s=SampleScalarLayeredReflection(matched,base,n,uc,u,0,o);
        invariant=invariant&&s&&s->internalReflections==0&&Rel(s->f*std::abs(s->wi.z)/s->pdf,.5)<2e-6;
        if(s)++bins[std::clamp(int(s->wi.z*8),0,7)];
    }
    bool histogram=true;double maxZ=0;
    for(int i=0;i<8;++i) {
        const double p=double(2*i+1)/64,sigma=std::sqrt(p*(1-p)/65536);
        maxZ=std::max(maxZ,std::abs(double(bins[i])/65536-p)/(sigma+1.0/65536));
    }
    histogram=maxZ<6;
    Check("matched sampled throughput constant",invariant);
    Check("matched true direction law cosine histogram",histogram);
    const double favg=InsideFresnel(65536);
    Check("smooth internal Fresnel quadrature converges",Rel(favg,InsideFresnel(32768))<1e-5);
    double firstAccepted=0;
    for(std::size_t cap:{std::size_t(0),std::size_t(4),std::size_t(16)}) {
        auto controls=o;controls.maxInternalReflections=cap;
        const double expected=.04+.96*.5*(1-favg)*(1-std::pow(.5*favg,double(cap+1)))/(1-.5*favg);
        const auto m=Measure(smooth,base,n,0,controls,131072,42+cap);
        Oracle(("smooth cap "+std::to_string(cap)).c_str(),m,expected,3e-5);
        Check("smooth direct reflection is a discrete event",std::abs(double(m.specular)/m.n-.04)<.004);
        if(cap==0)firstAccepted=double(m.accepted)/m.n;
    }
    Check("smooth rejection retains null mass at TRT cap",firstAccepted>.1&&firstAccepted<.9);
    const DiffuseInterface black(0);
    const auto dark=Measure(smooth,black,n,0,o,65536);
    Oracle("black base leaves direct R only",dark,.04,1e-6);
    Check("black base all accepted paths direct Delta",dark.accepted==dark.specular);
    const double nan=std::numeric_limits<double>::quiet_NaN();
    const CoatedDiffuseReflection model(1,1.5,.4f,.6f,0,o);
    for(float x:{-1.f,1.f,float(nan)}) {
        Check("invalid event variate rejected",Rejects<std::invalid_argument>([&]{model.Sample_f(n,x,{.3f,.7f},.5);}));
        Check("invalid VNDF variate rejected",Rejects<std::invalid_argument>([&]{model.Sample_f(n,.5f,{x,.7f},.5);}));
    }
    Check("opaque layer transmission and unset return no sample",
        !model.Sample_f(n,.5f,{.3f,.7f},.5,rad,trans)&&!model.Sample_f(n,.5f,{.3f,.7f},.5,rad,BxDFReflectionType::Unset));
    Check("opaque layer transmission PDF zero",model.Pdf(n,n,.5,rad,trans)==0);
    Check("backside sample and PDF zero",!model.Sample_f(-n,.5f,{.3f,.7f},.5)&&model.Pdf(n,-n,.5)==0);
    Check("zero direction rejected",Rejects<std::invalid_argument>([&]{model.Sample_f({0,0,0},.5f,{.3f,.7f},.5);}));
    Check("unknown flags rejected",Rejects<std::invalid_argument>([&]{model.Sample_f(n,.5f,{.3f,.7f},.5,rad,static_cast<BxDFReflectionType>(8));}));
    Check("invalid base reflectance rejected",Rejects<std::invalid_argument>([&]{model.Pdf(n,n,-1);}));
    const auto direct=model.Sample_f(n,0.f,{0,0},0);
    const auto native=rough.SampleAll(n,0.f,{0,0},rad);
    Check("direct R path flagged proportional even before entering",
        direct&&native&&direct->pdfIsProportional&&Rel(direct->f/direct->pdf,native->f/native->pdf)<1e-7);
    const auto sampled=model.Sample_f(n,.8f,{.2f,.7f},.5);
    const auto repeat=model.Sample_f(n,.8f,{.2f,.7f},.5);
    Check("walk deterministic without mutable RNG state",bool(sampled)==bool(repeat)&&(!sampled||
        (sampled->wi.x==repeat->wi.x&&sampled->f==repeat->f&&sampled->pdf==repeat->pdf)));
    auto changed=o;changed.pdfSamples=128;changed.useMIS=true;
    const auto unrelated=CoatedDiffuseReflection(1,1.5,.4f,.6f,0,changed).Sample_f(n,.8f,{.2f,.7f},.5);
    Check("walk independent of f MIS and PDF budgets",bool(sampled)==bool(unrelated)&&(!sampled||
        (sampled->wi.x==unrelated->wi.x&&sampled->f==unrelated->f&&sampled->pdf==unrelated->pdf)));
    rng=RandomStream(44);bool modes=true,flags=true;std::size_t valid=0;
    for(int i=0;i<8192;++i) {
        const float uc=rng.Next();const auto u=rng.Next2D();
        const auto r=model.Sample_f(oblique,uc,u,.5,rad,all),im=model.Sample_f(oblique,uc,u,.5,imp,all);
        const auto rf=model.Sample_f(oblique,uc,u,.5,rad,refl);
        modes=modes&&(bool(r)==bool(im));flags=flags&&(bool(r)==bool(rf));
        if(r&&im&&rf){++valid;modes=modes&&r->wi.x==im->wi.x&&r->wi.z==im->wi.z&&
            Rel(r->f/r->pdf,im->f/im->pdf)<2e-5;
            flags=flags&&r->wi.x==rf->wi.x&&r->f==rf->f&&r->pdf==rf->pdf;}
    }
    Check("same-medium exterior transport modes agree without RR",modes&&valid>100);
    Check("exterior Reflection includes all hidden R T routes",flags&&valid>100);
    // PDF(): independently seeded MIS proxy, not the returned path-product pdf.
    const double floor=.1/(2*pi);
    const double p=model.Pdf(n,oblique,.5),p2=model.Pdf(n,oblique,.5);
    Check("MIS density finite positive and repeatable",std::isfinite(p)&&p>=floor&&p==p2);
    Check("MIS query mode independent",p==model.Pdf(n,oblique,.5,imp));
    Check("PDF queries do not change subsequent walk",bool(sampled)==bool(model.Sample_f(n,.8f,{.2f,.7f},.5)));
    const double matchedExpected=.9*double(oblique.z)/pi+floor;
    const double matchedPdf=EstimateScalarLayeredPdfForMIS(matched,base,n,oblique,o);
    Check("matched PDF proxy analytic control",Rel(matchedPdf,matchedExpected)<2e-6);
    const double smoothExpected=.9*.96*(1-FrDielectric(double(oblique.z),1.,1.5))*double(oblique.z)/(pi*2.25)+floor;
    Check("smooth TRT density includes inverse Snell solid angle Jacobian",
        Rel(EstimateScalarLayeredPdfForMIS(smooth,base,n,oblique,o),smoothExpected)<3e-6);
    const double blackExpected=.9*rough.PdfAll(n,oblique,rad)+floor;
    Check("black base proxy omits TRT but retains documented uniform floor",Rel(model.Pdf(n,oblique,0),blackExpected)<1e-12);
    // RR kernel analytic truncated series. Both use the SAME permitted orders.
    for(bool rr:{false,true}) {
        auto controls=o;controls.maxInternalReflections=12;controls.russianRoulette=rr;
        const auto m=Measure(BranchTop{},MirrorBase{},n,0,controls,262144,rr?45:46);
        const double expected=.6*.5*(1-std::pow(.6*.5,13))/(1-.6*.5);
        Oracle(rr?"RR compensated discrete series":"no RR discrete series",m,expected,1e-6);
        Check(rr?"RR surviving path denominator compensated":"RR switch really off",rr?m.survivors>100:m.survivors==0);
    }
    auto longOptions=o;longOptions.maxInternalReflections=50;longOptions.russianRoulette=true;
    const auto longPath=SampleScalarLayeredReflection(LongTop{},ScaledBase{},n,.5f,{.3f,.7f},0,longOptions);
    Check("common pair rescaling preserves a long path",longPath&&longPath->internalReflections==50&&
        longPath->pairRescalings>20&&Rel(longPath->f/longPath->pdf,1)<1e-12);
    longOptions.maxInternalReflections=49;
    Check("hard cap differs from compensated roulette",!SampleScalarLayeredReflection(LongTop{},ScaledBase{},n,.5f,{.3f,.7f},0,longOptions));
    // Independent direction-space integration of the EXISTING f estimator.
    auto integralOptions=o;integralOptions.samplesPerEvaluation=128;
    const double a1=RoughAlbedo(rough,base,integralOptions,24,48);
    integralOptions.samplesPerEvaluation=256;
    const double a2=RoughAlbedo(rough,base,integralOptions,48,96);
    const auto transport=Measure(rough,base,n,0,o,131072,47);
    Check("rough independent f integral stable",std::abs(a1-a2)<.01);
    Oracle("rough TRT integrated f",transport,a2,std::abs(a1-a2)+.001);
    std::cout<<"[INFO] layered random walk rough albedo coarse="<<a1<<" fine="<<a2<<std::endl;

    const auto qc=FirstTrtPdfBins(rough,base,o,32,64),qf=FirstTrtPdfBins(rough,base,o,64,128);
    double margin=0,maxBinZ=0,pdfL1=0;
    for(int i=0;i<8;++i) {
        const double prob=qf[i],difference=std::abs(qc[i]-qf[i]);
        margin+=prob;pdfL1+=difference;
        const double sigma=std::sqrt(std::max(0.,prob*(1-prob))/transport.n);
        maxBinZ=std::max(maxBinZ,std::abs(double(transport.muCounts[i])/transport.n-prob)/(sigma+difference+.003));
    }
    std::cout<<"[INFO] layered random walk first TRT density mass="<<margin<<" observed="
        <<double(transport.accepted)/transport.n<<" binL1="<<pdfL1<<" maxBinZ="<<maxBinZ<<std::endl;
    Check("first TRT PDF estimator quadrature converges",pdfL1<.03);
    Check("direct plus TRT term agrees with actual random walk bins",maxBinZ<6&&
        std::abs(margin-double(transport.accepted)/transport.n)<.03);
    auto multi=o;multi.maxInternalReflections=4;multi.samplesPerEvaluation=128;
    const double multiCoarse=RoughAlbedo(rough,base,multi,24,48);
    multi.samplesPerEvaluation=256;
    const double multiFine=RoughAlbedo(rough,base,multi,48,96);
    multi.russianRoulette=true;
    const auto multiWalk=Measure(rough,base,n,0,multi,131072,48);
    Check("rough multibounce independent f integral stable",std::abs(multiCoarse-multiFine)<.01);
    Oracle("rough multibounce RR integrated f",multiWalk,multiFine,std::abs(multiCoarse-multiFine)+.001);
    Check("real rough walk actually exercises surviving RR",multiWalk.survivors>0);
    // Ordinary BSDF/Material adapter is explicitly neutral, not a NIR->RGB cast.
    const CoatedDiffuseBxDF bxdf(1,1,.2f,.3f,.4,0,o);
    const auto s=bxdf.Sample_f(n,.7f,{.3f,.7f});
    Check("adapter returns proportional neutral sample",s&&s->pdfIsProportional&&s->eta==1&&
        s->f.c[0]==s->f.c[1]&&s->f.c[1]==s->f.c[2]&&Rel(s->f.c[0]*s->wi.z/s->pdf,.4)<2e-6);
    Check("2D adapter entry cannot invent hidden event sample",Rejects<std::invalid_argument>([&]{bxdf.Sample_f(n,{.3f,.7f});}));
    const CoatedDiffuseBxDF absorbed(1,1,.2f,.3f,.4,100,o);
    Check("float adapter safely terminates numerically absorbed paths",!absorbed.Sample_f(n,.7f,{.3f,.7f}));
    Check("2D unsupported exterior transmission remains zero",!bxdf.Sample_f(n,{.3f,.7f},rad,trans));
    const CoatedDiffuseMaterial material(1,1,.2f,.3f,.4,0,o);
    const auto bsdf=material.GetBSDF({{0,0,0},n,n,{1,0,0},0,0});
    Check("reflection only material still requests independent component variate",
        bsdf.NeedsComponentSample()&&!(bsdf.Flags()&BSDF_TRANSMISSION));
    const auto world=bsdf.Sample_f(n,.7f,{.3f,.7f});
    Check("BSDF preserves proportional flag through world frame",world&&world->pdfIsProportional&&Rel(world->f.c[0]*world->wi.z/world->pdf,.4)<2e-6);
    Point2f rhoSamples[]={{.2f,.3f},{.4f,.6f}};
    Check("legacy rho uses direction integration not a fake event RNG",Rel(bxdf.rho(n,2,rhoSamples).c[0],.4)<2e-6);
    // Actual PathIntegrator exercises NEE, approximate PDF query, and escape MIS.
    HittableList scene;scene.add(std::make_shared<Sphere>(Vector3f(0,0,-1),.5f,
        std::make_shared<CoatedDiffuseMaterial>(material)));
    const std::vector<std::shared_ptr<Light>> lights{std::make_shared<UniformInfiniteLight>(Spectrum(1.f))};
    auto sampler=std::make_shared<IndependentSampler>(42);
    PathIntegrator integrator(1,scene,Camera(8,8),sampler,lights);
    double sum=0,sum2=0;constexpr int count=4096;
    for(int i=0;i<count;++i){const double v=integrator.Li(Ray({0,0,0},{0,0,-1}),*sampler).c[0];sum+=v;sum2+=v*v;}
    const double mean=sum/count,se=std::sqrt(std::max(0.,sum2/count-mean*mean)/count);
    std::cout<<"[INFO] layered random walk PathIntegrator environment mean="<<mean<<" oracle=.4 se="<<se<<std::endl;
    Check("full path environment MIS retains matched Lambert mean",std::abs(mean-.4)<6*se+.002);
    HittableList roughScene;roughScene.add(std::make_shared<Sphere>(Vector3f(0,0,-1),.5f,
        std::make_shared<CoatedDiffuseMaterial>(1,1.5,.4f,.6f,.5,0,o)));
    auto roughSampler=std::make_shared<IndependentSampler>(43);
    PathIntegrator roughIntegrator(1,roughScene,Camera(8,8),roughSampler,lights);
    double roughSum=0,roughSum2=0;
    for(int i=0;i<count;++i){const double v=roughIntegrator.Li(Ray({0,0,0},{0,0,-1}),*roughSampler).c[0];
        roughSum+=v;roughSum2+=v*v;}
    const double roughMean=roughSum/count;
    const double roughSE=std::sqrt(std::max(0.,roughSum2/count-roughMean*roughMean)/count);
    std::cout<<"[INFO] layered random walk rough PathIntegrator mean="<<roughMean<<" oracle="<<a2<<" se="<<roughSE<<std::endl;
    Check("full rough path MIS agrees with independent f integral",
        std::abs(roughMean-a2)<6*roughSE+std::abs(a1-a2)+.002);
}
