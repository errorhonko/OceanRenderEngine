#include "reflection/lidar/CoatedDiffuseLidarScattering.h" // Self-contained public header.
#include "reflection/lidar/SurfaceLidarScattering.h"
#include "scene/import/ObjSceneAsset.h"
#include "lidar/simulation/LidarIntegrator.h"
#include "lidar/simulation/LidarWaveform.h"
#include "core/sampling/IndependentSampler.h"
#include <iostream>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <numbers>
#include <type_traits>

namespace {
constexpr double pi = std::numbers::pi_v<double>;
void Check(const char* name, bool ok)
{
    if (!ok) { std::cerr << "[FAIL] coated diffuse " << name << '\n'; std::exit(EXIT_FAILURE); }
    std::cout << "[PASS] coated diffuse " << name << '\n';
}
bool Near(double a, double b, double relative=1e-6)
{
    const bool ok=std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=1e-14+relative*std::max(std::abs(a),std::abs(b));
    if(!ok)std::cerr<<std::setprecision(16)<<"[INFO] coated diffuse comparison actual="<<a<<" expected="<<b<<" relativeTolerance="<<relative<<'\n';
    return ok;
}
template<class E,class F> bool Rejects(F f) { try { f(); } catch(const E&) { return true; } return false; }
// Smooth coating oracle: independent hemispherical Fresnel quadrature, then
// sum the Lambert-base geometric series. Delta top reflection is excluded.
double InternalFresnel(int n)
{
    double sum=0;
    for(int i=0;i<n;++i) { const double mu=(i+.5)/n; sum+=2*mu*FrDielectric(-mu,1.,1.5); }
    return sum/n;
}
double TransmissionIntegral(const LayeredReflectionDetail::DielectricInterface& top,
    Vector3f outside,TransportMode mode,int nMu,int nPhi)
{
    double sum=0;
    for(int i=0;i<nMu;++i)for(int j=0;j<nPhi;++j)
    {
        const double mu=(i+.5)/nMu,phi=2*pi*(j+.5)/nPhi,s=std::sqrt(1-mu*mu);
        const Vector3f inside(float(s*std::cos(phi)),float(s*std::sin(phi)),-float(mu));
        sum+=top.f(outside,inside,mode)*mu;
    }
    return sum*2*pi/(double(nMu)*nPhi);
}
struct RejectingTop
{
    bool IsDelta() const {return false;}
    double Pdf(const Vector3f&,const Vector3f&,TransportMode,BxDFReflectionType) const {return 0;}
    mutable std::size_t calls=0;
    double f(const Vector3f&,const Vector3f&,TransportMode)const {return .125;}
    std::optional<LayeredInterfaceSample> Sample_f(const Vector3f&,Point2f,TransportMode,BxDFReflectionType)const
    {++calls;return {};}
};
// Synthetic scalar mock: constant f=1/pi over the downward hemisphere, sampled
// uniformly (pdf=1/(2*pi)). Reject half the azimuth without renormalizing pdf.
// Its hemispherical throughput is 1/2, so two independent connections give 1/4.
struct HalfTransmissionTop
{
    bool IsDelta() const {return false;}
    double Pdf(const Vector3f& wo,const Vector3f& wi,TransportMode,BxDFReflectionType flags)const
    {return flags==BxDFReflectionType::Transmission&&wo.z>0&&wi.z<0&&wi.y>=0?1/(2*pi):0;}
    double f(const Vector3f& wo,const Vector3f& wi,TransportMode mode)const
    {return Pdf(wo,wi,mode,BxDFReflectionType::Transmission)>0?1/pi:0;}
    std::optional<LayeredInterfaceSample> Sample_f(const Vector3f&,Point2f u,TransportMode,BxDFReflectionType flags)const
    {
        if(flags!=BxDFReflectionType::Transmission || u.y>=.5f)return {};
        const double mu=u.x,phi=2*pi*u.y,s=std::sqrt(1-mu*mu);
        return LayeredInterfaceSample{1/pi,{float(s*std::cos(phi)),float(s*std::sin(phi)),-float(mu)},1/(2*pi)};
    }
};
LidarPulseResult Pulse(const Hittable& world,const LidarScatteringModel& model,std::size_t count,float wavelength)
{
    const Vector3f sensor(-.5f,15,.5f),down(0,-1,0);
    const LaserEmitter emitter(sensor,down,wavelength,1e-3f,0);
    const LidarReceiver receiver(sensor,down,.25f,.01f,.8f);
    const LidarIntegrator integrator(world,emitter,receiver,count,model);
    IndependentSampler sampler(42);return integrator.SimulatePulse(sampler);
}
}

void RunCoatedDiffuseLidarAcceptanceTests()
{
    static_assert(std::is_base_of_v<LidarScatteringModel,CoatedDiffuseLidarScattering>);
    LayeredReflectionOptions options;
    Check("defaults to Monte Carlo",options.method==LayeredEvaluationMethod::MonteCarlo);
    Check("default sample count depth seed and RR explicit",options.samplesPerEvaluation==64&&options.maxInternalReflections==32&&options.seed==42&&options.russianRoulette);
    options.samplesPerEvaluation=256;
    const Vector3f n(0,0,1),a(.3f,.1f,1),b(-.2f,.4f,1);
    const CoatedDiffuseReflection model(1,1.5,.2f,.3f,0,options);
    const auto estimate=model.Estimate(a,b,.5);
    Check("positive finite direct and internal parts",estimate.directReflection>0&&estimate.internalReflection>0&&std::isfinite(estimate.Brdf()));
    Check("repeat queries reproducible",estimate.Brdf()==model.Evaluate(a,b,.5));
    Check("canonical finite estimate reciprocal by construction",estimate.Brdf()==model.Evaluate(b,a,.5));
    auto changed=options;changed.seed=43;
    Check("explicit seed changes internal estimate",estimate.internalReflection!=CoatedDiffuseReflection(1,1.5,.2f,.3f,0,changed).Estimate(a,b,.5).internalReflection);
    Check("unrelated evaluation does not mutate RNG state",(model.Evaluate(n,n,.2),model.Evaluate(a,b,.5))==estimate.Brdf());
    changed=options;changed.method=LayeredEvaluationMethod::AnalyticApproximation;
    Check("analytic selection explicitly unavailable no fallback",Rejects<std::logic_error>([&]{CoatedDiffuseReflection c(1,1.5,.2f,.3f,0,changed);}));
    changed.method=static_cast<LayeredEvaluationMethod>(99);
    Check("reject unknown backend",Rejects<std::invalid_argument>([&]{changed.Validate();}));
    for(std::size_t count:{std::size_t(0),std::size_t(1048577)})
    { changed=options;changed.samplesPerEvaluation=count;Check("reject invalid sample count",Rejects<std::invalid_argument>([&]{changed.Validate();})); }
    changed=options;changed.maxInternalReflections=4097;
    Check("reject excessive internal depth",Rejects<std::invalid_argument>([&]{changed.Validate();}));
    const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
    for(double x:{0.,-1.,nan,inf})
    {
        Check("reject invalid incident index",Rejects<std::invalid_argument>([&]{CoatedDiffuseReflection c(x,1.5,.2f,.3f);}));
        Check("reject invalid coat index",Rejects<std::invalid_argument>([&]{CoatedDiffuseReflection c(1,x,.2f,.3f);}));
    }
    Check("reject unrepresentable relative index",Rejects<std::overflow_error>([]{CoatedDiffuseReflection c(1e-300,1e300,.2f,.3f);}));
    for(float x:{0.f,-1.f,1e-4f,float(nan),float(inf)})
    {
        Check("reject invalid alphaX or Delta",Rejects<std::invalid_argument>([&]{CoatedDiffuseReflection c(1,1.5,x,.3f);}));
        Check("reject invalid alphaY or Delta",Rejects<std::invalid_argument>([&]{CoatedDiffuseReflection c(1,1.5,.2f,x);}));
    }
    for(double x:{-1.,nan,inf})Check("reject invalid optical thickness",Rejects<std::invalid_argument>([&]{CoatedDiffuseReflection c(1,1.5,.2f,.3f,x);}));
    for(double x:{-1.,1.001,nan,inf})Check("reject invalid base reflectance",Rejects<std::invalid_argument>([&]{(void)model.Evaluate(n,n,x);}));
    for(Vector3f w:{Vector3f(0,0,0),Vector3f(float(nan),0,1),Vector3f(0,float(inf),1)})
        Check("reject invalid direction",Rejects<std::invalid_argument>([&]{(void)model.Evaluate(n,w,.5);}));
    Check("reject backside and tangent in local frame",model.Evaluate(n,-n,.5)==0&&model.Evaluate({1,0,0},n,.5)==0);
    // Exact no-index-contrast and Beer-Lambert controls, several angles/rhos.
    const CoatedDiffuseReflection matched(1,1,.2f,.3f),absorbing(1,1,.2f,.3f,.7);
    Check("equal index extreme grazing remains straight through",Near(matched.Evaluate(n,{1,0,1e-6f},.5),.5/pi));
    Check("equal index Fresnel exact grazing limit",FrDielectric(0.f,1.f,1.f)==0&&FrDielectric(1e-6f,1.f,1.f)==0);
    for(double rho:{0.,.2,.7,1.})for(Vector3f w:{n,a,Vector3f(1,0,.05f)})
    {
        const auto wn=MicrofacetReflectionDetail::NormalizeDirection(w);
        Check("matched media reduce to Lambert",Near(matched.Evaluate(n,w,rho),rho/pi,3e-6));
        Check("matched absorbing layer matches two crossing exponential",Near(absorbing.Evaluate(n,w,rho),rho/pi*std::exp(-.7*(1+1/double(wn.z))),3e-6));
    }
    const LayeredReflectionDetail::DielectricInterface roughTop(1.5f,TrowbridgeReitzDistribution(.2f,.3f));
    Check("black base leaves existing dielectric direct reflection",model.Evaluate(a,b,0)==roughTop.f(MicrofacetReflectionDetail::NormalizeDirection(b),MicrofacetReflectionDetail::NormalizeDirection(a),TransportMode::Radiance));
    changed=options;changed.samplesPerEvaluation=1;changed.maxInternalReflections=0;
    Check("direct reflection not divided by internal sample count",CoatedDiffuseReflection(1,1.5,.2f,.3f,0,changed).Evaluate(a,b,0)==model.Evaluate(a,b,0));
    RejectingTop rejected;
    const LayeredReflectionDetail::DiffuseInterface bottom(.5);
    const auto failed=EvaluateScalarLayeredReflectionMonteCarlo(rejected,bottom,n,n,0,options);
    Check("rejected paths retain original N and are not retried",failed.internalReflection==0&&failed.directReflection==.125&&rejected.calls==2*options.samplesPerEvaluation);
    changed=options;changed.samplesPerEvaluation=65536;changed.maxInternalReflections=0;
    const auto half=EvaluateScalarLayeredReflectionMonteCarlo(HalfTransmissionTop{},bottom,n,n,0,changed);
    Check("partial rejection not normalized by accepted samples",Near(half.internalReflection,.25*.5/pi,.02));
    // Smooth interface is accessible in generic core only, as a test oracle.
    const LayeredReflectionDetail::DielectricInterface smoothTop(1.5f,TrowbridgeReitzDistribution(0,0));
    const double favg=InternalFresnel(32768),favgFine=InternalFresnel(65536);
    Check("smooth Fresnel quadrature converges",Near(favg,favgFine,1e-5));
    const double trt=.5/pi*.96*.96/2.25,reference=trt/(1-.5*favgFine);
    changed.russianRoulette=false;
    const auto first=EvaluateScalarLayeredReflectionMonteCarlo(smoothTop,bottom,n,n,0,changed);
    Check("smooth first TRT has correct eta squared",Near(first.internalReflection,trt));
    Check("zero depth means TRT only not zero BRDF",first.internalReflection>0&&first.pathsAtReflectionLimit==changed.samplesPerEvaluation);
    changed.maxInternalReflections=32;
    const auto full=EvaluateScalarLayeredReflectionMonteCarlo(smoothTop,bottom,n,n,0,changed);
    Check("smooth multilayer agrees with analytic geometric series",Near(full.internalReflection,reference,.01));
    Check("internal reflections add energy beyond first TRT",full.internalReflection>first.internalReflection);
    changed.russianRoulette=true;
    const auto rr=EvaluateScalarLayeredReflectionMonteCarlo(smoothTop,bottom,n,n,0,changed);
    Check("compensated RR agrees with smooth analytic control",Near(rr.internalReflection,reference,.015));
    std::cout<<"[INFO] coated diffuse smooth MC="<<full.internalReflection<<" RR="<<rr.internalReflection<<" oracle="<<reference<<'\n';
    // Independent direct BTDF integration tests rough transmission weights/PDFs.
    const LayeredReflectionDetail::DielectricInterface quadTop(1.5f,TrowbridgeReitzDistribution(.4f,.4f));
    // At normal incidence this isotropic interface is azimuth-independent.
    // Refraction concentrates the BTDF near mu=1: spend resolution on mu,
    // rather than repeating the same azimuthal integrand 64 times.
    const double q1=TransmissionIntegral(quadTop,n,TransportMode::Radiance,4096,1);
    const double q2=TransmissionIntegral(quadTop,n,TransportMode::Radiance,8192,1);
    Check("rough BTDF quadrature resolution stable",Near(q1,q2,.005));
    changed.maxInternalReflections=0;changed.russianRoulette=false;
    const auto roughFirst=EvaluateScalarLayeredReflectionMonteCarlo(quadTop,bottom,n,n,0,changed);
    const double roughReference=.5/pi*q2*q2*2.25; // Importance/Radiance transmission ratio eta^2.
    Check("rough first TRT matches independent BTDF integration",Near(roughFirst.internalReflection,roughReference,.02));
    std::cout<<"[INFO] coated diffuse rough TRT="<<roughFirst.internalReflection<<" quadrature="<<roughReference<<'\n';
    // World adapter, spectral ownership and geometric gates.
    SpectralReflectanceCurve source({{500,.2},{905,.4},{1600,.8}});
    const CoatedDiffuseLidarScattering spectral(1,1,.2f,.3f,source);
    source=SpectralReflectanceCurve({{500,.9},{1600,.9}});
    HitRecord hit{};hit.geometricNormal=hit.normal={0,1,0};hit.dpdu={1,0,0};
    const Vector3f up(0,1,0),oblique(.2f,1,.3f);
    Check("owns base reflectance curve",spectral.BaseReflectanceAtWavelength(905)==.4);
    for(float wavelength:{500.f,905.f,1550.f,1600.f})
        Check("wavelength scalar remains separate from RGB",Near(spectral.Evaluate(hit,up,oblique,wavelength),spectral.BaseReflectanceAtWavelength(wavelength)/pi));
    Check("spectral boundary precedes backside rejection",Rejects<std::out_of_range>([&]{(void)spectral.Evaluate(hit,-up,up,1601);}));
    for(float wavelength:{0.f,-1.f,float(nan),float(inf)})Check("reject invalid wavelength",Rejects<std::invalid_argument>([&]{(void)spectral.Evaluate(hit,up,up,wavelength);}));
    Check("geometric hemisphere rejection",spectral.Evaluate(hit,-up,up,905)==0&&spectral.Evaluate(hit,up,-up,905)==0);
    for(Vector3f normal:{Vector3f(0,0,0),-up,Vector3f(float(nan),0,0)})
    {auto missing=hit;missing.normal=normal;Check("normal fallback or orientation correction",Near(spectral.Evaluate(missing,up,up,905),.4/pi));}
    for(Vector3f tangent:{Vector3f(0,0,0),up,Vector3f(float(inf),0,0)})
    {auto missing=hit;missing.dpdu=tangent;Check("degenerate tangent fallback",Near(spectral.Evaluate(missing,up,up,905),.4/pi));}
    Check("matched media retain direction scale invariance",Near(spectral.Evaluate(hit,up*5,oblique*7,905),spectral.Evaluate(hit,up,oblique,905)));
    auto rotated=hit;rotated.geometricNormal=rotated.normal={0,0,1};
    Check("matched media retain world rotation invariance",Near(spectral.Evaluate(rotated,n,n,905),spectral.Evaluate(hit,up,up,905)));
    const auto copy=spectral;
    Check("copied model reproducible",copy.Evaluate(hit,up,up,1550)==spectral.Evaluate(hit,up,up,1550));
    // Finite roughness hemisphere energy smoke check (same geometric/shading normal).
    auto energyOptions=options;energyOptions.samplesPerEvaluation=128;
    const CoatedDiffuseReflection energyModel(1,1.5,.35f,.35f,0,energyOptions);
    double energy=0;
    for(int i=0;i<24;++i)for(int j=0;j<48;++j)
    {
        const double mu=(i+.5)/24,phi=2*pi*(j+.5)/48,s=std::sqrt(1-mu*mu);
        energy+=energyModel.Evaluate(n,{float(s*std::cos(phi)),float(s*std::sin(phi)),float(mu)},.5)*mu*2*pi/(24*48);
    }
    Check("finite roughness directional albedo bounded smoke test",energy>0&&energy<=1.02);
    std::cout<<"[INFO] coated diffuse normal incidence albedo smoke="<<energy<<'\n';
    // Existing per-face binding -> LiDAR return -> waveform. Matched interface
    // gives an independent Lambert energy oracle, not a self-referential f check.
    const auto fixture=std::filesystem::path(__FILE__).parent_path()/"fixtures"/"obj_materials";
    auto shared=std::make_shared<CoatedDiffuseLidarScattering>(spectral);
    ObjMaterialBindingOptions binding;binding.spectraDirectory=fixture;
    binding.overrides["low"]=std::make_shared<const SurfaceMaterial>(SurfaceMaterial{nullptr,shared});
    const auto asset=ObjSceneAsset::Read(fixture/"materials.obj",{},binding);
    SceneBuilder builder;asset.AddToScene(builder);const auto scene=builder.Build();const SurfaceLidarScattering dispatch;
    HitRecord bound{};
    Check("OBJ override binds model without auto material guessing",scene->hit(Ray({-.5f,15,.5f},{0,-1,0}),1e-4f,100,bound)&&bound.lidarScattering==shared);
    for(float wavelength:{905.f,1550.f})
    {
        const auto pulse=Pulse(*scene,dispatch,1,wavelength),many=Pulse(*scene,dispatch,16,wavelength);
        const double energyJ=double(1e-3f)*spectral.BaseReflectanceAtWavelength(wavelength)/pi*double(.01f)/225*double(.8f);
        Check("full chain produces returns",pulse.returns.size()==1&&many.returns.size()==16);
        Check("full chain independent analytic received energy",Near(pulse.TotalReceivedEnergyJ(),energyJ));
        Check("full chain outer MC normalization unchanged",Near(many.TotalReceivedEnergyJ(),pulse.TotalReceivedEnergyJ()));
        Check("full chain geometric propagation time unchanged",Near(pulse.returns.at(0).arrivalTimeSeconds,30./299792458.));
        LidarWaveform waveform({0,1e-9,256});
        Check("full chain waveform energy conserved",Near(waveform.AccumulatePulse(many),many.TotalReceivedEnergyJ()));
    }
}
