#include "reflection/lidar/RoughConductorLidarScattering.h" // Public header must be self-contained.
#include "reflection/bxdfs/ConductorBxDF.h"
#include "scene/import/ObjSceneAsset.h"
#include "reflection/lidar/SurfaceLidarScattering.h"
#include "lidar/simulation/LidarIntegrator.h"
#include "lidar/simulation/LidarWaveform.h"
#include "core/sampling/IndependentSampler.h"
#include <iostream>
#include <limits>
#include <numbers>
#include <type_traits>

namespace
{
void Check(const std::string& name, bool condition)
{
    if (!condition) { std::cerr << "[FAIL] " << name << '\n'; throw std::runtime_error(name); }
    std::cout << "[PASS] " << name << '\n';
}
bool Near(double a, double b, double relative = 1e-6)
{
    return std::isfinite(a) && std::isfinite(b) &&
        std::abs(a-b) <= 1e-22 + relative * std::max(std::abs(a),std::abs(b));
}
template<class Exception, class Function> bool Rejects(Function function)
{
    try { function(); } catch (const Exception&) { return true; } return false;
}
double NormalF(double etaI, double n, double k)
{
    return ((n-etaI)*(n-etaI)+k*k)/((n+etaI)*(n+etaI)+k*k);
}
struct InvalidDistribution
{
    double d=1,g=1;
    double D(const Vector3f&) const { return d; }
    double G(const Vector3f&,const Vector3f&) const { return g; }
};
struct CountingDistribution
{
    mutable int dCalls=0,gCalls=0;
    double D(const Vector3f&) const { ++dCalls; return 2; }
    double G(const Vector3f&,const Vector3f&) const { ++gCalls; return .5; }
};
// Independent test oracle, not a second production implementation: Cartesian
// GGX expression and complex amplitudes check the shared kernel after refactoring.
double ReferenceConductor(Vector3f wo, Vector3f wi, double n, double k,
    double alphaX, double alphaY)
{
    wo=wo.normalize();wi=wi.normalize();
    const auto m=(wo+wi).normalize();
    const double cosine=std::clamp(double(wo.dot(m)),-1.,1.);
    const std::complex<double> eta(n,k);
    const auto ct=std::sqrt(std::complex<double>(1,0)-(1-cosine*cosine)/(eta*eta));
    const auto rp=(eta*cosine-ct)/(eta*cosine+ct);
    const auto rs=(cosine-eta*ct)/(cosine+eta*ct);
    const double f=(std::norm(rp)+std::norm(rs))*.5;
    const double q=double(m.x)*m.x/(alphaX*alphaX)+double(m.y)*m.y/(alphaY*alphaY)+double(m.z)*m.z;
    const double d=1/(std::numbers::pi_v<double>*alphaX*alphaY*q*q);
    auto lambda=[&](const Vector3f& w) {
        return (std::sqrt(1+(alphaX*alphaX*double(w.x)*w.x+
            alphaY*alphaY*double(w.y)*w.y)/(double(w.z)*w.z))-1)*.5;
    };
    return f*d/(1+lambda(wo)+lambda(wi))/(4*double(wi.z)*wo.z);
}
double IntegrateEnergy(int nMu, int nPhi, const Vector3f& wi)
{
    // Deterministic midpoint quadrature: dOmega = dMu*dPhi, mu=cos(theta_o).
    // Same geometric and shading frame. Not proof for arbitrary perturbed normals.
    TrowbridgeReitzDistribution distribution(.3f,.3f);
    double sum=0;const double pi=std::numbers::pi_v<double>;
    for(int i=0;i<nMu;++i)for(int j=0;j<nPhi;++j)
    {
        const double mu=(i+.5)/nMu,phi=2*pi*(j+.5)/nPhi;
        const double s=std::sqrt(1-mu*mu);
        const Vector3f wo(float(s*std::cos(phi)),float(s*std::sin(phi)),float(mu));
        sum += EvaluateMicrofacetConductorReflection(distribution,wo,wi,1,.3,3)*mu;
    }
    return sum*(2*pi)/(double(nMu)*nPhi);
}
LidarPulseResult Pulse(const Hittable& world,const LidarScatteringModel& scattering,
    std::size_t count=1,float wavelength=905)
{
    const Vector3f sensor(-.5f,15,.5f),down(0,-1,0);
    const LaserEmitter emitter(sensor,down,wavelength,1e-3f,0);
    const LidarReceiver receiver(sensor,down,.25f,.01f,.8f);
    const LidarIntegrator integrator(world,emitter,receiver,count,scattering);
    IndependentSampler sampler(42);return integrator.SimulatePulse(sampler);
}
}

void RunRoughConductorLidarAcceptanceTests()
{
    static_assert(std::is_same_v<decltype(FrConductor(1.f,1.f,.3f,3.f)),float>);
    static_assert(std::is_same_v<decltype(FrConductor(1.,1.,.3,3.)),double>);
    static_assert(std::is_base_of_v<LidarScatteringModel,RoughConductorLidarScattering>);
    const double pi=std::numbers::pi_v<double>,f0=NormalF(1,.3,3);
    Check("conductor Fresnel analytic normal incidence",Near(FrConductor(1.,1.,.3,3.),f0,1e-12));
    Check("conductor Fresnel incident medium index explicit",Near(FrConductor(1.,1.33,.3,3.),NormalF(1.33,.3,3),1e-12));
    Check("conductor Fresnel optical constants scale invariant",Near(FrConductor(.5,1.,.3,3.),FrConductor(.5,2.,.6,6.),1e-12));
    Check("conductor Fresnel clamps finite cosine like PBRT",Near(FrConductor(2.,1.,.3,3.),f0,1e-12));
    Check("conductor Fresnel grazing endpoint one",FrConductor(0.,1.,.3,3.)==1);
    Check("conductor Fresnel matched nonabsorbing media zero",FrConductor(.5,1.,1.,0.)==0&&FrConductor(0.,1.,1.,0.)==0);
    Check("conductor Fresnel imaginary index limit one",Near(FrConductor(.5,1.,0.,3.),1,1e-12));
    bool bounded=true,dielectric=true,legacy=true;
    for(int i=1;i<=100;++i)
    {
        const double cosine=double(i)/100;
        const double f=FrConductor(cosine,1.,.3,3.);
        bounded=bounded&&f>=0&&f<=1&&std::isfinite(f);
        dielectric=dielectric&&Near(FrConductor(cosine,1.,1.5,0.),FrDielectric(cosine,1.,1.5),1e-11);
        legacy=legacy&&Near(f,FrComplex(float(cosine),std::complex<float>(.3f,3.f)),1e-6);
    }
    Check("conductor Fresnel angle sweep finite bounded",bounded);
    Check("conductor Fresnel k zero matches dielectric Fresnel",dielectric);
    Check("conductor Fresnel compatibility adapter agrees with shared scalar",legacy);
    Check("conductor Fresnel k zero TIR limit",Near(FrConductor(.3,1.5,1.,0.),1,1e-12));
    Check("conductor Fresnel rejects negative n",Rejects<std::invalid_argument>([]{FrConductor(.5,1.,-.3,3.);}));
    Check("conductor Fresnel rejects negative k",Rejects<std::invalid_argument>([]{FrConductor(.5,1.,.3,-3.);}));
    Check("conductor Fresnel rejects zero complex index",Rejects<std::invalid_argument>([]{FrConductor(.5,1.,0.,0.);}));
    Check("conductor Fresnel rejects zero incident index",Rejects<std::invalid_argument>([]{FrConductor(.5,0.,.3,3.);}));
    Check("conductor Fresnel rejects nonfinite input",Rejects<std::invalid_argument>([]{FrConductor(std::numeric_limits<double>::quiet_NaN(),1.,.3,3.);}));
    Check("conductor Fresnel extreme ratio fails explicitly",Rejects<std::overflow_error>([]{FrConductor(.5,1e-300,1.,1.);}));

    TrowbridgeReitzDistribution distribution(.2f,.3f);
    const Vector3f normal(0,0,1),wi(.2f,-.1f,1),wo(-.3f,.2f,1);
    const double expected=f0/(4*pi*double(.2f)*double(.3f));
    auto local=[&](const Vector3f& a,const Vector3f& b){return EvaluateMicrofacetConductorReflection(distribution,a,b,1,.3,3);};
    Check("conductor BRDF normal incidence analytic F D G",Near(local(normal,normal),expected));
    Check("conductor BRDF may exceed one without clamping",local(normal,normal)>1);
    Check("conductor BRDF reciprocal",Near(local(wi,wo),local(wo,wi)));
    Check("conductor BRDF direction normalization",Near(local(wi*3,wo*7),local(wi,wo)));
    Check("conductor BRDF lower hemisphere zero",local(-normal,normal)==0&&local(normal,-normal)==0);
    Check("conductor BRDF horizon zero",local({1,0,0},normal)==0);
    Check("conductor BRDF invalid direction rejected",Rejects<std::invalid_argument>([&]{local({0,0,0},normal);}));
    Check("conductor BRDF invalid D rejected",Rejects<std::domain_error>([&]{EvaluateMicrofacetConductorReflection(InvalidDistribution{-1,1},normal,normal,1,.3,3);}));
    Check("conductor BRDF invalid G rejected",Rejects<std::domain_error>([&]{EvaluateMicrofacetConductorReflection(InvalidDistribution{1,1.1},normal,normal,1,.3,3);}));
    const ConductorBxDF old(distribution,Spectrum(.3f),Spectrum(3.f));
    Check("conductor BRDF agrees with existing ConductorBxDF",Near(local(wi,wo),old.f(wi.normalize(),wo.normalize(),TransportMode::Radiance).c[0]));
    // Different channels must remain different; never interpret RGB as laser wavelengths.
    const Spectrum channelN(.3f,1.5f,2.0f),channelK(3.f,0.f,4.f);
    const ConductorBxDF colored(distribution,channelN,channelK);
    bool reference=true,lower=true,scaled=true,reciprocal=true;
    const Vector3f directions[]={{0,0,1},{.2f,-.1f,1},{-.3f,.2f,1},{.8f,-.3f,.1f}};
    for(const auto& a:directions)for(const auto& b:directions)
    {
        const auto value=colored.f(a,b,TransportMode::Radiance);
        const auto negative=colored.f(-a,-b,TransportMode::Radiance);
        const auto rescaled=colored.f(a*3,b*7,TransportMode::Radiance);
        const auto reversed=colored.f(b,a,TransportMode::Radiance);
        for(std::size_t c=0;c<value.c.size();++c)
        {
            reference=reference&&Near(value.c[c],ReferenceConductor(a,b,channelN.c[c],channelK.c[c],.2f,.3f),2e-5);
            lower=lower&&Near(value.c[c],negative.c[c]);
            scaled=scaled&&Near(value.c[c],rescaled.c[c],2e-5);
            reciprocal=reciprocal&&Near(value.c[c],reversed.c[c]);
        }
    }
    Check("shared conductor RGB angular sweep independent oracle",reference);
    Check("shared conductor preserves two hemispheres",lower);
    Check("shared conductor RGB direction normalization",scaled);
    Check("shared conductor RGB reciprocity",reciprocal);
    const auto coloredNormal=colored.f(normal,normal,TransportMode::Radiance);
    bool normalChannels=true;
    for(std::size_t c=0;c<coloredNormal.c.size();++c)
        normalChannels=normalChannels&&Near(coloredNormal.c[c],NormalF(1,channelN.c[c],channelK.c[c])/(4*pi*double(.2f)*double(.3f)));
    Check("shared conductor distinct channels analytic normal incidence",normalChannels&&coloredNormal.c[0]!=coloredNormal.c[1]);
    Check("shared conductor opposite hemisphere and horizon remain zero",
        colored.f(normal,-normal,TransportMode::Radiance).IsBlack()&&colored.f({1,0,0},normal,TransportMode::Radiance).IsBlack());
    const auto sampled=colored.Sample_f(normal,{.25f,.5f});
    bool sampleMatches=sampled&&sampled->pdf>0;
    if(sampled)
    {
        const auto evaluated=colored.f(normal,sampled->wi,TransportMode::Radiance);
        sampleMatches=sampleMatches&&Near(sampled->pdf,colored.Pdf(normal,sampled->wi,TransportMode::Radiance,BxDFReflectionType::All));
        for(std::size_t c=0;c<evaluated.c.size();++c) sampleMatches=sampleMatches&&Near(sampled->f.c[c],evaluated.c[c]);
    }
    Check("shared conductor rough sampling f and PDF remain consistent",sampleMatches);
    const ConductorBxDF smooth(TrowbridgeReitzDistribution(1e-4f,1e-4f),channelN,channelK);
    const auto delta=smooth.Sample_f(normal,{.2f,.3f});
    bool deltaMatches=delta&&delta->pdf==1&&delta->flags==BxDFType::SpecularReflection&&delta->wi.z==1;
    if(delta)for(std::size_t c=0;c<delta->f.c.size();++c)
        deltaMatches=deltaMatches&&Near(delta->f.c[c],NormalF(1,channelN.c[c],channelK.c[c]));
    Check("shared conductor delta sampling preserved",deltaMatches&&smooth.f(normal,normal,TransportMode::Radiance).IsBlack()&&smooth.Pdf(normal,normal,TransportMode::Radiance,BxDFReflectionType::All)==0);
    CountingDistribution counting;
    const auto counted=EvaluateMicrofacetConductorReflection(counting,normal,normal,
        [](double) { return Spectrum(.2f,.4f,.6f); });
    Check("shared conductor computes D G once for all channels",counting.dCalls==1&&counting.gCalls==1&&Near(counted.c[0],.05)&&Near(counted.c[2],.15));
    Check("shared Fresnel compatibility adapter matched grazing finite",FrComplex(0.f,std::complex<float>(1,0))==0);
    Check("shared Fresnel compatibility adapter invalid constants rejected",Rejects<std::invalid_argument>([]{FrComplex(.5f,std::complex<float>(.3f,-3.f));}));
    const double energyNormal=IntegrateEnergy(128,256,normal);
    const double energyOblique=IntegrateEnergy(128,256,{.8660254f,0,.5f});
    const double refined=IntegrateEnergy(256,512,normal);
    Check("conductor BRDF normal hemisphere energy bounded",energyNormal>0&&energyNormal<=1.001);
    Check("conductor BRDF oblique hemisphere energy bounded",energyOblique>0&&energyOblique<=1.001);
    Check("conductor BRDF hemisphere quadrature converges",std::abs(energyNormal-refined)<.003);

    const RoughConductorLidarScattering model(1,.3,3,.2f,.3f,905);
    HitRecord hit{};hit.geometricNormal=hit.normal={0,1,0};hit.dpdu={1,0,0};const Vector3f up(0,1,0);
    auto eval=[&](const HitRecord& h,const Vector3f& a,const Vector3f& b){return model.Evaluate(h,a,b,905);};
    Check("conductor lidar world normal analytic BRDF",Near(eval(hit,up,up),expected));
    Check("conductor lidar retains configured optical constants",model.EtaIncident()==1&&model.EtaConductor()==.3&&model.KappaConductor()==3&&model.WavelengthNm()==905);
    Check("conductor lidar rejects wrong wavelength",Rejects<std::out_of_range>([&]{model.Evaluate(hit,up,up,532);}));
    Check("conductor lidar rejects invalid query wavelength",Rejects<std::invalid_argument>([&]{model.Evaluate(hit,up,up,0);}));
    Check("conductor lidar geometric backside zero",eval(hit,-up,up)==0&&eval(hit,up,-up)==0);
    HitRecord missing=hit;missing.normal={0,0,0};
    Check("conductor lidar missing shading normal fallback",Near(eval(missing,up,up),expected));
    missing.normal={std::numeric_limits<float>::quiet_NaN(),0,0};
    Check("conductor lidar invalid shading normal fallback",Near(eval(missing,up,up),expected));
    missing=hit;missing.normal=-up;
    Check("conductor lidar reversed shading normal corrected",Near(eval(missing,up,up),expected));
    missing=hit;missing.dpdu={0,0,0};
    Check("conductor lidar missing tangent fallback",Near(eval(missing,up,up),expected));
    missing.dpdu=up;
    Check("conductor lidar degenerate tangent fallback",Near(eval(missing,up,up),expected));
    missing=hit;missing.dpdu={1,3,0};
    const Vector3f worldWi(.2f,1,.1f),worldWo(-.3f,1,-.2f);
    Check("conductor lidar projects dpdu into tangent plane",Near(eval(missing,worldWi,worldWo),eval(hit,worldWi,worldWo)));
    Check("conductor lidar reciprocal in world frame",Near(eval(hit,worldWi,worldWo),eval(hit,worldWo,worldWi)));
    Check("conductor lidar direction scaling invariant",Near(eval(hit,worldWi*3,worldWo*7),eval(hit,worldWi,worldWo)));
    HitRecord rotated=hit;rotated.normal=rotated.geometricNormal={0,0,1};
    Check("conductor lidar rigid rotation invariant",Near(eval(rotated,{worldWi.x,-worldWi.z,worldWi.y},{worldWo.x,-worldWo.z,worldWo.y}),eval(hit,worldWi,worldWo)));
    Check("conductor lidar invalid geometric normal rejected",Rejects<std::invalid_argument>([&]{auto h=hit;h.geometricNormal={0,0,0};eval(h,up,up);}));
    Check("conductor lidar zero direction rejected",Rejects<std::invalid_argument>([&]{eval(hit,{0,0,0},up);}));
    Check("conductor lidar negative k rejected",Rejects<std::invalid_argument>([]{RoughConductorLidarScattering bad(1,.3,-3,.2f,.3f,905);}));
    Check("conductor lidar smooth delta rejected",Rejects<std::invalid_argument>([]{RoughConductorLidarScattering bad(1,.3,3,0,.3f,905);}));
    Check("conductor lidar nonfinite alpha rejected",Rejects<std::invalid_argument>([]{RoughConductorLidarScattering bad(1,.3,3,std::numeric_limits<float>::infinity(),.3f,905);}));
    Check("conductor lidar invalid reference wavelength rejected",Rejects<std::invalid_argument>([]{RoughConductorLidarScattering bad(1,.3,3,.2f,.3f,0);}));
    const auto factory=RoughConductorLidarScattering::FromRoughness(1,.3,3,.04f,.09f,905);
    Check("conductor lidar PBRT roughness sqrt factory",Near(factory.AlphaX(),.2)&&Near(factory.AlphaY(),.3));
    Check("conductor lidar roughness factory rejects zero",Rejects<std::invalid_argument>([]{RoughConductorLidarScattering::FromRoughness(1,.3,3,0,.1f,905);}));
    Check("conductor lidar roughness factory rejects above one",Rejects<std::invalid_argument>([]{RoughConductorLidarScattering::FromRoughness(1,.3,3,1.1f,.1f,905);}));

    const auto fixture=std::filesystem::path(__FILE__).parent_path()/"fixtures"/"obj_materials";
    auto shared=std::make_shared<RoughConductorLidarScattering>(model);
    ObjMaterialBindingOptions options;options.spectraDirectory=fixture;
    options.overrides["low"]=std::make_shared<const SurfaceMaterial>(SurfaceMaterial{nullptr,shared});
    const auto asset=ObjSceneAsset::Read(fixture/"materials.obj",{},options);
    SceneBuilder builder;asset.AddToScene(builder);const auto scene=builder.Build();const SurfaceLidarScattering dispatch;
    HitRecord bound{};
    Check("conductor lidar OBJ override binds concrete model",scene->hit(Ray({-.5f,15,.5f},{0,-1,0}),1e-4f,100,bound)&&bound.lidarScattering==shared&&asset.MaterialBindings().info[0].source==ObjLidarBindingSource::Override);
    const auto pulse=Pulse(*scene,dispatch),many=Pulse(*scene,dispatch,16);
    Check("conductor lidar complete chain produces returns",pulse.returns.size()==1&&many.returns.size()==16);
    const double expectedEnergy=double(1e-3f)*expected*double(.01f)/225*double(.8f);
    Check("conductor lidar analytic received energy",Near(pulse.TotalReceivedEnergyJ(),expectedEnergy));
    Check("conductor lidar MC normalization unchanged",Near(many.TotalReceivedEnergyJ(),pulse.TotalReceivedEnergyJ()));
    Check("conductor lidar propagation time unchanged",Near(pulse.returns.at(0).arrivalTimeSeconds,30.0/299792458.0));
    Check("conductor lidar wavelength preserved in return",pulse.returns.at(0).wavelengthNm==905);
    LidarWaveform waveform({0,1e-9,256});
    Check("conductor lidar waveform energy conserved",Near(waveform.AccumulatePulse(many),many.TotalReceivedEnergyJ()));
    Check("conductor lidar bound wrong wavelength fails explicitly",Rejects<std::out_of_range>([&]{Pulse(*scene,dispatch,1,532);}));
}
