#include "reflection/materials/SurfaceMaterial.h" // Self-contained forward-declared model types.
#include "reflection/lidar/SurfaceLidarScattering.h"
#include "scene/SceneBuilder.h"
#include "reflection/materials/DiffuseMaterial.h"
#include "reflection/lidar/LambertianLidarScattering.h"
#include "reflection/lidar/SpectralLambertianLidarScattering.h"
#include "lidar/simulation/LidarIntegrator.h"
#include "lidar/simulation/LidarWaveform.h"
#include "core/sampling/IndependentSampler.h"
#include "geometry/primitives/Sphere.h"
#include "geometry/primitives/Triangle.h"
#include <cmath>
#include <iostream>
#include <numbers>
#include <type_traits>

namespace
{
void Check(const char* name, bool condition)
{
    if (!condition) { std::cerr << "[FAIL] " << name << '\n'; throw std::runtime_error(name); }
    std::cout << "[PASS] " << name << '\n';
}
template<class Exception, class Function> bool Rejects(Function function)
{
    try { function(); } catch (const Exception&) { return true; } return false;
}
bool Near(double a, double b, double relative = 1e-6)
{
    return std::isfinite(a) && std::isfinite(b) &&
        std::abs(a-b) <= 1e-22 + relative * std::max(std::abs(a), std::abs(b));
}
ImportedMesh Quad()
{
    // Two +Y triangles share the diagonal and all four vertices.
    std::vector<MeshVertex> vertices{
        {{-1,0,-1},{0,1,0},{0,0,0}}, {{-1,0,1},{0,1,0},{0,1,0}},
        {{1,0,1},{0,1,0},{1,1,0}}, {{1,0,-1},{0,1,0},{1,0,0}}};
    return {"quad",std::make_shared<TriangleMesh>(std::move(vertices),
        std::vector<TriangleIndices>{{0,1,2},{0,2,3}}),{0,1}};
}
Ray Probe(bool left)
{
    return Ray(left ? Vector3f(-.5f,15,.5f) : Vector3f(.5f,15,-.5f),{0,-1,0});
}
bool Hit(const Hittable& world, bool left, HitRecord& hit)
{
    return world.hit(Probe(left),1e-4f,100,hit);
}
LidarPulseResult Pulse(const Hittable& world, const LidarScatteringModel& model,
    bool left, float wavelength=532, std::size_t count=1)
{
    const auto ray=Probe(left);
    const LaserEmitter emitter(ray.orig,ray.dir,wavelength,1e-3f,0);
    const LidarReceiver receiver(ray.orig,ray.dir,.25f,.01f,.8f);
    const LidarIntegrator integrator(world,emitter,receiver,count,model);
    IndependentSampler sampler(42);
    return integrator.SimulatePulse(sampler);
}
std::shared_ptr<const SurfaceMaterial> Bind(std::shared_ptr<Material> render,
    std::shared_ptr<const LidarScatteringModel> lidar)
{
    return std::make_shared<const SurfaceMaterial>(SurfaceMaterial{std::move(render),std::move(lidar)});
}
class NegativeModel final : public LidarScatteringModel
{
    double Evaluate(const HitRecord&,const Vector3f&,const Vector3f&,float) const override { return -1; }
};
class RecordingModel final : public LidarScatteringModel
{
public:
    mutable const HitRecord* recordedHit=nullptr;
    mutable Vector3f incident,outgoing;
    mutable float wavelength=0;
    double Evaluate(const HitRecord& hit,const Vector3f& wi,const Vector3f& wo,float lambda) const override
    {
        recordedHit=&hit;incident=wi;outgoing=wo;wavelength=lambda;return .125;
    }
};
}

void RunSurfaceMaterialAcceptanceTests()
{
    static_assert(std::is_base_of_v<LidarScatteringModel,SurfaceLidarScattering>);
    const SurfaceMaterial empty{};
    Check("surface material permits explicit empty model pair",!empty.renderMaterial&&!empty.lidarScattering);
    const auto renderLeft=std::make_shared<DiffuseMaterial>(Spectrum(.25f));
    const auto renderRight=std::make_shared<DiffuseMaterial>(Spectrum(.75f));
    const auto lidarLeft=std::make_shared<LambertianLidarScattering>(.2);
    const auto lidarRight=std::make_shared<LambertianLidarScattering>(.6);
    const auto left=Bind(renderLeft,lidarLeft),right=Bind(renderRight,lidarRight);
    SurfaceMaterialBindings bindings{left,right};
    auto source=Quad();
    Check("surface material shared vertices remain geometry only",source.mesh->Vertices().size()==4&&source.mesh->Triangles().size()==2);
    auto faces=CreateMeshTriangles(source.mesh,bindings);
    HitRecord hitLeft{},hitRight{};
    Check("surface material resolves each face render model",Hit(*faces[0],true,hitLeft)&&Hit(*faces[1],false,hitRight)&&hitLeft.material==renderLeft&&hitRight.material==renderRight);
    Check("surface material resolves each face lidar model",hitLeft.lidarScattering==lidarLeft&&hitRight.lidarScattering==lidarRight);
    Check("surface material equal geometry path lengths",hitLeft.t==15&&hitRight.t==15);
    MaterialEvalContext ctx{hitLeft.point,{0,1,0},hitLeft.normal,hitLeft.dpdu,hitLeft.u,hitLeft.v};
    const auto bsdfLeft=hitLeft.material->GetBSDF(ctx);
    ctx={hitRight.point,{0,1,0},hitRight.normal,hitRight.dpdu,hitRight.u,hitRight.v};
    const auto bsdfRight=hitRight.material->GetBSDF(ctx);
    Check("surface material render BSDF still uses selected material",Near(bsdfRight.f({0,1,0},{0,1,0}).c[0]/bsdfLeft.f({0,1,0},{0,1,0}).c[0],3));
    const SurfaceLidarScattering strict;
    Check("surface material dispatch returns selected BRDF",Near(strict.Evaluate(hitRight,{0,1,0},{0,1,0},532),.6/std::numbers::pi_v<double>));
    HitRecord missing{};
    Check("surface material strict missing binding fails",Rejects<std::logic_error>([&]{strict.Evaluate(missing,{0,1,0},{0,1,0},532);}));
    const SurfaceLidarScattering compatible(lidarLeft);
    Check("surface material explicit fallback used for missing binding",Near(compatible.Evaluate(missing,{0,1,0},{0,1,0},532),.2/std::numbers::pi_v<double>));
    Check("surface material bound model takes precedence over fallback",Near(compatible.Evaluate(hitRight,{0,1,0},{0,1,0},532),.6/std::numbers::pi_v<double>));
    const auto recorder=std::make_shared<RecordingModel>();
    missing.lidarScattering=recorder;const Vector3f wi(.1f,1,.2f),wo(.3f,1,-.1f);
    Check("surface material dispatch preserves model value",strict.Evaluate(missing,wi,wo,905)==.125);
    Check("surface material forwards hit directions and wavelength",recorder->recordedHit==&missing&&recorder->incident.x==wi.x&&recorder->outgoing.z==wo.z&&recorder->wavelength==905);
    const auto dispatcher=std::make_shared<SurfaceLidarScattering>();missing.lidarScattering=dispatcher;
    Check("surface material rejects dispatcher bound to face",Rejects<std::logic_error>([&]{strict.Evaluate(missing,wi,wo,532);}));
    Check("surface material rejects dispatcher fallback",Rejects<std::invalid_argument>([&]{SurfaceLidarScattering bad(dispatcher);}));

    Check("surface material collection rejects null mesh",Rejects<std::invalid_argument>([&]{CreateMeshTriangles(nullptr,bindings);}));
    Check("surface material collection rejects mismatched count",Rejects<std::invalid_argument>([&]{CreateMeshTriangles(source.mesh,SurfaceMaterialBindings{left});}));
    Check("surface material collection rejects null binding",Rejects<std::invalid_argument>([&]{CreateMeshTriangles(source.mesh,SurfaceMaterialBindings{left,nullptr});}));
    Check("surface material direct bound triangle validates index",Rejects<std::out_of_range>([&]{MeshTriangle bad(source.mesh,2,*left);}));
    Check("surface material aggregate rejects null mesh",Rejects<std::invalid_argument>([&]{TriangleMeshAggregate bad(nullptr,bindings);}));
    Check("surface material aggregate rejects zero leaf capacity",Rejects<std::invalid_argument>([&]{TriangleMeshAggregate bad(source.mesh,bindings,0);}));
    TriangleMeshAggregate aggregate(source.mesh,bindings,1);
    Check("surface material aggregate BVH preserves both bindings",Hit(aggregate,true,hitLeft)&&Hit(aggregate,false,hitRight)&&hitLeft.lidarScattering==lidarLeft&&hitRight.lidarScattering==lidarRight);
    aggregate.Refit();aggregate.Rebuild();
    Check("surface material refit rebuild preserve models",Hit(aggregate,true,hitLeft)&&Hit(aggregate,false,hitRight)&&hitLeft.lidarScattering==lidarLeft&&hitRight.lidarScattering==lidarRight);
    Check("surface material repeated faces share model instances",CreateMeshTriangles(source.mesh,SurfaceMaterialBindings{left,left})[1]->hit(Probe(false),1e-4f,100,hitRight)&&hitRight.lidarScattering.get()==lidarLeft.get());
    // A reused output record must not retain a previous face's LiDAR model.
    MeshTriangle legacy(source.mesh,0,renderLeft);
    hitLeft.lidarScattering=lidarRight;
    Check("surface material legacy mesh triangle clears lidar model",Hit(legacy,true,hitLeft)&&!hitLeft.lidarScattering&&hitLeft.material==renderLeft);
    Sphere sphere({0,0,0},1,renderRight);hitLeft.lidarScattering=lidarLeft;
    Check("surface material legacy sphere clears lidar model",sphere.hit(Ray({0,3,0},{0,-1,0}),1e-4f,100,hitLeft)&&!hitLeft.lidarScattering&&hitLeft.material==renderRight);
    Triangle triangle({-1,0,-1},{-1,0,1},{1,0,1},{0,1,0},{0,1,0},{0,1,0},{0,0,0},{0,1,0},{1,1,0},renderLeft);
    hitLeft.lidarScattering=lidarRight;
    Check("surface material legacy triangle clears lidar model",Hit(triangle,true,hitLeft)&&!hitLeft.lidarScattering&&hitLeft.material==renderLeft);
    const auto unbound=Bind(nullptr,nullptr);
    MeshTriangle emptyFace(source.mesh,0,*unbound);hitLeft.SetSurfaceMaterials(renderLeft,lidarLeft);
    Check("surface material explicit unbound face clears both fields",Hit(emptyFace,true,hitLeft)&&!hitLeft.material&&!hitLeft.lidarScattering);

    SceneBuilder builder;
    Check("surface material scene returns stable object index",builder.AddMeshWithSurfaceMaterials(source,bindings)==0);
    bindings[0]=right;bindings.clear();
    const auto scene=builder.Build(1,1);
    Check("surface material scene snapshots binding vector",Hit(*scene,true,hitLeft)&&hitLeft.lidarScattering==lidarLeft);
    source.mesh->UpdateVertex(0,{100,100,100},{0,1,0});
    Check("surface material scene still snapshots source geometry",Hit(*scene,false,hitRight)&&hitRight.t==15&&hitRight.lidarScattering==lidarRight);
    SceneBuilder invalid;auto fresh=Quad();
    Check("surface material scene rejects missing face binding",Rejects<std::invalid_argument>([&]{invalid.AddMeshWithSurfaceMaterials(fresh,{left});})&&invalid.ObjectCount()==0);
    Check("surface material scene rejects null binding atomically",Rejects<std::invalid_argument>([&]{invalid.AddMeshWithSurfaceMaterials(fresh,{left,nullptr});})&&invalid.ObjectCount()==0);
    Check("surface material scene rejects null source",Rejects<std::invalid_argument>([&]{invalid.AddMeshWithSurfaceMaterials(ImportedMesh{},{});})&&invalid.ObjectCount()==0);
    const auto lifetime=[] {
        SceneBuilder local;auto mesh=Quad();
        auto model=std::make_shared<LambertianLidarScattering>(.4);
        auto binding=Bind(nullptr,model);
        local.AddMeshWithSurfaceMaterials(mesh,{binding,binding});return local.Build();
    }();
    Check("surface material scene owns models beyond caller lifetimes",Hit(*lifetime,true,hitLeft)&&!hitLeft.material&&hitLeft.lidarScattering&&Near(strict.Evaluate(hitLeft,{0,1,0},{0,1,0},532),.4/std::numbers::pi_v<double>));
    SceneBuilder placed;SceneObjectPose pose;pose.translationMeters={0,5,0};
    placed.AddMeshWithSurfaceMaterials(fresh,{left,right},pose);
    Check("surface material placement preserves face assignment",Hit(*placed.Build(),false,hitRight)&&hitRight.t==10&&hitRight.lidarScattering==lidarRight);
    builder.AddMesh(fresh,pose,renderLeft);const auto nearerLegacy=builder.Build();
    Check("surface material nearest unbound surface not skipped",Hit(*nearerLegacy,true,hitLeft)&&hitLeft.t==10&&!hitLeft.lidarScattering);
    Check("surface material missing nearest model fails strict pulse",Rejects<std::logic_error>([&]{Pulse(*nearerLegacy,strict,true);}));
    Check("surface material missing nearest model allows explicit fallback",Pulse(*nearerLegacy,compatible,true).returns.size()==1);

    const auto low=Pulse(*scene,strict,true),high=Pulse(*scene,strict,false);
    Check("surface material two bound faces produce returns",low.surfaceHitCount==1&&high.surfaceHitCount==1&&low.returns.size()==1&&high.returns.size()==1);
    const double expected=double(1e-3f)*(.2/std::numbers::pi_v<double>)*double(.01f)/225*double(.8f);
    Check("surface material analytic received energy",Near(low.TotalReceivedEnergyJ(),expected));
    Check("surface material equal range reflectance energy ratio three",Near(high.TotalReceivedEnergyJ()/low.TotalReceivedEnergyJ(),3));
    Check("surface material binding does not alter propagation time",low.returns.at(0).arrivalTimeSeconds==high.returns.at(0).arrivalTimeSeconds&&Near(low.returns.at(0).arrivalTimeSeconds,30.0/299792458.0));
    const auto many=Pulse(*scene,strict,true,532,16);
    Check("surface material MC normalization preserved",many.returns.size()==16&&Near(many.TotalReceivedEnergyJ(),low.TotalReceivedEnergyJ()));
    LidarWaveform waveform({0,1e-9,256});
    Check("surface material waveform energy conserved",Near(waveform.AccumulatePulse(many),many.TotalReceivedEnergyJ()));
    Check("surface material explicit fixed model still overrides bindings",Near(Pulse(*scene,*lidarLeft,false).TotalReceivedEnergyJ(),low.TotalReceivedEnergyJ()));
    const auto spectral=std::make_shared<SpectralLambertianLidarScattering>(SpectralReflectanceCurve({{532,.2},{1550,.6}}));
    SceneBuilder spectralBuilder;auto spectralBinding=Bind(nullptr,spectral);
    spectralBuilder.AddMeshWithSurfaceMaterials(fresh,{spectralBinding,spectralBinding});const auto spectralScene=spectralBuilder.Build();
    Check("surface material spectral lidar only scene works",Pulse(*spectralScene,strict,true).returns.size()==1);
    Check("surface material spectral wavelength energy ratio",Near(Pulse(*spectralScene,strict,true,1550).TotalReceivedEnergyJ()/Pulse(*spectralScene,strict,true,532).TotalReceivedEnergyJ(),3));
    Check("surface material fallback does not hide spectral out of band",Rejects<std::out_of_range>([&]{Pulse(*spectralScene,compatible,true,1600);}));
    const auto black=Bind(nullptr,std::make_shared<LambertianLidarScattering>(0));SceneBuilder darkBuilder;
    darkBuilder.AddMeshWithSurfaceMaterials(fresh,{black,black});const auto dark=Pulse(*darkBuilder.Build(),compatible,true);
    Check("surface material black binding not replaced by fallback",dark.surfaceHitCount==1&&dark.returns.empty());
    const auto negative=Bind(nullptr,std::make_shared<NegativeModel>());SceneBuilder negativeBuilder;
    negativeBuilder.AddMeshWithSurfaceMaterials(fresh,{negative,negative});
    Check("surface material invalid BRDF still rejected by return estimator",Rejects<std::domain_error>([&]{Pulse(*negativeBuilder.Build(),compatible,true);}));
}
