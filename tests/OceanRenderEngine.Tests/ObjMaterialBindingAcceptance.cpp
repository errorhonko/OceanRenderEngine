#include "scene/import/ObjSceneAsset.h" // Public owning-import header must stand alone.
#include "reflection/lidar/SurfaceLidarScattering.h"
#include "lidar/simulation/LidarIntegrator.h"
#include "lidar/simulation/LidarWaveform.h"
#include "core/sampling/IndependentSampler.h"
#include <iostream>
#include <limits>
#include <numbers>

namespace
{
void Check(const std::string& name, bool condition)
{
    if (!condition) { std::cerr << "[FAIL] " << name << '\n'; throw std::runtime_error(name); }
    std::cout << "[PASS] " << name << '\n';
}
template<class Exception, class Function> bool Rejects(Function function)
{
    try { function(); } catch (const Exception&) { return true; } return false;
}
bool Near(double a, double b)
{
    return std::isfinite(a) && std::isfinite(b) && std::abs(a-b) <= 1e-22+1e-6*std::max(std::abs(a),std::abs(b));
}
std::filesystem::path Fixtures()
{
    return std::filesystem::path(__FILE__).parent_path()/"fixtures"/"obj_materials";
}
tinyobj::material_t TestMaterial(const std::string& name="test")
{
    tinyobj::material_t m{};m.name=name;m.dissolve=1;m.diffuse[0]=.1f;m.diffuse[1]=.2f;m.diffuse[2]=.3f;
    m.unknown_parameter["helios_reflectance"]="20";return m;
}
ObjMaterialBindingOptions Options()
{
    ObjMaterialBindingOptions options;options.spectraDirectory=Fixtures();return options;
}
std::shared_ptr<const SurfaceMaterial> Default()
{
    return std::make_shared<const SurfaceMaterial>(SurfaceMaterial{nullptr,std::make_shared<LambertianLidarScattering>(.5)});
}
tinyobj::ObjReader NoMaterial(bool warning=false)
{
    tinyobj::ObjReader reader;tinyobj::ObjReaderConfig options;options.triangulate=true;
    const std::string obj="v -1 0 -1\nv -1 0 1\nv 1 0 1\n"+
        std::string(warning?"usemtl absent\n":"")+"f 1 2 3\n";
    if(!reader.ParseFromString(obj,"",options))throw std::runtime_error("synthetic OBJ parse failed");return reader;
}
Ray Probe(bool low)
{
    return Ray(low?Vector3f(-.5f,15,.5f):Vector3f(.5f,15,-.5f),{0,-1,0});
}
bool Hit(const Hittable& world,bool low,HitRecord& hit)
{
    return world.hit(Probe(low),1e-4f,100,hit);
}
LidarPulseResult Pulse(const Hittable& world,bool low,float wavelength=532,std::size_t count=1)
{
    const auto ray=Probe(low);const SurfaceLidarScattering dispatch;
    const LaserEmitter emitter(ray.orig,ray.dir,wavelength,1e-3f,0);
    const LidarReceiver receiver(ray.orig,ray.dir,.25f,.01f,.8f);
    const LidarIntegrator integrator(world,emitter,receiver,count,dispatch);IndependentSampler sampler(42);
    return integrator.SimulatePulse(sampler);
}
double Brdf(const SurfaceMaterial& binding,float wavelength=532)
{
    HitRecord hit{};return binding.lidarScattering->Evaluate(hit,{0,1,0},{0,1,0},wavelength);
}
}

void RunObjMaterialBindingAcceptanceTests()
{
    const auto options=Options();auto scalar=TestMaterial();
    const auto table=ObjMaterialBinding::ConvertTable({scalar},options);
    const double pi=std::numbers::pi_v<double>;
    Check("OBJ material scalar percent converted to rho",Near(Brdf(*table.materials[0]),.2/pi));
    Check("OBJ material scalar source and name reported",table.info[0].name=="test"&&table.info[0].source==ObjLidarBindingSource::ReflectancePercent);
    Check("OBJ material render diffuse model constructed",bool(table.materials[0]->renderMaterial));
    scalar.unknown_parameter["helios_reflectance"]="0.2";
    Check("OBJ material subunit percent is not guessed as fraction",Near(Brdf(*ObjMaterialBinding::ConvertTable({scalar},options).materials[0]),.002/pi));
    Check("OBJ material subunit percent diagnostic warns about units",!ObjMaterialBinding::ConvertTable({scalar},options).info[0].warnings.empty());
    scalar.unknown_parameter["helios_reflectance"]=" +2e1 # percent";
    Check("OBJ material numeric whitespace exponent comment supported",Near(Brdf(*ObjMaterialBinding::ConvertTable({scalar},options).materials[0]),.2/pi));
    for(const std::string text:{"","nan","inf","-1","100.1","20 extra","+-1","1e9999"}) {
        scalar.unknown_parameter["helios_reflectance"]=text;
        Check("OBJ material rejects malformed percent ["+text+"]",Rejects<std::invalid_argument>([&]{ObjMaterialBinding::ConvertTable({scalar},options);}));
    }
    for(const auto percent:{0,100}) {
        scalar.unknown_parameter["helios_reflectance"]=std::to_string(percent);
        Check("OBJ material percent endpoint "+std::to_string(percent),Near(Brdf(*ObjMaterialBinding::ConvertTable({scalar},options).materials[0]),percent*.01/pi));
    }
    scalar=TestMaterial();scalar.unknown_parameter["helios_spectra"]="nonexistent";
    const auto prioritized=ObjMaterialBinding::ConvertTable({scalar},options);
    Check("OBJ material reflectance takes priority over missing spectrum",Near(Brdf(*prioritized.materials[0]),.2/pi)&&!prioritized.info[0].warnings.empty());
    auto spectral=TestMaterial("spectral");spectral.unknown_parameter.erase("helios_reflectance");spectral.unknown_parameter["helios_spectra"]="synthetic";
    auto twin=spectral;twin.name="twin";twin.unknown_parameter["helios_spectra"]="synthetic.txt";
    const auto spectra=ObjMaterialBinding::ConvertTable({spectral,twin},options);
    Check("OBJ material spectrum unit conversion reaches BRDF",Near(Brdf(*spectra.materials[0]),.6/pi));
    Check("OBJ material spectrum interpolates wavelength",Near(Brdf(*spectra.materials[0],1041),.45/pi));
    Check("OBJ material same spectrum model cached within table",spectra.materials[0]->lidarScattering==spectra.materials[1]->lidarScattering);
    Check("OBJ material spectral provenance path reported",spectra.info[0].source==ObjLidarBindingSource::Spectrum&&spectra.info[0].spectrumPath.filename()=="synthetic.txt");
    const auto spectrumModel=std::dynamic_pointer_cast<const SpectralLambertianLidarScattering>(spectra.materials[0]->lidarScattering);
    Check("OBJ material spectral source metadata preserved",spectrumModel&&spectrumModel->ReflectanceCurve().Metadata().sampleNumber=="OBJ-TEST");
    auto withFallback=options;withFallback.missingLidarFallback=Default()->lidarScattering;
    spectral.unknown_parameter["helios_spectra"]="nonexistent";
    Check("OBJ material missing spectrum not replaced by fallback",Rejects<std::runtime_error>([&]{ObjMaterialBinding::ConvertTable({spectral},withFallback);}));
    spectral.unknown_parameter["helios_spectra"]="bad";
    Check("OBJ material malformed spectrum not replaced by fallback",Rejects<std::runtime_error>([&]{ObjMaterialBinding::ConvertTable({spectral},withFallback);}));
    spectral.unknown_parameter["helios_spectra"]="synthetic";
    Check("OBJ material missing library directory rejected",Rejects<std::runtime_error>([&]{ObjMaterialBinding::ConvertTable({spectral},ObjMaterialBindingOptions{});}));
    for(const std::string name:{"","../synthetic","a/synthetic","C:\\synthetic","synthetic..txt","CON","lpt1"}) {
        spectral.unknown_parameter["helios_spectra"]=name;
        Check("OBJ material rejects unsafe spectrum name ["+name+"]",Rejects<std::invalid_argument>([&]{ObjMaterialBinding::ConvertTable({spectral},options);}));
    }
    auto missing=TestMaterial("missing");missing.unknown_parameter.clear();missing.diffuse[0]=.9f;
    Check("OBJ material missing lidar definition fails instead of using Kd",Rejects<std::invalid_argument>([&]{ObjMaterialBinding::ConvertTable({missing},options);}));
    const auto fallbackTable=ObjMaterialBinding::ConvertTable({missing},withFallback);
    Check("OBJ material explicit missing fallback reported",fallbackTable.materials[0]->lidarScattering==withFallback.missingLidarFallback&&fallbackTable.info[0].source==ObjLidarBindingSource::ExplicitFallback&&!fallbackTable.info[0].warnings.empty());
    auto overrideOptions=options;overrideOptions.overrides["missing"]=Default();
    Check("OBJ material explicit model override authoritative",ObjMaterialBinding::ConvertTable({missing},overrideOptions).materials[0]==overrideOptions.overrides["missing"]);
    overrideOptions.overrides["missing"]=nullptr;
    Check("OBJ material null model override rejected",Rejects<std::invalid_argument>([&]{ObjMaterialBinding::ConvertTable({missing},overrideOptions);}));
    scalar=TestMaterial();scalar.dissolve=.5;
    Check("OBJ material transparency rejected by default",Rejects<std::invalid_argument>([&]{ObjMaterialBinding::ConvertTable({scalar},options);}));
    auto opaque=options;opaque.allowOpaqueApproximation=true;
    Check("OBJ material opaque approximation is explicit and reported",!ObjMaterialBinding::ConvertTable({scalar},opaque).info[0].warnings.empty());
    scalar=TestMaterial();scalar.alpha_texname="alpha.png";
    Check("OBJ material alpha map rejected by default",Rejects<std::invalid_argument>([&]{ObjMaterialBinding::ConvertTable({scalar},options);}));
    scalar=TestMaterial();scalar.illum=7;
    Check("OBJ material transmissive illumination rejected",Rejects<std::invalid_argument>([&]{ObjMaterialBinding::ConvertTable({scalar},options);}));
    scalar=TestMaterial();scalar.specular[0]=.2f;scalar.diffuse_texname="not_loaded.png";scalar.emission[1]=1;
    Check("OBJ material unsupported render features reported",ObjMaterialBinding::ConvertTable({scalar},options).info[0].warnings.size()>=3);
    scalar=TestMaterial();scalar.diffuse[0]=1.1f;
    Check("OBJ material invalid decoded Kd rejected",Rejects<std::invalid_argument>([&]{ObjMaterialBinding::ConvertTable({scalar},options);}));
    auto lidarOnly=options;lidarOnly.createDiffuseRenderMaterials=false;
    Check("OBJ material lidar only mode does not convert Kd",!ObjMaterialBinding::ConvertTable({scalar},lidarOnly).materials[0]->renderMaterial);

    const auto asset=ObjSceneAsset::Read(Fixtures()/"materials.obj",{},options);
    Check("OBJ material asset retains complete source table",asset.SourceMaterials().size()==3&&asset.SourceMaterials()[0].unknown_parameter.at("helios_classification")=="2");
    Check("OBJ material asset retains source path and parse diagnostics",asset.SourcePath().filename()=="materials.obj"&&asset.ParserWarning().empty());
    Check("OBJ material asset group and triangle counts",asset.Meshes().size()==2&&asset.GeometryDiagnostics().importedTriangleCount==3);
    Check("OBJ material per face IDs resolve against local table",asset.FaceBindings()[0].size()==2&&asset.FaceBindings()[0][0]==asset.MaterialBindings().materials[0]&&asset.FaceBindings()[0][1]==asset.MaterialBindings().materials[1]);
    Check("OBJ material same source material reused across groups",asset.FaceBindings()[1][0]==asset.FaceBindings()[0][0]);
    auto copied=asset.Meshes()[0];copied.materialIds={-1,1};
    Check("OBJ material unassigned face requires explicit binding",Rejects<std::invalid_argument>([&]{ObjMaterialBinding::BindFaces(copied,asset.MaterialBindings().materials);}));
    const auto defaultBinding=Default();
    Check("OBJ material unassigned face uses explicit binding",ObjMaterialBinding::BindFaces(copied,asset.MaterialBindings().materials,defaultBinding)[0]==defaultBinding);
    copied.materialIds={-2,1};
    Check("OBJ material invalid negative face ID rejected",Rejects<std::out_of_range>([&]{ObjMaterialBinding::BindFaces(copied,asset.MaterialBindings().materials,defaultBinding);}));
    copied.materialIds={3,1};
    Check("OBJ material face ID cannot reference another asset table",Rejects<std::out_of_range>([&]{ObjMaterialBinding::BindFaces(copied,asset.MaterialBindings().materials,defaultBinding);}));
    copied.materialIds={0};
    Check("OBJ material ID triangle count mismatch rejected",Rejects<std::invalid_argument>([&]{ObjMaterialBinding::BindFaces(copied,asset.MaterialBindings().materials);}));
    copied=asset.Meshes()[0];auto nullTable=asset.MaterialBindings().materials;nullTable[0]=nullptr;
    Check("OBJ material null table entry rejected",Rejects<std::invalid_argument>([&]{ObjMaterialBinding::BindFaces(copied,nullTable);}));
    Check("OBJ material empty MTL table supported with explicit unassigned",ObjSceneAsset::FromReader(NoMaterial(),{},[&]{auto o=options;o.unassignedMaterial=defaultBinding;return o;}()).FaceBindings()[0][0]==defaultBinding);
    Check("OBJ material missing MTL never silently gains default rho",Rejects<std::invalid_argument>([&]{ObjSceneAsset::FromReader(NoMaterial(),{},withFallback);}));
    auto unassignedOptions=options;unassignedOptions.unassignedMaterial=defaultBinding;
    Check("OBJ material parser missing usemtl warning preserved",!ObjSceneAsset::FromReader(NoMaterial(true),{},unassignedOptions).ParserWarning().empty());
    Check("OBJ material invalid reader rejected",Rejects<std::invalid_argument>([&]{ObjSceneAsset::FromReader(tinyobj::ObjReader(),{},options);}));

    SceneBuilder builder;asset.AddToScene(builder);const auto scene=builder.Build(1,1);
    HitRecord lowHit{},highHit{};
    Check("OBJ material assembled scene selects each face model",Hit(*scene,true,lowHit)&&Hit(*scene,false,highHit)&&lowHit.lidarScattering==asset.MaterialBindings().materials[0]->lidarScattering&&highHit.lidarScattering==asset.MaterialBindings().materials[1]->lidarScattering);
    MaterialEvalContext ctx{lowHit.point,{0,1,0},lowHit.normal,lowHit.dpdu,lowHit.u,lowHit.v};
    Check("OBJ material RGB Kd reaches render BSDF only",Near(lowHit.material->GetBSDF(ctx).f({0,1,0},{0,1,0}).c[0],.1/pi));
    const auto low=Pulse(*scene,true),high=Pulse(*scene,false);
    Check("OBJ material import to pulse produces both returns",low.returns.size()==1&&high.returns.size()==1);
    const double expected=double(1e-3f)*(.2/pi)*double(.01f)/225*double(.8f);
    Check("OBJ material imported analytic received energy",Near(low.TotalReceivedEnergyJ(),expected));
    Check("OBJ material imported reflectance energy ratio three",Near(high.TotalReceivedEnergyJ()/low.TotalReceivedEnergyJ(),3));
    Check("OBJ material imported spectral wavelength energy ratio",Near(Pulse(*scene,false,1550).TotalReceivedEnergyJ()/high.TotalReceivedEnergyJ(),.5));
    Check("OBJ material import does not alter travel time",low.returns.at(0).arrivalTimeSeconds==high.returns.at(0).arrivalTimeSeconds&&Near(low.returns.at(0).arrivalTimeSeconds,30.0/299792458.0));
    const auto many=Pulse(*scene,false,532,16);LidarWaveform waveform({0,1e-9,256});
    Check("OBJ material imported MC normalization",many.returns.size()==16&&Near(many.TotalReceivedEnergyJ(),high.TotalReceivedEnergyJ()));
    Check("OBJ material imported waveform energy conserved",Near(waveform.AccumulatePulse(many),many.TotalReceivedEnergyJ()));
    Check("OBJ material imported out of band propagates",Rejects<std::out_of_range>([&]{Pulse(*scene,false,1600);}));
    const auto lifetime=[&]{SceneBuilder b;ObjSceneAsset::Read(Fixtures()/"materials.obj",{},options).AddToScene(b);return b.Build();}();
    Check("OBJ material scene outlives asset and reader",Hit(*lifetime,false,highHit)&&Near(Pulse(*lifetime,false).TotalReceivedEnergyJ(),high.TotalReceivedEnergyJ()));
    const auto another=ObjMaterialBinding::ConvertTable({TestMaterial("another")},options);
    Check("OBJ material asset tables do not share bare numeric IDs",another.materials[0]!=asset.MaterialBindings().materials[0]);
    const auto before=builder.ObjectCount();
    Check("OBJ material batch group count mismatch atomic",Rejects<std::invalid_argument>([&]{builder.AddMeshesWithSurfaceMaterials(asset.Meshes(),{});})&&builder.ObjectCount()==before);
    auto invalidGroups=asset.Meshes();invalidGroups[1].mesh=nullptr;
    Check("OBJ material second invalid group leaves first uncommitted",Rejects<std::invalid_argument>([&]{builder.AddMeshesWithSurfaceMaterials(invalidGroups,asset.FaceBindings());})&&builder.ObjectCount()==before);
    SceneObjectPose pose;pose.translationMeters={0,5,0};SceneBuilder placed;asset.AddToScene(placed,pose);
    Check("OBJ material placement preserves resolved models",Hit(*placed.Build(),false,highHit)&&highHit.t==10&&highHit.lidarScattering==asset.MaterialBindings().materials[1]->lidarScattering);
    ObjImportOptions cm;cm.metersPerUnit=.01;
    Check("OBJ material import preserves explicit geometry units",Near(ObjSceneAsset::Read(Fixtures()/"materials.obj",cm,options).Meshes()[0].mesh->Vertices()[0].position.x,-.01));
}
