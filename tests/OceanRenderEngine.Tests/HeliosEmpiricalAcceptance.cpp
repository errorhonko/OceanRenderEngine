#include "scene/import/ObjSceneAsset.h"
#include "reflection/lidar/SurfaceLidarScattering.h"
#include "lidar/simulation/LidarIntegrator.h"
#include "lidar/simulation/LidarWaveform.h"
#include "core/sampling/IndependentSampler.h"
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>

namespace {
using Angular = HeliosEmpiricalAngularResponse;
using Model = HeliosEmpiricalLidarScattering;
constexpr double pi = std::numbers::pi_v<double>;
void Check(const std::string& name, bool value) {
    if (!value) { std::cerr << "[FAIL] HELIOS empirical " << name << std::endl; std::exit(EXIT_FAILURE); }
    std::cout << "[PASS] HELIOS empirical " << name << std::endl;
}
template<class E, class F> bool Rejects(F f) { try { f(); } catch (const E&) { return true; } return false; }
bool Near(double a, double b, double tolerance = 2e-6) {
    return std::isfinite(a) && std::isfinite(b) && std::abs(a-b) <= 1e-25 + tolerance*std::max(std::abs(a),std::abs(b));
}
tinyobj::material_t MakeMaterial(const std::string& name, double d, double s, double ns) {
    tinyobj::material_t m{}; m.name=name; m.dissolve=1; m.shininess=float(ns);
    for(int c=0;c<3;++c){m.diffuse[c]=float(d);m.specular[c]=float(s);} return m;
}
std::shared_ptr<const Model> Empirical(const ObjMaterialBindingResult& table, std::size_t index=0) {
    const auto model=std::dynamic_pointer_cast<const Model>(table.materials.at(index)->lidarScattering);
    if(!model){std::cerr<<"[FAIL] HELIOS empirical expected model type"<<std::endl;std::exit(EXIT_FAILURE);} return model;
}
HitRecord Hit() { HitRecord hit{};hit.geometricNormal={0,1,0};hit.normal={0,1,0};return hit; }
Vector3f Direction(double cosine) { return {float(std::sqrt(1-cosine*cosine)),float(cosine),0}; }
double Response(const LidarScatteringModel& model, double cosine, float wavelength=532) {
    const auto v=Direction(cosine);return model.EvaluateProjectedResponse(Hit(),v,v,wavelength);
}
LidarPulseResult Scan(std::shared_ptr<const LidarScatteringModel> model, double cosine, std::size_t count=1) {
    tinyobj::ObjReader reader;
    if(!reader.ParseFromString("mtllib inline.mtl\nv -100 0 -100\nv 0 0 100\nv 100 0 -100\nusemtl surface\nf 1 2 3\n",
        "newmtl surface\nKd 0.2 0.2 0.2\nKs 0.6 0.6 0.6\nNs 4\nhelios_reflectance 50\n")) throw std::runtime_error("test OBJ parsing failed");
    if(reader.GetMaterials().size()!=1 || !reader.Warning().empty())
        throw std::runtime_error("test OBJ must resolve its named MTL, not the unassigned fallback");
    auto options=ObjMaterialBindingOptions::HeliosEmpiricalBaseline({});
    if(model)options.overrides["surface"]=std::make_shared<const SurfaceMaterial>(SurfaceMaterial{nullptr,std::move(model)});
    const auto asset=ObjSceneAsset::FromReader(reader,{},options);SceneBuilder builder;asset.AddToScene(builder);
    const auto scene=builder.Build();const auto direction=Direction(cosine);
    const LaserEmitter emitter(direction*10,-direction,532,1e-3f,0);
    const LidarReceiver receiver(direction*10,-direction,.2f,.01f,.8f);
    const SurfaceLidarScattering dispatch;IndependentSampler sampler(42);
    return LidarIntegrator(*scene,emitter,receiver,count,dispatch).SimulatePulse(sampler);
}
}

void RunHeliosEmpiricalAcceptanceTests() {
    const Angular phong({.2,.2,.2},{.6,.6,.6},4),lambert({.1,.1,.1},{},10),uniform({},{});
    Check("Phong classification",phong.Kind()==HeliosEmpiricalKind::Phong);
    Check("Lambert classification",lambert.Kind()==HeliosEmpiricalKind::Lambert);
    Check("zero components classify direction independent",uniform.Kind()==HeliosEmpiricalKind::DirectionIndependent);
    Check("specularity initialized in constructor",Near(phong.Specularity(),.75));
    Check("zero sum avoids division by zero",uniform.Specularity()==0);
    for(const double c:{1.,.9,.5,.1,1e-8}) {
        const double legacy=.25+.75*std::pow(std::abs(std::cos(2*std::acos(c))),4)/c;
        Check("stable algebra matches upstream at cosine "+std::to_string(c),Near(phong.Evaluate(c),legacy*c,1e-12));
    }
    Check("normal incidence gain one",phong.Evaluate(1)==1);
    Check("45 degree specular minimum",Near(phong.Evaluate(std::sqrt(.5)),.25*std::sqrt(.5),1e-12));
    Check("absolute cosine grazing lobe deliberately retained",phong.Evaluate(1e-8)>.74);
    Check("exact tangent zero without singular arithmetic",phong.Evaluate(0)==0);
    Check("back side rejected",uniform.Evaluate(-.5)==0);
    const Angular zeroExponent({.2,.2,.2},{.6,.6,.6},0);
    Check("Ns zero broad lobe",Near(zeroExponent.Evaluate(.5),.875));
    Check("Ns zero at 45 degrees finite",std::isfinite(zeroExponent.Evaluate(std::sqrt(.5))));
    const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
    Check("negative Kd rejected",Rejects<std::invalid_argument>([]{Angular({-.1,0,0},{},1);}));
    Check("nonfinite Ks rejected",Rejects<std::invalid_argument>([&]{Angular({.1,0,0},{nan,0,0},1);}));
    Check("out of range component rejected",Rejects<std::invalid_argument>([]{Angular({1.1,0,0},{},1);}));
    Check("Ks only unsupported branch explicit",Rejects<std::invalid_argument>([]{Angular({},{.1,0,0},1);}));
    Check("negative exponent rejected",Rejects<std::invalid_argument>([]{Angular({},{},-1);}));
    Check("nonfinite exponent rejected",Rejects<std::invalid_argument>([&]{Angular({},{},inf);}));
    Check("invalid cosine rejected",Rejects<std::invalid_argument>([&]{phong.Evaluate(nan);}));
    Check("cosine outside interval rejected",Rejects<std::invalid_argument>([&]{phong.Evaluate(1.1);}));
    const Model empirical(.5,phong),flat(.5,lambert),isotropic(.5,uniform);
    const LambertianLidarScattering physical(.5);
    for(const double c:{1.,.8,.5,.1})
        Check("Lambert projected convention matches physical at "+std::to_string(c),Near(Response(flat,c),Response(physical,c)));
    Check("uniform is not multiplied by an extra cosine",Near(Response(isotropic,.5),.5/pi));
    Check("Phong normalized projected response",Near(Response(empirical,.5),.5*(.25*.5+.75*.0625)/pi));
    Check("empirical cannot masquerade as pure BRDF",Rejects<std::logic_error>([&]{empirical.Evaluate(Hit(),{0,1,0},{0,1,0},532);}));
    Check("bistatic directions rejected",Rejects<std::invalid_argument>([&]{empirical.EvaluateProjectedResponse(Hit(),{0,1,0},{1,1,0},532);}));
    Check("zero direction rejected",Rejects<std::invalid_argument>([&]{empirical.EvaluateProjectedResponse(Hit(),{},{},532);}));
    auto invalidHit=Hit();invalidHit.geometricNormal={};
    Check("zero geometric normal rejected",Rejects<std::invalid_argument>([&]{empirical.EvaluateProjectedResponse(invalidHit,{0,1,0},{0,1,0},532);}));
    auto rotatedHit=Hit();rotatedHit.geometricNormal={1,0,0};rotatedHit.normal={0,1,0};
    Check("rotation and shading normal independence",Near(empirical.EvaluateProjectedResponse(rotatedHit,{.5f,.8660254f,0},{.5f,.8660254f,0},532),Response(empirical,.5)));
    Check("direction and normal scaling invariant",Near(empirical.EvaluateProjectedResponse(Hit(),{0,5,0},{0,2,0},532),Response(empirical,1)));
    Check("negative reflectance rejected",Rejects<std::invalid_argument>([&]{Model(-.1,phong);}));
    Check("reflectance above one rejected",Rejects<std::invalid_argument>([&]{Model(1.1,phong);}));
    Check("nonfinite reflectance rejected",Rejects<std::invalid_argument>([&]{Model(nan,phong);}));
    Check("invalid wavelength rejected",Rejects<std::invalid_argument>([&]{Response(empirical,.5,0);}));
    const Model spectral(SpectralReflectanceCurve({{500,.2},{1000,.6}}),phong);
    Check("spectral interpolation reused",Near(spectral.ReflectanceAtWavelength(750),.4));
    Check("spectral support remains strict",Rejects<std::out_of_range>([&]{Response(spectral,.5,1100);}));
    Check("zero reflectance endpoint",Response(Model(0,uniform),.5)==0);
    Check("unit reflectance endpoint",Near(Response(Model(1,uniform),.5),1/pi));

    auto options=ObjMaterialBindingOptions::HeliosEmpiricalBaseline(
        std::filesystem::path(__FILE__).parent_path()/"fixtures"/"obj_materials");
    auto first=MakeMaterial("first",.2,.6,4),middle=MakeMaterial("middle",.6,.2,2),last=MakeMaterial("last",.4,.4,0);
    auto table=ObjMaterialBinding::ConvertTable({first,middle,last},options);
    Check("first entry initialized without spectrum",Near(Empirical(table,0)->AngularResponse().Specularity(),.75));
    Check("middle entry initialized without spectrum",Near(Empirical(table,1)->AngularResponse().Specularity(),.25));
    Check("last entry initialized without spectrum",Near(Empirical(table,2)->AngularResponse().Specularity(),.5));
    const auto reversed=ObjMaterialBinding::ConvertTable({last,middle,first},options);
    Check("material ordering cannot change initialization",Near(Response(*Empirical(table,0),.5),Response(*Empirical(reversed,2),.5)));
    Check("empirical default rho explicit provenance",table.info[0].source==ObjLidarBindingSource::HeliosDefault&&!table.info[0].warnings.empty());
    Check("empirical angular mode reported",table.info[0].angularModel==ObjLidarAngularModel::HeliosEmpirical);
    Check("empirical default rho one half",Empirical(table)->ReflectanceAtWavelength(532)==.5);
    first.unknown_parameter["helios_reflectance"]="20";
    first.unknown_parameter["helios_spectra"]="nonexistent";
    table=ObjMaterialBinding::ConvertTable({first},options);
    Check("percent precedence reused",Near(Empirical(table)->ReflectanceAtWavelength(532),.2));
    Check("scalar source retained",table.info[0].source==ObjLidarBindingSource::ReflectancePercent);
    first.unknown_parameter["helios_reflectance"]="0.2";
    Check("subunit percent not interpreted as fraction",Near(Empirical(ObjMaterialBinding::ConvertTable({first},options))->ReflectanceAtWavelength(532),.002));
    first.unknown_parameter.erase("helios_reflectance");first.unknown_parameter["helios_spectra"]="synthetic";
    table=ObjMaterialBinding::ConvertTable({first},options);
    Check("existing spectrum reader reused",Near(Empirical(table)->ReflectanceAtWavelength(532),.6));
    Check("spectral first entry initialized",Near(Empirical(table)->AngularResponse().Specularity(),.75));
    first.unknown_parameter["helios_spectra"]="missing";
    Check("declared missing spectrum remains an error",Rejects<std::runtime_error>([&]{ObjMaterialBinding::ConvertTable({first},options);}));
    auto original=ObjMaterialBindingOptions{};
    Check("default binding mode unchanged",original.lidarAngularModel==ObjLidarAngularModel::Lambert);
    Check("default missing reflectance still strict",Rejects<std::invalid_argument>([&]{ObjMaterialBinding::ConvertTable({middle},original);}));
    original.missingLidarFallback=std::make_shared<LambertianLidarScattering>(.3);
    Check("original fallback still authoritative",ObjMaterialBinding::ConvertTable({middle},original).materials[0]->lidarScattering==original.missingLidarFallback);
    options.missingLidarFallback=original.missingLidarFallback;
    Check("empirical explicit fallback not reinterpreted",ObjMaterialBinding::ConvertTable({middle},options).materials[0]->lidarScattering==options.missingLidarFallback);
    options.overrides["middle"]=std::make_shared<const SurfaceMaterial>(SurfaceMaterial{nullptr,original.missingLidarFallback});
    Check("whole material override still authoritative",ObjMaterialBinding::ConvertTable({middle},options).materials[0]==options.overrides.at("middle"));
    const auto defaults=ObjMaterialBindingOptions::HeliosEmpiricalBaseline({});
    Check("unassigned factory default explicit uniform response",Near(Response(*defaults.unassignedMaterial->lidarScattering,.5),.5/pi));
    const SurfaceLidarScattering dispatcher(defaults.unassignedMaterial->lidarScattering);
    Check("dispatcher fallback forwards projected entry",Near(Response(dispatcher,.5),.5/pi));
    const SurfaceLidarScattering strictDispatch;
    Check("dispatcher missing model stays strict",Rejects<std::logic_error>([&]{Response(strictDispatch,.5);}));

    const auto lambertPulse=Scan(std::make_shared<LambertianLidarScattering>(.5),.5);
    const auto flatPulse=Scan(std::make_shared<Model>(.5,lambert),.5);
    const auto uniformPulse=Scan(std::make_shared<Model>(.5,uniform),.5);
    Check("OBJ scene dispatcher produces empirical return",flatPulse.returns.size()==1&&uniformPulse.returns.size()==1);
    Check("end to end Lambert energy unchanged",Near(flatPulse.TotalReceivedEnergyJ(),lambertPulse.TotalReceivedEnergyJ()));
    std::cout<<std::setprecision(17)<<"[INFO] HELIOS empirical energies physical="<<lambertPulse.TotalReceivedEnergyJ()
        <<" empiricalLambert="<<flatPulse.TotalReceivedEnergyJ()<<" uniform="<<uniformPulse.TotalReceivedEnergyJ()
        <<" ratio="<<uniformPulse.TotalReceivedEnergyJ()/flatPulse.TotalReceivedEnergyJ()<<std::endl;
    Check("end to end uniform no double cosine",Near(uniformPulse.TotalReceivedEnergyJ()/flatPulse.TotalReceivedEnergyJ(),2));
    const auto phongPulse=Scan(std::make_shared<Model>(.5,phong),.5);
    Check("end to end Phong analytic angular ratio",Near(phongPulse.TotalReceivedEnergyJ()/flatPulse.TotalReceivedEnergyJ(),.34375));
    Check("automatic empirical OBJ conversion reaches pulse",Near(Scan(nullptr,.5).TotalReceivedEnergyJ(),phongPulse.TotalReceivedEnergyJ()));
    Check("arrival time unaffected",Near(phongPulse.returns.at(0).arrivalTimeSeconds,flatPulse.returns.at(0).arrivalTimeSeconds));
    const auto many=Scan(std::make_shared<Model>(.5,phong),.5,16);
    Check("MC sample count normalization preserved",many.returns.size()==16&&Near(many.TotalReceivedEnergyJ(),phongPulse.TotalReceivedEnergyJ()));
    LidarWaveform waveform({0,1e-9,256});
    Check("waveform energy conserved",Near(waveform.AccumulatePulse(many),many.TotalReceivedEnergyJ()));

    // Actual local asset diagnostics, without changing/downloading original data.
    const auto urban=std::filesystem::path(__FILE__).parent_path().parent_path().parent_path()/"models"/"helios_urban_static";
    if(std::filesystem::is_regular_file(urban/"vivaro.mtl")) {
        std::ifstream input(urban/"vivaro.mtl");std::map<std::string,int> ids;
        std::vector<tinyobj::material_t> source;std::string warning,error;
        tinyobj::LoadMtl(&ids,&source,&input,&warning,&error);
        Check("real vivaro has eighteen decoded materials",source.size()==18&&error.empty());
        const auto actual=ObjMaterialBinding::ConvertTable(source,defaults);
        Check("real vivaro first aluminum specularity initialized",Near(Empirical(actual,ids.at("15-alu"))->AngularResponse().Specularity(),.5));
        Check("real vivaro body specularity initialized",Near(Empirical(actual,ids.at("15-body"))->AngularResponse().Specularity(),1.7/3.1));
        Check("real vivaro chrome specularity initialized",Near(Empirical(actual,ids.at("15-chrome"))->AngularResponse().Specularity(),.625));
        Check("real vivaro commented mirror Ks stays zero",Empirical(actual,ids.at("15-mirror"))->AngularResponse().Kind()==HeliosEmpiricalKind::DirectionIndependent);
        Check("real vivaro plate is Lambert not retroreflection",Empirical(actual,ids.at("15-plate"))->AngularResponse().Kind()==HeliosEmpiricalKind::Lambert);
        bool allDefault=true;for(const auto& info:actual.info)allDefault &= info.source==ObjLidarBindingSource::HeliosDefault;
        Check("real vivaro all reflectances clearly assumed",allDefault);
    } else std::cout<<"[SKIP] HELIOS empirical optional real urban MTL absent\n";
}
