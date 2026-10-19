#include "reflection/lidar/LidarScatteringFactory.h" // Public header must stand alone.
#include "core/spectrum/OpticalConstantsTableReader.h"
#include "scene/import/ObjMaterialBinding.h"
#include <iostream>
#include <limits>
#include <numbers>

namespace {
void Check(const std::string& name, bool value)
{
    if (!value) throw std::runtime_error("LiDAR material config: " + name);
    std::cout << "[PASS] material config " << name << '\n';
}
template<class E, class F> bool Rejects(F f)
{ try { f(); } catch (const E&) { return true; } return false; }
bool Near(double a, double b)
{ return std::isfinite(a) && std::isfinite(b) && std::abs(a-b) <= 1e-24+2e-6*std::max(std::abs(a),std::abs(b)); }
HitRecord Hit()
{ HitRecord hit{}; hit.normal={0,1,0};hit.geometricNormal={0,1,0};hit.dpdu={1,0,0};return hit; }
Vector3f Direction(double c)
{ return {float(std::sqrt(1-c*c)),float(c),0}; }
double Response(const LidarScatteringModel& model, double c, float wavelength)
{ const auto w=Direction(c);return model.EvaluateProjectedResponse(Hit(),w,w,wavelength); }
std::filesystem::path Root()
{ return std::filesystem::path(u8"" __FILE__).parent_path().parent_path().parent_path(); }
}

void RunLidarMaterialConfigAcceptanceTests()
{
    const auto fixture=std::filesystem::path(u8"" __FILE__).parent_path()/"fixtures"/"obj_materials";
    const auto curve=SpectralReflectanceCurve::FromAsterFile(fixture/"synthetic.txt");
    const HeliosEmpiricalAngularResponse angular({.2,.2,.2},{.6,.6,.6},4);
    const auto scalarLambert=LidarScatteringFactory::Create({ConstantLidarReflectance{.4},LambertLidarResponse{}});
    const auto scalarHelios=LidarScatteringFactory::Create({ConstantLidarReflectance{.4},angular});
    const auto spectralLambert=LidarScatteringFactory::Create({curve,LambertLidarResponse{}});
    const auto spectralHelios=LidarScatteringFactory::Create({curve,angular});
    const LambertianLidarScattering directScalar(.4);
    const HeliosEmpiricalLidarScattering directScalarHelios(.4,angular);
    const SpectralLambertianLidarScattering directSpectral(curve);
    const HeliosEmpiricalLidarScattering directSpectralHelios(curve,angular);
    for (float wavelength : {532.0f,1041.0f,1550.0f})
        for (double cosine : {1.0,.8,.5})
        {
            const auto suffix=std::to_string(wavelength)+"/"+std::to_string(cosine);
            Check("constant Lambert equivalence "+suffix,Near(Response(*scalarLambert,cosine,wavelength),Response(directScalar,cosine,wavelength)));
            Check("constant HELIOS equivalence "+suffix,Near(Response(*scalarHelios,cosine,wavelength),Response(directScalarHelios,cosine,wavelength)));
            Check("spectrum Lambert equivalence "+suffix,Near(Response(*spectralLambert,cosine,wavelength),Response(directSpectral,cosine,wavelength)));
            Check("spectrum HELIOS equivalence "+suffix,Near(Response(*spectralHelios,cosine,wavelength),Response(directSpectralHelios,cosine,wavelength)));
        }
    const auto spectral=std::dynamic_pointer_cast<const SpectralLambertianLidarScattering>(spectralLambert);
    Check("spectrum provenance retained",spectral && spectral->ReflectanceCurve().Metadata().sampleNumber=="OBJ-TEST");
    Check("spectral Lambert out of band rejects",Rejects<std::out_of_range>([&]{Response(*spectralLambert,.5,2000);}));
    Check("spectral HELIOS out of band rejects",Rejects<std::out_of_range>([&]{Response(*spectralHelios,.5,2000);}));
    Check("empirical response is not exposed as BRDF",Rejects<std::logic_error>([&]{spectralHelios->Evaluate(Hit(),{0,1,0},{0,1,0},532);}));
    Check("empirical response remains monostatic",Rejects<std::invalid_argument>([&]{spectralHelios->EvaluateProjectedResponse(Hit(),{0,1,0},{1,1,0},532);}));
    for (double invalid : {-1.0,1.1,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()})
    {
        Check("invalid constant Lambert rejects",Rejects<std::invalid_argument>([&]{LidarScatteringFactory::Create({ConstantLidarReflectance{invalid},LambertLidarResponse{}});}));
        Check("invalid constant HELIOS rejects",Rejects<std::invalid_argument>([&]{LidarScatteringFactory::Create({ConstantLidarReflectance{invalid},angular});}));
    }
    const auto nk=OpticalConstantsTableReader::FromFile(Root()/"data"/"materials"/"optical_constants"/"Al_Rakic1995_um.txt",
        OpticalConstantsWavelengthUnit::Micrometers,{"Al Rakic 1995","https://doi.org/10.1364/AO.34.004755","Roughness is an independent assumption"});
    const GGXConductorLidarResponse ggx{1.0,.2f,.3f};
    const auto metal=LidarScatteringFactory::Create({nk,ggx});
    const RoughConductorLidarScattering directMetal(1.0,nk,.2f,.3f);
    for (float wavelength : {905.0f,1550.0f})
        for (double cosine : {1.0,.8,.5})
            Check("optical constants GGX equivalence",Near(Response(*metal,cosine,wavelength),Response(directMetal,cosine,wavelength)));
    const auto conductor=std::dynamic_pointer_cast<const RoughConductorLidarScattering>(metal);
    Check("optical provenance retained",conductor && conductor->OpticalConstantsSpectrum()->Metadata().name=="Al Rakic 1995");
    Check("optical constants out of band rejects",Rejects<std::out_of_range>([&]{Response(*metal,.5,3000);}));
    Check("n/k plus Lambert rejects",Rejects<std::invalid_argument>([&]{LidarScatteringFactory::Create({nk,LambertLidarResponse{}});}));
    Check("n/k plus HELIOS rejects",Rejects<std::invalid_argument>([&]{LidarScatteringFactory::Create({nk,angular});}));
    Check("scalar rho plus GGX rejects",Rejects<std::invalid_argument>([&]{LidarScatteringFactory::Create({ConstantLidarReflectance{.4},ggx});}));
    Check("spectral rho plus GGX rejects",Rejects<std::invalid_argument>([&]{LidarScatteringFactory::Create({curve,ggx});}));
    Check("invalid alpha rejects",Rejects<std::invalid_argument>([&]{LidarScatteringFactory::Create({nk,GGXConductorLidarResponse{1,0,.3f}});}));
    Check("invalid incident index rejects",Rejects<std::invalid_argument>([&]{LidarScatteringFactory::Create({nk,GGXConductorLidarResponse{0,.2f,.3f}});}));

    // Same source spectrum, independent per-entry response. No per-hit I/O.
    tinyobj::material_t a{};a.name="a";a.dissolve=1;a.shininess=4;
    a.diffuse[0]=a.diffuse[1]=a.diffuse[2]=.2f;
    a.unknown_parameter["helios_spectra"]="synthetic";
    auto b=a;b.name="b";b.specular[0]=b.specular[1]=b.specular[2]=.6f;
    ObjMaterialBindingOptions options;options.spectraDirectory=fixture;options.createDiffuseRenderMaterials=false;
    auto table=ObjMaterialBinding::ConvertTable({a,b},options);
    Check("OBJ Lambert spectrum model sharing retained",table.materials[0]->lidarScattering==table.materials[1]->lidarScattering);
    options.lidarAngularModel=ObjLidarAngularModel::HeliosEmpirical;
    table=ObjMaterialBinding::ConvertTable({a,b},options);
    Check("OBJ empirical responses are per entry",table.materials[0]->lidarScattering!=table.materials[1]->lidarScattering);
    Check("OBJ empirical angular distinction retained",!Near(Response(*table.materials[0]->lidarScattering,.5,532),Response(*table.materials[1]->lidarScattering,.5,532)));
    options.overrides["a"]=std::make_shared<const SurfaceMaterial>(SurfaceMaterial{nullptr,metal});
    a.unknown_parameter["helios_spectra"]="intentionally_missing";
    table=ObjMaterialBinding::ConvertTable({a},options);
    Check("configured conductor uses existing authoritative override",table.materials[0]->lidarScattering==metal && table.info[0].source==ObjLidarBindingSource::Override);
}
