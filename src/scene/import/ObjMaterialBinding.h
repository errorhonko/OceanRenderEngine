#pragma once

#include "scene/import/ObjMeshLoader.h"
#include "reflection/materials/SurfaceMaterial.h"
#include "reflection/materials/DiffuseMaterial.h"
#include "reflection/lidar/LambertianLidarScattering.h"
#include "reflection/lidar/SpectralLambertianLidarScattering.h"
#include "reflection/lidar/HeliosEmpiricalLidarScattering.h"
#include "reflection/lidar/LidarScatteringFactory.h"
#include <charconv>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>

enum class ObjLidarAngularModel { Lambert, HeliosEmpirical };

struct ObjMaterialBindingOptions
{
    // Only named .txt spectra directly in this directory are accepted.
    std::filesystem::path spectraDirectory;
    // Names are local to the material table passed to ConvertTable, not global IDs.
    std::map<std::string, std::shared_ptr<const SurfaceMaterial>> overrides;
    std::shared_ptr<const LidarScatteringModel> missingLidarFallback;
    std::shared_ptr<const SurfaceMaterial> unassignedMaterial; // ID == -1 only.
    bool createDiffuseRenderMaterials = true; // RGB Kd -> diffuse BSDF, NEVER NIR rho.
    bool allowOpaqueApproximation = false; // Explicit opt-in for d/map_d/transmissive illum.
    ObjLidarAngularModel lidarAngularModel = ObjLidarAngularModel::Lambert;

    static ObjMaterialBindingOptions HeliosEmpiricalBaseline(std::filesystem::path spectraDirectory)
    {
        ObjMaterialBindingOptions options;
        options.spectraDirectory = std::move(spectraDirectory);
        options.lidarAngularModel = ObjLidarAngularModel::HeliosEmpirical;
        options.unassignedMaterial = std::make_shared<const SurfaceMaterial>(SurfaceMaterial{
            nullptr, std::make_shared<HeliosEmpiricalLidarScattering>(0.5,
                HeliosEmpiricalAngularResponse(std::array<double,3>{}, std::array<double,3>{}))});
        return options;
    }
};

enum class ObjLidarBindingSource { Override, ReflectancePercent, Spectrum, ExplicitFallback, HeliosDefault };
struct ObjMaterialBindingInfo
{
    std::string name;
    ObjLidarBindingSource source = ObjLidarBindingSource::ExplicitFallback;
    std::filesystem::path spectrumPath;
    std::vector<std::string> warnings;
    ObjLidarAngularModel angularModel = ObjLidarAngularModel::Lambert;
};
struct ObjMaterialBindingResult
{
    SurfaceMaterialBindings materials; // Same indices as reader.GetMaterials().
    std::vector<ObjMaterialBindingInfo> info;
};

// Adapter after tinyobj parsing. Not a general PBR/Phong material converter.
// HELIOS percent reflectance overrides spectra. Lambert is the unchanged default;
// empirical angular response/default rho require an explicit mode selection.
class ObjMaterialBinding
{
public:
    static ObjMaterialBindingResult ConvertTable(const std::vector<tinyobj::material_t>& source,
        const ObjMaterialBindingOptions& options = {})
    {
        ObjMaterialBindingResult result;
        if (options.lidarAngularModel != ObjLidarAngularModel::Lambert &&
            options.lidarAngularModel != ObjLidarAngularModel::HeliosEmpirical)
            throw std::invalid_argument("Unknown OBJ LiDAR angular model.");
        // Source data and resolved models have different cache keys: HELIOS
        // response depends on each MTL entry's Kd/Ks/Ns, not just the spectrum.
        std::map<std::filesystem::path, SpectralReflectanceCurve> spectra;
        std::map<std::filesystem::path, std::shared_ptr<const LidarScatteringModel>> lambertSpectra;
        for (const auto& material : source)
        {
            ObjMaterialBindingInfo info; info.name = material.name;
            if (const auto override = options.overrides.find(material.name); override != options.overrides.end())
            {
                if (!override->second) Fail(material.name, "explicit override is null");
                info.source = ObjLidarBindingSource::Override;
                result.materials.push_back(override->second); result.info.push_back(std::move(info));
                continue; // Explicit whole-model override is authoritative.
            }
            if (!std::isfinite(material.dissolve) || material.dissolve < 0 || material.dissolve > 1)
                Fail(material.name, "decoded MTL dissolve must be finite and in [0,1]");
            const bool transparent = material.dissolve != 1 || !material.alpha_texname.empty() ||
                material.illum == 4 || material.illum == 6 || material.illum == 7 || material.illum == 9;
            if (transparent)
            {
                if (!options.allowOpaqueApproximation)
                    Fail(material.name, "opacity/transmission is unsupported; supply an override or explicitly allow an opaque approximation");
                info.warnings.push_back("Opacity/transmission approximated as an opaque surface.");
            }
            std::shared_ptr<Material> render;
            if (options.createDiffuseRenderMaterials)
            {
                for (const auto kd : material.diffuse)
                    if (!std::isfinite(kd) || kd < 0 || kd > 1)
                        Fail(material.name, "decoded RGB Kd must be finite and in [0,1]");
                render = std::make_shared<DiffuseMaterial>(Spectrum(
                    material.diffuse[0], material.diffuse[1], material.diffuse[2]));
                if (!material.diffuse_texname.empty() || !material.bump_texname.empty() ||
                    !material.normal_texname.empty() || !material.displacement_texname.empty())
                    info.warnings.push_back("Render textures/normal/displacement maps are not loaded; using constant RGB Kd.");
                if (material.emission[0] != 0 || material.emission[1] != 0 || material.emission[2] != 0 ||
                    !material.emissive_texname.empty())
                    info.warnings.push_back("MTL emission is not converted to an area light.");
            }
            if (options.lidarAngularModel == ObjLidarAngularModel::Lambert &&
                (material.specular[0] != 0 || material.specular[1] != 0 || material.specular[2] != 0 ||
                material.metallic != 0 || material.roughness != 0))
                info.warnings.push_back("Ks/Ns/PBR parameters are not converted; automatic scattering is a diffuse baseline.");
            std::shared_ptr<const LidarScatteringModel> lidar;
            std::optional<LidarReflectionData> reflectionData;
            const auto reflectance = material.unknown_parameter.find("helios_reflectance");
            const auto spectrum = material.unknown_parameter.find("helios_spectra");
            if (reflectance != material.unknown_parameter.end())
            {
                const double percent = Number(reflectance->second, material.name);
                if (percent < 0 || percent > 100) Fail(material.name, "helios_reflectance must be in [0,100] percent");
                reflectionData = ConstantLidarReflectance{percent * .01};
                info.source = ObjLidarBindingSource::ReflectancePercent;
                if (percent > 0 && percent < 1)
                    info.warnings.push_back("helios_reflectance uses percent: this value is below 1%, not fractional albedo.");
                if (spectrum != material.unknown_parameter.end())
                    info.warnings.push_back("helios_reflectance overrides helios_spectra; the spectrum is not loaded.");
            }
            else if (spectrum != material.unknown_parameter.end())
            {
                const auto filename = SpectrumFilename(Text(spectrum->second), material.name);
                try
                {
                    const auto root = std::filesystem::canonical(options.spectraDirectory);
                    if (!std::filesystem::is_directory(root)) Fail(material.name, "spectraDirectory is not a directory");
                    const auto file = std::filesystem::canonical(root / filename);
                    if (!std::filesystem::equivalent(root, file.parent_path()))
                        Fail(material.name, "spectrum file resolves outside spectraDirectory");
                    info.spectrumPath = file;
                    auto found = spectra.find(file);
                    if (found == spectra.end())
                        found = spectra.emplace(file, SpectralReflectanceCurve::FromAsterFile(file)).first;
                    reflectionData = found->second;
                }
                catch (const std::exception& error)
                {
                    throw std::runtime_error("MTL material '" + material.name + "', spectrum '" +
                        filename + "': " + error.what());
                }
                info.source = ObjLidarBindingSource::Spectrum;
            }
            else
            {
                if (!options.missingLidarFallback && options.lidarAngularModel == ObjLidarAngularModel::HeliosEmpirical) {
                    reflectionData = ConstantLidarReflectance{0.5};
                    info.source = ObjLidarBindingSource::HeliosDefault;
                    info.warnings.push_back("HELIOS empirical mode: missing reflectance uses assumed rho=0.5, not measured data.");
                } else {
                    if (!options.missingLidarFallback)
                        Fail(material.name, "no helios_reflectance/helios_spectra and no explicit LiDAR fallback");
                    lidar = options.missingLidarFallback;
                    info.source = ObjLidarBindingSource::ExplicitFallback;
                    info.warnings.push_back("Missing LiDAR reflectance; using caller-supplied fallback, not RGB Kd.");
                }
            }
            if (reflectionData)
            {
                LidarResponseModel response = LambertLidarResponse{};
                if (options.lidarAngularModel == ObjLidarAngularModel::HeliosEmpirical)
                {
                    response = HeliosEmpiricalAngularResponse(
                        {material.diffuse[0], material.diffuse[1], material.diffuse[2]},
                        {material.specular[0], material.specular[1], material.specular[2]}, material.shininess);
                    info.angularModel = ObjLidarAngularModel::HeliosEmpirical;
                    info.warnings.push_back("Monostatic empirical angular shape, normalized energy scale; NOT a physical BRDF or raw HELIOS intensity reproduction.");
                    if (material.metallic != 0 || material.roughness != 0)
                        info.warnings.push_back("PBR metallic/roughness ignored by HELIOS empirical mode.");
                }
                const bool cacheLambert = info.source == ObjLidarBindingSource::Spectrum &&
                    options.lidarAngularModel == ObjLidarAngularModel::Lambert;
                if (cacheLambert)
                {
                    const auto cached = lambertSpectra.find(info.spectrumPath);
                    if (cached != lambertSpectra.end()) lidar = cached->second;
                }
                if (!lidar)
                {
                    lidar = LidarScatteringFactory::Create({std::move(*reflectionData), std::move(response)});
                    if (cacheLambert) lambertSpectra.emplace(info.spectrumPath, lidar);
                }
            }
            result.materials.push_back(std::make_shared<const SurfaceMaterial>(SurfaceMaterial{render, lidar}));
            result.info.push_back(std::move(info));
        }
        return result; // No externally visible partial result on failure.
    }

    static SurfaceMaterialBindings BindFaces(const ImportedMesh& source,
        const SurfaceMaterialBindings& materialTable,
        std::shared_ptr<const SurfaceMaterial> unassigned = nullptr)
    {
        if (!source.mesh || source.materialIds.size() != source.mesh->Triangles().size())
            throw std::invalid_argument("OBJ face material IDs must match a valid mesh's triangle count.");
        SurfaceMaterialBindings bindings; bindings.reserve(source.materialIds.size());
        for (const int id : source.materialIds)
        {
            if (id == -1)
            {
                if (!unassigned) Fail(source.name, "face has no material (ID -1); supply an explicit unassignedMaterial");
                bindings.push_back(unassigned);
            }
            else
            {
                if (id < 0 || static_cast<std::size_t>(id) >= materialTable.size())
                    throw std::out_of_range("OBJ face material ID is outside its own asset's material table.");
                if (!materialTable[id]) Fail(source.name, "material table entry is null");
                bindings.push_back(materialTable[id]);
            }
        }
        return bindings;
    }
private:
    [[noreturn]] static void Fail(const std::string& material, const std::string& reason)
    { throw std::invalid_argument("MTL material/mesh '" + material + "': " + reason); }
    static std::string Text(std::string value)
    {
        if (const auto comment = value.find('#'); comment != std::string::npos) value.resize(comment);
        const auto begin = value.find_first_not_of(" \t\r\n");
        if (begin == std::string::npos) return {};
        return value.substr(begin, value.find_last_not_of(" \t\r\n") - begin + 1);
    }
    static double Number(const std::string& text, const std::string& material)
    {
        auto value = Text(text);
        if (!value.empty() && value.front() == '+')
        {
            value.erase(0,1);
            if (value.empty() || value.front() == '+' || value.front() == '-') Fail(material, "invalid helios_reflectance number");
        }
        double result = 0;
        const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
        if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || !std::isfinite(result))
            Fail(material, "helios_reflectance must be one finite numeric percent value");
        return result;
    }
    static std::string SpectrumFilename(std::string name, const std::string& material)
    {
        if (name.ends_with(".txt")) name.resize(name.size()-4);
        if (name.empty()) Fail(material, "empty helios_spectra name");
        for (const unsigned char c : name)
            if (!(c >= 'a' && c <= 'z') && !(c >= 'A' && c <= 'Z') &&
                !(c >= '0' && c <= '9') && c != '_' && c != '-')
                Fail(material, "helios_spectra must be a simple library name, not a path");
        std::string upper = name;
        for (auto& c : upper) if (c >= 'a' && c <= 'z') c -= 'a' - 'A';
        if (upper == "CON" || upper == "PRN" || upper == "AUX" || upper == "NUL" ||
            (upper.size() == 4 && (upper.starts_with("COM") || upper.starts_with("LPT")) && upper[3] >= '1' && upper[3] <= '9'))
            Fail(material, "reserved spectrum filename");
        return name + ".txt";
    }
};
