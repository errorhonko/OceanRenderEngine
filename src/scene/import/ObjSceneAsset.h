#pragma once

#include "scene/import/ObjMaterialBinding.h"
#include "scene/SceneBuilder.h"

// Owning import result: retains the source MTL table and its decoded metadata.
// Geometry import stays separate from engine-specific material conversion.
class ObjSceneAsset
{
public:
    static ObjSceneAsset FromReader(const tinyobj::ObjReader& reader,
        const ObjImportOptions& geometry = {}, const ObjMaterialBindingOptions& materials = {})
    {
        ObjSceneAsset result;
        result.meshes = ObjMeshLoader::Convert(reader, geometry, &result.geometryDiagnostics);
        result.sourceMaterials = reader.GetMaterials();
        result.parserWarning = reader.Warning();
        result.converted = ObjMaterialBinding::ConvertTable(result.sourceMaterials, materials);
        for (const auto& mesh : result.meshes)
            result.faceBindings.push_back(ObjMaterialBinding::BindFaces(
                mesh, result.converted.materials, materials.unassignedMaterial));
        return result;
    }
    static ObjSceneAsset Read(const std::filesystem::path& path,
        const ObjImportOptions& geometry = {}, const ObjMaterialBindingOptions& materials = {})
    {
        const auto reader = ObjMeshLoader::Read(path.string());
        auto result = FromReader(reader, geometry, materials);
        result.sourcePath = path;
        return result;
    }
    void AddToScene(SceneBuilder& builder, const SceneObjectPose& pose = {}) const
    {
        builder.AddMeshesWithSurfaceMaterials(meshes, faceBindings, pose);
    }
    const std::vector<ImportedMesh>& Meshes() const noexcept { return meshes; }
    const std::vector<tinyobj::material_t>& SourceMaterials() const noexcept { return sourceMaterials; }
    const ObjMaterialBindingResult& MaterialBindings() const noexcept { return converted; }
    const std::vector<SurfaceMaterialBindings>& FaceBindings() const noexcept { return faceBindings; }
    const ObjImportDiagnostics& GeometryDiagnostics() const noexcept { return geometryDiagnostics; }
    const std::string& ParserWarning() const noexcept { return parserWarning; }
    const std::filesystem::path& SourcePath() const noexcept { return sourcePath; }
private:
    std::vector<ImportedMesh> meshes;
    std::vector<tinyobj::material_t> sourceMaterials;
    ObjMaterialBindingResult converted;
    std::vector<SurfaceMaterialBindings> faceBindings;
    ObjImportDiagnostics geometryDiagnostics;
    std::string parserWarning;
    std::filesystem::path sourcePath;
};
