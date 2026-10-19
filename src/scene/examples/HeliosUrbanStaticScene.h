#pragma once
#include "scene/SceneBuilder.h"
#include "scene/import/ObjMaterialBinding.h"
#include "scene/examples/HeliosUrbanStaticSceneParts.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

enum class HeliosUrbanGroundingMode { SourcePlane, TerrainContact };
struct HeliosUrbanPlacementOptions
{
    // SourcePlane preserves the fixed HELIOS recipe, including ungrounded cars.
    // TerrainContact also grounds all nine frozen vehicle placements by tires.
    HeliosUrbanGroundingMode groundingMode = HeliosUrbanGroundingMode::SourcePlane;
};

struct HeliosUrbanGroundPlane
{
    std::array<double,3> center{}, normal{0,1,0};
    Bounds3f bounds;
    double Height(double x, double z) const
    {
        return center[1]-(normal[0]*(x-center[0])+normal[2]*(z-center[2]))/normal[1];
    }
};
struct HeliosUrbanAssetInfo
{
    std::string file, warning;
    std::size_t meshGroups=0, materialCount=0;
    ObjImportDiagnostics triangles;
    Bounds3f sourceEngineBounds;
    std::vector<tinyobj::material_t> sourceMaterials;
    std::vector<ObjMaterialBindingInfo> materialBindings; // Populated by the opt-in material load.
};
struct HeliosUrbanPartInfo
{
    std::size_t sourcePartIndex=0, firstSceneObject=0, sceneObjectCount=0, triangleCount=0;
    std::string file;
    bool originallyDynamic=false, onGround=false;
    bool terrainGrounded=false, groundingUsesTires=false;
    std::size_t terrainSupportPointCount=0;
    double groundShiftMeters=0;
    Vector3f groundingReferenceBefore, groundingReferenceAfter;
    Bounds3f bounds;
};
struct HeliosUrbanStaticSceneResult
{
    std::shared_ptr<StaticScene> scene;
    std::vector<HeliosUrbanAssetInfo> assets;
    std::vector<HeliosUrbanPartInfo> parts;
    HeliosUrbanGroundPlane groundPlane;
    HeliosUrbanPlacementOptions placement;
    // Unique OBJ source count versus the contribution across repeated parts.
    std::size_t triangleCount=0, skippedSourceTriangles=0, skippedInstancedTriangles=0;
};

namespace HeliosUrbanStaticDetail {
using Matrix=std::array<std::array<double,3>,3>;
inline Matrix Identity() { return {{{1,0,0},{0,1,0},{0,0,1}}}; }
inline Matrix Product(const Matrix& a,const Matrix& b)
{
    Matrix r{};
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)for(int k=0;k<3;++k)r[i][j]+=a[i][k]*b[k][j];
    return r;
}
inline Matrix Rotation(const HeliosUrbanPartSpec& part)
{
    Matrix rotation=Identity();
    for(std::size_t i=0;i<part.rotationCount;++i) {
        const auto& r=part.rotations[i];
        std::array<double,3> axis{};
        switch(r.sourceAxis) {case 'x':axis={1,0,0};break;case 'y':axis={0,0,-1};break;case 'z':axis={0,1,0};break;default:throw std::invalid_argument("Urban rotation axis is invalid.");}
        const double angle=r.degrees*3.14159265358979323846/180, c=std::cos(angle),s=std::sin(angle),d=1-c;
        const double x=axis[0],y=axis[1],z=axis[2];
        const Matrix step{{{c+x*x*d,x*y*d-z*s,x*z*d+y*s},
            {y*x*d+z*s,c+y*y*d,y*z*d-x*s},{z*x*d-y*s,z*y*d+x*s,c+z*z*d}}};
        rotation=Product(step,rotation); // Global axes: later XML rotations act last.
    }
    return rotation;
}
inline std::array<double,3> Apply(const Matrix& r,const Vector3f& v)
{
    return {r[0][0]*v.x+r[0][1]*v.y+r[0][2]*v.z,
        r[1][0]*v.x+r[1][1]*v.y+r[1][2]*v.z,r[2][0]*v.x+r[2][1]*v.y+r[2][2]*v.z};
}
inline Vector3f FloatPoint(const std::array<double,3>& p)
{
    for(double value:p)if(!std::isfinite(value)||std::abs(value)>std::numeric_limits<float>::max())
        throw std::overflow_error("Urban placement exceeds finite float geometry.");
    return {float(p[0]),float(p[1]),float(p[2])};
}
inline Bounds3f Bounds(const std::vector<ImportedMesh>& meshes)
{
    Bounds3f bounds;
    for(const auto& mesh:meshes)for(const auto& v:mesh.mesh->Vertices())bounds=Union(bounds,v.position);
    return bounds;
}
// Orthogonal least-squares plane, using triangle-corner multiplicities as the
// upstream grounding algorithm does. Covariance eigenvector via 3x3 Jacobi;
// no external linear-algebra dependency. This is NOT a terrain ray hit.
inline HeliosUrbanGroundPlane FitGroundPlane(const std::vector<ImportedMesh>& meshes)
{
    HeliosUrbanGroundPlane plane;
    std::size_t count=0;
    Matrix covariance{};
    for(const auto& mesh:meshes)for(const auto& triangle:mesh.mesh->Triangles())for(auto index:triangle) {
        const auto p=mesh.mesh->Vertices()[index].position;
        plane.bounds=Union(plane.bounds,p);
        const std::array<double,3> values{p.x,p.y,p.z};
        std::array<double,3> delta{};
        ++count;
        for(int i=0;i<3;++i) {delta[i]=values[i]-plane.center[i];plane.center[i]+=delta[i]/double(count);}
        for(int i=0;i<3;++i)for(int j=0;j<3;++j)covariance[i][j]+=delta[i]*(values[j]-plane.center[j]);
    }
    if(count<3)throw std::invalid_argument("Urban ground plane needs nonempty triangle geometry.");
    for(int i=0;i<3;++i)for(int j=0;j<i;++j)covariance[i][j]=covariance[j][i]=(covariance[i][j]+covariance[j][i])*0.5;
    Matrix vectors=Identity();
    for(int iteration=0;iteration<64;++iteration) {
        int p=0,q=1;
        for(const auto ij:std::array<std::array<int,2>,3>{{{0,1},{0,2},{1,2}}})
            if(std::abs(covariance[ij[0]][ij[1]])>std::abs(covariance[p][q])){p=ij[0];q=ij[1];}
        const double scale=std::max({std::abs(covariance[0][0]),std::abs(covariance[1][1]),std::abs(covariance[2][2])});
        if(std::abs(covariance[p][q])<=std::max(1e-30,scale*1e-14))break;
        const double tau=(covariance[q][q]-covariance[p][p])/(2*covariance[p][q]);
        const double t=(tau>=0?1.0:-1.0)/(std::abs(tau)+std::hypot(1.0,tau));
        const double c=1/std::sqrt(1+t*t),s=t*c,a=covariance[p][q];
        covariance[p][p]-=t*a;covariance[q][q]+=t*a;covariance[p][q]=covariance[q][p]=0;
        for(int k=0;k<3;++k) {
            if(k!=p&&k!=q) {const double kp=covariance[k][p],kq=covariance[k][q];covariance[k][p]=covariance[p][k]=c*kp-s*kq;covariance[k][q]=covariance[q][k]=s*kp+c*kq;}
            const double vp=vectors[k][p],vq=vectors[k][q];vectors[k][p]=c*vp-s*vq;vectors[k][q]=s*vp+c*vq;
        }
    }
    std::array<int,3> order{0,1,2};
    std::sort(order.begin(),order.end(),[&](int a,int b){return covariance[a][a]<covariance[b][b];});
    if(!(covariance[order[1]][order[1]]>std::max(1e-30,covariance[order[2]][order[2]]*1e-12)))
        throw std::invalid_argument("Urban ground vertices do not span a plane.");
    for(int i=0;i<3;++i)plane.normal[i]=vectors[i][order[0]];
    if(plane.normal[1]<0)for(auto& v:plane.normal)v=-v;
    if(plane.normal[1]<1e-6)throw std::invalid_argument("Urban ground plane is nearly vertical.");
    return plane;
}
inline double TerrainHeight(const Hittable& terrain,const Bounds3f& bounds,const Vector3f& point)
{
    const std::array<double,3> p{point.x,point.y,point.z},lo{bounds.pMin.x,bounds.pMin.y,bounds.pMin.z},hi{bounds.pMax.x,bounds.pMax.y,bounds.pMax.z};
    for(int i=0;i<3;++i)
        if(!std::isfinite(p[i])||!std::isfinite(lo[i])||!std::isfinite(hi[i])||lo[i]>hi[i])
            throw std::invalid_argument("Terrain contact needs finite points and nonempty bounds.");
    if(point.x<bounds.pMin.x||point.x>bounds.pMax.x||point.z<bounds.pMin.z||point.z>bounds.pMax.z)
        throw std::invalid_argument("Terrain contact point lies outside terrain bounds.");
    const double span=double(bounds.pMax.y)-bounds.pMin.y;
    const double top=double(bounds.pMax.y)+std::max(1.0,span*0.01);
    const double bottom=double(bounds.pMin.y)-std::max(1.0,span*0.01);
    if(!std::isfinite(top)||top>std::numeric_limits<float>::max()||top-bottom>std::numeric_limits<float>::max())
        throw std::overflow_error("Terrain contact ray exceeds finite float geometry.");
    HitRecord hit;
    if(!terrain.hit(Ray({point.x,float(top),point.z},{0,-1,0}),0,float(top-bottom),hit))
        throw std::invalid_argument("No terrain surface below contact point; no plane fallback is used.");
    if(!std::isfinite(hit.point.y))throw std::domain_error("Terrain returned a nonfinite height.");
    return hit.point.y; // Highest Terrain surface; never query vehicles/buildings.
}

struct TerrainContactShift { double meters=0; std::size_t reference=0; };
inline TerrainContactShift ContactShift(const std::vector<Vector3f>& supports,
    const Hittable& terrain,const Bounds3f& terrainBounds)
{
    if(supports.empty())throw std::invalid_argument("Terrain contact requires explicit support points.");
    TerrainContactShift result{-std::numeric_limits<double>::infinity(),0};
    for(std::size_t i=0;i<supports.size();++i) {
        const double needed=TerrainHeight(terrain,terrainBounds,supports[i])-supports[i].y;
        if(needed>result.meters)result={needed,i};
    }
    return result; // Translation-only: all sampled supports nonpenetrating, one in contact.
}

// This preset knows vivaro's tire material; this is not a universal wheel detector.
inline std::vector<Vector3f> TireSupportPoints(const std::vector<ImportedMesh>& source,
    int tireMaterialId,const Matrix& rotation,double scale,const std::array<double,3>& offset)
{
    if(tireMaterialId<0)throw std::invalid_argument("Vehicle contact requires the explicit 15-tire material.");
    std::vector<Vector3f> supports;
    for(const auto& mesh:source) {
        if(!mesh.mesh||mesh.materialIds.size()!=mesh.mesh->Triangles().size())
            throw std::invalid_argument("Vehicle contact needs valid per-face material IDs.");
        std::vector<bool> used(mesh.mesh->Vertices().size(),false);
        for(std::size_t t=0;t<mesh.mesh->Triangles().size();++t) {
            if(mesh.materialIds[t]!=tireMaterialId)continue;
            for(auto index:mesh.mesh->Triangles()[t]) {
                if(used[index])continue;
                used[index]=true;
                auto p=Apply(rotation,mesh.mesh->Vertices()[index].position);
                for(int j=0;j<3;++j)p[j]=p[j]*scale+offset[j];
                supports.push_back(FloatPoint(p));
            }
        }
    }
    if(supports.empty())throw std::invalid_argument("Vehicle has no referenced tire vertices.");
    return supports;
}

inline std::vector<ImportedMesh> Place(const std::vector<ImportedMesh>& source,
    const HeliosUrbanPartSpec& spec,std::size_t partIndex,const HeliosUrbanGroundPlane& plane,HeliosUrbanPartInfo& info,
    const HeliosUrbanPlacementOptions& placement={},const Hittable* terrain=nullptr,
    const Bounds3f* terrainBounds=nullptr,int tireMaterialId=-1)
{
    if(placement.groundingMode!=HeliosUrbanGroundingMode::SourcePlane&&placement.groundingMode!=HeliosUrbanGroundingMode::TerrainContact)
        throw std::invalid_argument("Unknown urban grounding mode.");
    const auto rotation=Rotation(spec);
    const std::array<double,3> offset{spec.sourceOffset[0],spec.sourceOffset[2],-spec.sourceOffset[1]};
    std::array<double,3> lowest{};bool first=true;
    for(const auto& mesh:source)for(const auto& triangle:mesh.mesh->Triangles())for(auto index:triangle) {
        auto p=Apply(rotation,mesh.mesh->Vertices()[index].position);
        for(int i=0;i<3;++i)p[i]=p[i]*spec.scale+offset[i];
        if(first||p[1]<lowest[1]){lowest=p;first=false;}
    }
    if(first)throw std::invalid_argument("Urban placement requires referenced triangle vertices.");
    HeliosUrbanPartInfo staged;
    staged.sourcePartIndex=partIndex;staged.file=spec.file;staged.originallyDynamic=spec.originallyDynamic;staged.onGround=spec.onGround;
    const bool vehicle=std::string(spec.file).starts_with("vivaro");
    const bool terrainContact=placement.groundingMode==HeliosUrbanGroundingMode::TerrainContact&&(spec.onGround||vehicle);
    if(terrainContact) {
        if(!terrain||!terrainBounds)throw std::invalid_argument("TerrainContact placement requires an actual terrain aggregate.");
        const auto supports=vehicle?TireSupportPoints(source,tireMaterialId,rotation,spec.scale,offset):std::vector<Vector3f>{FloatPoint(lowest)};
        const auto shift=ContactShift(supports,*terrain,*terrainBounds);
        staged.groundShiftMeters=shift.meters;staged.terrainGrounded=true;staged.groundingUsesTires=vehicle;
        staged.terrainSupportPointCount=supports.size();
        lowest={supports[shift.reference].x,supports[shift.reference].y,supports[shift.reference].z};
    } else if(spec.onGround) {
        if(lowest[0]<plane.bounds.pMin.x||lowest[0]>plane.bounds.pMax.x||lowest[2]<plane.bounds.pMin.z||lowest[2]>plane.bounds.pMax.z)
            throw std::invalid_argument("Urban grounding reference lies outside Terrain bounds.");
        staged.groundShiftMeters=plane.Height(lowest[0],lowest[2])-lowest[1];
    }
    staged.groundingReferenceBefore=FloatPoint(lowest);
    lowest[1]+=staged.groundShiftMeters;staged.groundingReferenceAfter=FloatPoint(lowest);
    auto result=source;
    for(auto& mesh:result) {
        auto vertices=mesh.mesh->Vertices();
        for(auto& vertex:vertices) {
            auto p=Apply(rotation,vertex.position);
            for(int i=0;i<3;++i)p[i]=p[i]*spec.scale+offset[i];
            p[1]+=staged.groundShiftMeters;vertex.position=FloatPoint(p);
            vertex.normal=FloatPoint(Apply(rotation,vertex.normal));
            staged.bounds=Union(staged.bounds,vertex.position);
        }
        mesh.name="urban_part_"+std::to_string(partIndex)+":"+spec.file+":"+mesh.name;
        staged.triangleCount+=mesh.mesh->Triangles().size();
        mesh.mesh=std::make_shared<TriangleMesh>(std::move(vertices),mesh.mesh->Triangles());
    }
    info=std::move(staged); // Publish diagnostics only after successful placement.
    return result;
}
}

// Dataset-specific static preset, not a general HELIOS XML loader. Preserve the
// source layout (metres assumed), rotate Z-up to Y-up, apply scale*R*p+offset,
// reproduce onGround=1 with a terrain fitted plane, freeze BEFORE any dmotion.
// No per-asset recentering, scene-wide shift or animation. Material mapping is opt-in;
// semantic labels are not propagated. Generic SceneBuilder and OBJ strict defaults stay intact.
class HeliosUrbanStaticScene
{
public:
    static bool AssetsPresent(const std::filesystem::path& directory)
    {
        for(const auto& part:HeliosUrbanStaticParts)
            if(!std::filesystem::is_regular_file(directory/part.file))return false;
        for(const char* file:{"Building.mtl","Terrain.mtl","vivaro.mtl","small_maple.mtl","white_birch.mtl"})
            if(!std::filesystem::is_regular_file(directory/file))return false;
        return true;
    }
    static HeliosUrbanStaticSceneResult Load(const std::filesystem::path& directory)
    {
        return LoadInternal(directory, nullptr); // Preserve the geometry-only preset.
    }
    static HeliosUrbanStaticSceneResult Load(const std::filesystem::path& directory,
        const ObjMaterialBindingOptions& materials)
    {
        return LoadInternal(directory, &materials);
    }
    static HeliosUrbanStaticSceneResult Load(const std::filesystem::path& directory,
        const ObjMaterialBindingOptions& materials,const HeliosUrbanPlacementOptions& placement)
    {
        return LoadInternal(directory,&materials,placement);
    }
private:
    static HeliosUrbanStaticSceneResult LoadInternal(const std::filesystem::path& directory,
        const ObjMaterialBindingOptions* materialOptions,const HeliosUrbanPlacementOptions& placement={})
    {
        if(placement.groundingMode!=HeliosUrbanGroundingMode::SourcePlane&&placement.groundingMode!=HeliosUrbanGroundingMode::TerrainContact)
            throw std::invalid_argument("Unknown urban grounding mode.");
        if(!AssetsPresent(directory))throw std::runtime_error("Missing urban assets; run models/helios_urban_static/FetchUrbanStatic.ps1.");
        HeliosUrbanStaticSceneResult result;
        result.placement=placement;
        std::map<std::string,std::vector<ImportedMesh>> cache;
        std::map<std::string,ObjMaterialBindingResult> materialCache;
        std::map<std::string,int> tireMaterialIds;
        ObjImportOptions options;
        options.sourceAxesInEngine={ObjAxisDirection::PositiveX,ObjAxisDirection::NegativeZ,ObjAxisDirection::PositiveY};
        options.skipDegenerateTriangles=true; // Real vehicles have six zero-area faces each.
        for(const auto& spec:HeliosUrbanStaticParts) {
            if(cache.contains(spec.file))continue;
            const auto reader=ObjMeshLoader::Read((directory/spec.file).string());
            HeliosUrbanAssetInfo info;info.file=spec.file;info.warning=reader.Warning();info.materialCount=reader.GetMaterials().size();
            info.sourceMaterials=reader.GetMaterials();
            if(std::string(spec.file).starts_with("vivaro")) {
                int tire=-1;
                for(std::size_t i=0;i<info.sourceMaterials.size();++i)
                    if(info.sourceMaterials[i].name=="15-tire")tire=int(i);
                tireMaterialIds[spec.file]=tire;
            }
            if(materialOptions) {
                auto converted=ObjMaterialBinding::ConvertTable(info.sourceMaterials,*materialOptions);
                info.materialBindings=converted.info;materialCache.emplace(spec.file,std::move(converted));
            }
            auto meshes=ObjMeshLoader::Convert(reader,options,&info.triangles);
            info.meshGroups=meshes.size();info.sourceEngineBounds=HeliosUrbanStaticDetail::Bounds(meshes);
            result.skippedSourceTriangles+=info.triangles.skippedDegenerateTriangleCount;
            result.assets.push_back(info);cache.emplace(spec.file,std::move(meshes));
        }
        result.groundPlane=HeliosUrbanStaticDetail::FitGroundPlane(cache.at("Terrain.obj"));
        std::shared_ptr<StaticScene> terrain;
        Bounds3f terrainBounds;
        if(placement.groundingMode==HeliosUrbanGroundingMode::TerrainContact) {
            SceneBuilder terrainBuilder;terrainBuilder.AddMeshes(cache.at("Terrain.obj"));
            terrain=terrainBuilder.Build();terrainBounds=terrain->Bounds();
        }
        SceneBuilder builder;
        for(std::size_t i=0;i<HeliosUrbanStaticParts.size();++i) {
            const auto& spec=HeliosUrbanStaticParts[i];HeliosUrbanPartInfo info;
            auto placed=HeliosUrbanStaticDetail::Place(cache.at(spec.file),spec,i,result.groundPlane,info,
                placement,terrain.get(),terrain?&terrainBounds:nullptr,
                tireMaterialIds.contains(spec.file)?tireMaterialIds.at(spec.file):-1);
            info.firstSceneObject=builder.ObjectCount();
            if(materialOptions) {
                std::vector<SurfaceMaterialBindings> perGroup;
                for(const auto& mesh:placed) perGroup.push_back(ObjMaterialBinding::BindFaces(
                    mesh,materialCache.at(spec.file).materials,materialOptions->unassignedMaterial));
                builder.AddMeshesWithSurfaceMaterials(placed,perGroup);
            } else builder.AddMeshes(placed);
            info.sceneObjectCount=builder.ObjectCount()-info.firstSceneObject;
            const auto asset=std::find_if(result.assets.begin(),result.assets.end(),[&](const auto& a){return a.file==spec.file;});
            result.skippedInstancedTriangles+=asset->triangles.skippedDegenerateTriangleCount;
            result.triangleCount+=info.triangleCount;result.parts.push_back(info);
        }
        result.scene=builder.Build();return result;
    }
};
