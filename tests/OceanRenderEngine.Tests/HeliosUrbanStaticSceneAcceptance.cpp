#include "scene/examples/HeliosUrbanStaticScene.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <set>
#include <stdexcept>
#include <string>

namespace {
void Check(const char* name,bool value)
{
    if(!value){std::cerr<<"[FAIL] "<<name<<'\n';throw std::runtime_error(name);}
    std::cout<<"[PASS] "<<name<<'\n';
}
template<class Function> bool Rejects(Function function)
{
    try{function();}catch(const std::exception&){return true;}return false;
}
bool Near(const Vector3f& a,const Vector3f& b,float tolerance=1e-5f){return (a-b).norm()<tolerance;}
std::filesystem::path Assets()
{
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path()/"models"/"helios_urban_static";
}
std::vector<ImportedMesh> PlaneMesh()
{
    std::vector<MeshVertex> vertices;
    for(const auto xz:std::array<std::array<float,2>,4>{{{-10,-10},{10,-10},{10,10},{-10,10}}})
        vertices.push_back({{xz[0],2+0.2f*xz[0]-0.1f*xz[1],xz[1]},{0,1,0},{0.3f,0.7f,0}});
    return {ImportedMesh{"plane",std::make_shared<TriangleMesh>(std::move(vertices),std::vector<TriangleIndices>{{0,2,1},{0,3,2}}),{-1,-1}}};
}
void TerrainContactTests()
{
    using namespace HeliosUrbanStaticDetail;
    SceneBuilder builder;builder.AddMeshes(PlaneMesh());const auto terrain=builder.Build();
    const auto bounds=terrain->Bounds();
    Check("terrain grounding queries actual sloped surface",std::abs(TerrainHeight(*terrain,bounds,{3,100,4})-2.2)<1e-5);
    Check("terrain height does not depend on object altitude",std::abs(TerrainHeight(*terrain,bounds,{3,-100,4})-2.2)<1e-5);
    Check("terrain grounding rejects out of footprint samples",Rejects([&]{TerrainHeight(*terrain,bounds,{100,2,0});}));
    Check("terrain grounding rejects nonfinite samples",Rejects([&]{TerrainHeight(*terrain,bounds,{0,std::numeric_limits<float>::quiet_NaN(),0});}));
    SceneBuilder emptyBuilder;const auto empty=emptyBuilder.Build();
    Check("terrain grounding does not silently fall back across a hole",Rejects([&]{TerrainHeight(*empty,bounds,{0,0,0});}));
    Check("terrain grounding rejects empty support set",Rejects([&]{ContactShift({},*terrain,bounds);}));
    const auto lower=ContactShift({{0,3,0}},*terrain,bounds);
    const auto raise=ContactShift({{0,0,0}},*terrain,bounds);
    Check("terrain contact lowers a floating support",std::abs(lower.meters+1)<1e-6);
    Check("terrain contact raises an embedded support",std::abs(raise.meters-2)<1e-6);
    const std::vector<Vector3f> supports{{0,2,0},{5,2.5f,0}};
    const auto shift=ContactShift(supports,*terrain,bounds);
    Check("terrain support constraint need not be the globally lowest vertex",shift.reference==1 && std::abs(shift.meters-.5)<1e-5);
    bool nonpenetrating=true;double minGap=1e10;
    for(auto p:supports){p.y+=float(shift.meters);const double gap=p.y-TerrainHeight(*terrain,bounds,p);nonpenetrating &= gap>=-1e-5;minGap=std::min(minGap,gap);}
    Check("terrain max clearance shift keeps all sampled supports above ground",nonpenetrating);
    Check("terrain max clearance shift has a sampled contact",std::abs(minGap)<1e-5);

    ImportedMesh object{"support",std::make_shared<TriangleMesh>(std::vector<MeshVertex>{
        {{0,10,0},{0,1,0},{.3f,.7f,0}},{{.1f,10,0},{0,1,0},{}},{{0,10,.1f},{0,1,0},{}}},std::vector<TriangleIndices>{{0,2,1}}),{3}};
    const auto tire=TireSupportPoints({object},3,Identity(),2,{1,2,3});
    Check("tire support extraction applies original rotation scale and offset",tire.size()==3&&Near(tire[0],{1,22,3}));
    Check("tire support extraction rejects missing tire declaration",Rejects([&]{TireSupportPoints({object},-1,Identity(),1,{0,0,0});}));
    Check("tire support extraction rejects unmatched tire faces",Rejects([&]{TireSupportPoints({object},4,Identity(),1,{0,0,0});}));
    auto invalid=object;invalid.materialIds.clear();
    Check("tire support extraction rejects invalid per face mappings",Rejects([&]{TireSupportPoints({invalid},3,Identity(),1,{0,0,0});}));
    HeliosUrbanPlacementOptions contact{HeliosUrbanGroundingMode::TerrainContact};
    HeliosUrbanPartSpec spec{"vivaro.test",1,{0,0,0},{},0,false,true};
    const auto plane=FitGroundPlane(PlaneMesh());HeliosUrbanPartInfo info;
    auto placed=Place({object},spec,0,plane,info,contact,terrain.get(),&bounds,3);
    Check("terrain mode also grounds frozen vehicles without source onGround",info.terrainGrounded&&info.groundingUsesTires&&!info.onGround&&info.terrainSupportPointCount==3);
    Check("terrain mode translates vertically only",Near(placed[0].mesh->Vertices()[0].position,{0,2.02f,0},1e-4f));
    Check("terrain mode preserves normals UV and source vertices",Near(placed[0].mesh->Vertices()[0].normal,{0,1,0})&&Near(placed[0].mesh->Vertices()[0].uv,{.3f,.7f,0})&&object.mesh->Vertices()[0].position.y==10);
    HeliosUrbanPartInfo legacy;
    const auto unchanged=Place({object},spec,0,plane,legacy);
    Check("default plane mode preserves source ungrounded vehicles",!legacy.terrainGrounded&&legacy.groundShiftMeters==0&&unchanged[0].mesh->Vertices()[0].position.y==10);
    Check("terrain mode requires explicit terrain geometry",Rejects([&]{Place({object},spec,0,plane,info,contact);}));
    Check("terrain mode never guesses missing vehicle tire material",Rejects([&]{Place({object},spec,0,plane,info,contact,terrain.get(),&bounds);}));
    auto unknown=contact;unknown.groundingMode=static_cast<HeliosUrbanGroundingMode>(100);
    Check("urban placement rejects unknown grounding mode",Rejects([&]{Place({object},spec,0,plane,info,unknown);}));

    const ImportedMesh bump{"bump",std::make_shared<TriangleMesh>(std::vector<MeshVertex>{
        {{-10,0,-10},{},{}},{{10,0,-10},{},{}},{{10,0,10},{},{}},{{-10,0,10},{},{}},{{0,5,0},{},{}}},
        std::vector<TriangleIndices>{{0,4,1},{1,4,2},{2,4,3},{3,4,0}}),{-1,-1,-1,-1}};
    SceneBuilder bumpBuilder;bumpBuilder.AddMesh(bump);const auto bumpTerrain=bumpBuilder.Build();
    const auto bumpBounds=bumpTerrain->Bounds();const auto bumpPlane=FitGroundPlane({bump});
    Check("nonplanar contact queries the bump not its best fitting plane",std::abs(TerrainHeight(*bumpTerrain,bumpBounds,{0,0,0})-5)<1e-5&&std::abs(bumpPlane.Height(0,0)-5)>1);
    spec.file="tree.test";spec.onGround=true;spec.originallyDynamic=false;
    placed=Place({object},spec,0,bumpPlane,info,contact,bumpTerrain.get(),&bumpBounds);
    Check("nonvehicle source onGround uses actual terrain at its base",info.terrainGrounded&&!info.groundingUsesTires&&std::abs(info.groundingReferenceAfter.y-5)<1e-5);
    Check("nonvehicle terrain contact uses an explicit single base support",info.terrainSupportPointCount==1);
}

void RealTerrainContactTests()
{
    const auto diffuse=std::make_shared<LambertianLidarScattering>(.5);
    const auto material=std::make_shared<const SurfaceMaterial>(SurfaceMaterial{nullptr,diffuse});
    ObjMaterialBindingOptions materials;materials.createDiffuseRenderMaterials=false;
    materials.missingLidarFallback=diffuse;materials.unassignedMaterial=material;materials.overrides["terrain"]=material;
    const auto corrected=HeliosUrbanStaticScene::Load(Assets(),materials,{HeliosUrbanGroundingMode::TerrainContact});
    Check("real terrain placement retains original scene scale and triangle count",corrected.triangleCount==2445341&&corrected.parts.size()==42&&corrected.scene->ObjectCount()==53);
    ObjImportOptions options;options.sourceAxesInEngine={ObjAxisDirection::PositiveX,ObjAxisDirection::NegativeZ,ObjAxisDirection::PositiveY};options.skipDegenerateTriangles=true;
    const auto groundMeshes=ObjMeshLoader::Convert(ObjMeshLoader::Read((Assets()/"Terrain.obj").string()),options);
    SceneBuilder builder;builder.AddMeshes(groundMeshes);const auto terrain=builder.Build();const auto bounds=terrain->Bounds();
    std::size_t vehicles=0,trees=0;
    bool allSamples=true,oneContact=true,noHorizontalChange=true,materialRetained=true;
    for(const auto& part:corrected.parts) {
        const auto& spec=HeliosUrbanStaticParts[part.sourcePartIndex];
        if(!part.terrainGrounded)continue;
        if(!part.groundingUsesTires){++trees;continue;}
        ++vehicles;
        const auto reader=ObjMeshLoader::Read((Assets()/part.file).string());
        const auto meshes=ObjMeshLoader::Convert(reader,options);
        int tire=-1;for(std::size_t i=0;i<reader.GetMaterials().size();++i)if(reader.GetMaterials()[i].name=="15-tire")tire=int(i);
        const auto supports=HeliosUrbanStaticDetail::TireSupportPoints(meshes,tire,HeliosUrbanStaticDetail::Rotation(spec),spec.scale,{spec.sourceOffset[0],spec.sourceOffset[2],-spec.sourceOffset[1]});
        double minimum=1e10;
        for(auto p:supports){p.y=float(double(p.y)+part.groundShiftMeters);const double gap=p.y-HeliosUrbanStaticDetail::TerrainHeight(*terrain,bounds,p);minimum=std::min(minimum,gap);allSamples &= gap>=-2e-5;}
        oneContact &= std::abs(minimum)<2e-5;
        noHorizontalChange &= part.groundingReferenceBefore.x==part.groundingReferenceAfter.x&&part.groundingReferenceBefore.z==part.groundingReferenceAfter.z;
        const auto p=part.groundingReferenceAfter;HitRecord hit;
        materialRetained &= corrected.scene->hit(Ray({p.x,p.y+100,p.z},{0,-1,0}),1e-4f,200,hit)&&bool(hit.lidarScattering);
    }
    Check("real terrain mode grounds all nine vehicles including frozen dynamic cars",vehicles==9&&corrected.parts[0].terrainGrounded&&corrected.parts[1].terrainGrounded);
    Check("real terrain mode also grounds eleven source flagged trees",trees==11);
    Check("real tire vertices do not penetrate sampled terrain",allSamples);
    Check("each real vehicle has at least one sampled terrain contact",oneContact);
    Check("real terrain contact preserves horizontal position",noHorizontalChange);
    Check("real terrain placement retains per surface LiDAR material binding",materialRetained);
    Check("previous floating right vehicle moves down by about half a metre",corrected.parts[0].groundShiftMeters<-.53&&corrected.parts[0].groundShiftMeters>-.54);
    Check("previous buried left vehicle no longer targets global plane",corrected.parts[38].terrainGrounded&&corrected.parts[38].groundShiftMeters>-.2);
}

void MathTests()
{
    const auto mesh=PlaneMesh();const auto plane=HeliosUrbanStaticDetail::FitGroundPlane(mesh);
    bool residual=true;
    for(const auto& v:mesh[0].mesh->Vertices())residual &= std::abs(plane.Height(v.position.x,v.position.z)-v.position.y)<1e-6;
    Check("urban fitted tilted plane reconstructs all source heights",residual);
    Check("urban fitted plane has a unit upward normal",std::abs(std::hypot(plane.normal[0],plane.normal[1],plane.normal[2])-1)<1e-12 && plane.normal[1]>0);
    Check("urban plane height uses both horizontal coordinates",std::abs(plane.Height(3,4)-2.2)<1e-6);
    Check("urban plane fit rejects empty geometry",Rejects([]{(void)HeliosUrbanStaticDetail::FitGroundPlane({});}));
    auto line=mesh;line[0].mesh=std::make_shared<TriangleMesh>(std::vector<MeshVertex>{{{0,0,0},{},{}},{{1,1,1},{},{}},{{2,2,2},{},{}}},std::vector<TriangleIndices>{{0,1,2}});
    Check("urban plane fit rejects collinear geometry",Rejects([&]{(void)HeliosUrbanStaticDetail::FitGroundPlane(line);}));
    auto vertical=mesh;vertical[0].mesh=std::make_shared<TriangleMesh>(std::vector<MeshVertex>{{{0,0,0},{},{}},{{0,1,0},{},{}},{{0,0,1},{},{}}},std::vector<TriangleIndices>{{0,1,2}});
    Check("urban grounding refuses a vertical fitted plane",Rejects([&]{(void)HeliosUrbanStaticDetail::FitGroundPlane(vertical);}));
    HeliosUrbanPartSpec spec{"synthetic",1,{0,0,0},{{{'z',90},{'x',90}}},1,false,false};
    Check("urban source Z rotation maps to engine Y rotation",Near(HeliosUrbanStaticDetail::FloatPoint(HeliosUrbanStaticDetail::Apply(HeliosUrbanStaticDetail::Rotation(spec),{1,0,0})),{0,0,-1}));
    spec.rotationCount=2;
    Check("urban multiple global rotations preserve XML order",Near(HeliosUrbanStaticDetail::FloatPoint(HeliosUrbanStaticDetail::Apply(HeliosUrbanStaticDetail::Rotation(spec),{1,0,0})),{0,1,0}));
    std::vector<ImportedMesh> object{{"triangle",std::make_shared<TriangleMesh>(std::vector<MeshVertex>{
        {{1,2,-3},{0,1,0},{0.3f,0.7f,0}},{{4,2,-3},{0,1,0},{}},{{1,2,-5},{0,1,0},{}}},std::vector<TriangleIndices>{{0,1,2}}),{-1}}};
    spec.rotationCount=1;spec.scale=2;spec.sourceOffset={10,20,30};
    HeliosUrbanPartInfo info;const auto placed=HeliosUrbanStaticDetail::Place(object,spec,0,plane,info);
    Check("urban placement uses scale rotation offset without recentering",Near(placed[0].mesh->Vertices()[0].position,{4,34,-22}));
    Check("urban placement never scales the normal",Near(placed[0].mesh->Vertices()[0].normal,{0,1,0}));
    Check("urban placement retains UVs and source geometry",Near(placed[0].mesh->Vertices()[0].uv,{0.3f,0.7f,0}) && Near(object[0].mesh->Vertices()[0].position,{1,2,-3}));
    spec.scale=1;spec.rotationCount=0;spec.sourceOffset={0,0,0};spec.onGround=true;
    const auto grounded=HeliosUrbanStaticDetail::Place(object,spec,0,plane,info);
    Check("urban onGround vertically projects the lowest reference",std::abs(info.groundingReferenceAfter.y-plane.Height(info.groundingReferenceAfter.x,info.groundingReferenceAfter.z))<1e-6);
    Check("urban onGround leaves horizontal placement unchanged",grounded[0].mesh->Vertices()[0].position.x==1 && grounded[0].mesh->Vertices()[0].position.z==-3);
    spec.sourceOffset={1000,0,0};
    Check("urban onGround rejects references outside terrain bounds",Rejects([&]{(void)HeliosUrbanStaticDetail::Place(object,spec,0,plane,info);}));
    Check("urban fixed recipe identifies exactly two dynamic vehicles",std::count_if(HeliosUrbanStaticParts.begin(),HeliosUrbanStaticParts.end(),[](const auto& p){return p.originallyDynamic;})==2);
    Check("urban fixed recipe has eighteen grounding placements",std::count_if(HeliosUrbanStaticParts.begin(),HeliosUrbanStaticParts.end(),[](const auto& p){return p.onGround;})==18);
    Check("urban missing local data is reported before partial assembly",Rejects([]{(void)HeliosUrbanStaticScene::Load(Assets()/"intentionally_missing");}));
}
HeliosUrbanStaticSceneResult VerifyRealScene()
{
    const auto start=std::chrono::steady_clock::now();
    auto result=HeliosUrbanStaticScene::Load(Assets());
    Check("urban import reads all twenty seven distinct OBJ assets",result.assets.size()==27);
    Check("urban initial static snapshot retains forty two source parts",result.parts.size()==42);
    Check("urban scene assembles fifty three mesh groups",result.scene->ObjectCount()==53);
    Check("urban BVHs retain all nondegenerate placed triangles",result.triangleCount==2445341);
    Check("urban unique vehicle cleanup reports eighteen source faces",result.skippedSourceTriangles==18);
    Check("urban repeated placements report fifty four skipped contributions",result.skippedInstancedTriangles==54);
    bool materials=true,vehicleCounts=true;std::set<std::string> assetNames;
    for(const auto& asset:result.assets) {
        materials &= asset.warning.empty() && (asset.file=="Inliner-Bahn.obj" ? asset.materialCount==0 : asset.materialCount>0);
        assetNames.insert(asset.file);
        if(asset.file.starts_with("vivaro"))vehicleCounts &= asset.triangles.importedTriangleCount==161988 && asset.triangles.skippedDegenerateTriangleCount==6;
        else vehicleCounts &= asset.triangles.skippedDegenerateTriangleCount==0;
    }
    Check("urban referenced MTL files parse and unassigned material stays absent",materials);
    Check("urban unique asset cache has no duplicate source loads",assetNames.size()==27);
    Check("urban cleanup is limited to six zero area faces per vehicle asset",vehicleCounts);
    bool grouped=true,identity=true,grounded=true,scaled=true;std::size_t sceneIndex=0,triangles=0;
    for(const auto& part:result.parts) {
        const auto asset=std::find_if(result.assets.begin(),result.assets.end(),[&](const auto& a){return a.file==part.file;});
        const auto& spec=HeliosUrbanStaticParts[part.sourcePartIndex];
        grouped &= part.firstSceneObject==sceneIndex && part.sceneObjectCount==asset->meshGroups && part.triangleCount==asset->triangles.importedTriangleCount;
        sceneIndex+=part.sceneObjectCount;triangles+=part.triangleCount;
        if(!part.onGround && spec.rotationCount==0 && spec.scale==1)
            identity &= Near(part.bounds.pMin,asset->sourceEngineBounds.pMin) && Near(part.bounds.pMax,asset->sourceEngineBounds.pMax);
        if(part.onGround)grounded &= std::abs(part.groundingReferenceAfter.y-result.groundPlane.Height(part.groundingReferenceAfter.x,part.groundingReferenceAfter.z))<2e-5;
        if(part.file=="small_maple.obj")scaled &= std::abs((part.bounds.pMax.y-part.bounds.pMin.y)-(asset->sourceEngineBounds.pMax.y-asset->sourceEngineBounds.pMin.y)*spec.scale)<2e-5;
    }
    Check("urban source part to mesh group associations stay contiguous",grouped && sceneIndex==53 && triangles==2445341);
    Check("urban untransformed buildings terrain vehicles retain original layout",identity);
    Check("urban all eighteen onGround references reach the fitted plane",grounded);
    Check("urban XML tree scales change height independently of grounding",scaled);
    const auto bounds=result.scene->Bounds();
    Check("urban terrain footprint retains hundreds of metres not toy resizing",bounds.pMin.x < -136 && bounds.pMax.x>124 && bounds.pMin.z < -226 && bounds.pMax.z>113);
    Check("urban converted scene is finite and Y up",std::isfinite(bounds.pMin.y) && bounds.pMin.y==0 && bounds.pMax.y>28 && bounds.pMax.y<30);
    const auto terrain=ObjMeshLoader::Convert(ObjMeshLoader::Read((Assets()/"Terrain.obj").string()),[] {ObjImportOptions o;o.sourceAxesInEngine={ObjAxisDirection::PositiveX,ObjAxisDirection::NegativeZ,ObjAxisDirection::PositiveY};return o;}());
    TriangleMeshAggregate terrainOnly(terrain[0].mesh,nullptr);HitRecord a,b;
    const Ray groundRay({0,100,-70},{0,-1,0});
    Check("urban scene ray hits the same terrain as independently imported mesh",result.scene->hit(groundRay,1e-4f,200,a) && terrainOnly.hit(groundRay,1e-4f,200,b) && std::abs(a.t-b.t)<1e-5f);
    Check("urban original building roof remains intersectable",result.scene->hit(Ray({-30,40,-97},{0,-1,0}),1e-4f,100,a) && a.point.y>20);
    Check("urban sky ray creates no geometry hit",!result.scene->hit(Ray({0,bounds.pMax.y+5,-70},{0,1,0}),1e-4f,100,a));
    Check("urban rays outside terrain and city bounds miss",!result.scene->hit(Ray({bounds.pMax.x+10,100,0},{0,-1,0}),1e-4f,200,a));
    Check("urban BVH respects near ray segment limits",!result.scene->hit(groundRay,1e-4f,1,a));
    bool finiteHits=true;std::size_t hitCount=0;
    for(int ix=0;ix<16;++ix)for(int iz=0;iz<16;++iz) {
        const Ray ray({-100+float(ix)*12,100,-180+float(iz)*16},{0,-1,0});
        HitRecord hit;
        if(result.scene->hit(ray,1e-4f,200,hit)) {
            ++hitCount;
            finiteHits &= std::isfinite(hit.t) && hit.t>0 && Near(hit.point,ray.at(hit.t),1e-4f) && std::abs(hit.geometricNormal.norm()-1)<1e-4f;
        }
    }
    Check("urban full city BVH returns finite consistent surface intersections",finiteHits && hitCount==256);
    const auto reader=ObjMeshLoader::Read((Assets()/"vivaro.obj").string());
    Check("urban explicit vehicle cleanup has not weakened default OBJ policy",Rejects([&]{(void)ObjMeshLoader::Convert(reader);}));
    std::cout<<"[INFO] urban parts="<<result.parts.size()<<" assets="<<result.assets.size()<<" groups="<<result.scene->ObjectCount()<<" triangles="<<result.triangleCount
        <<" load_build_and_checks_s="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<'\n';
    return result;
}
void BoundsJson(std::ostream& output,const Bounds3f& bounds)
{
    output<<"{\"min\":["<<bounds.pMin.x<<','<<bounds.pMin.y<<','<<bounds.pMin.z<<"],\"max\":["<<bounds.pMax.x<<','<<bounds.pMax.y<<','<<bounds.pMax.z<<"]}";
}
}
void RunHeliosUrbanStaticSceneAcceptanceTests()
{
    MathTests();
    TerrainContactTests();
    if(!HeliosUrbanStaticScene::AssetsPresent(Assets())){std::cout<<"[SKIP] urban external assets absent; run models/helios_urban_static/FetchUrbanStatic.ps1.\n";return;}
    (void)VerifyRealScene();
    RealTerrainContactTests();
}
void ExportHeliosUrbanStaticImport(const std::filesystem::path& directory)
{
    MathTests();const auto result=VerifyRealScene();
    std::filesystem::create_directories(directory);
    if(std::filesystem::exists(directory/"summary.txt")||std::filesystem::exists(directory/"scene-layout.json"))
        throw std::runtime_error("Refusing to overwrite urban import reports.");
    std::ofstream summary(directory/"summary.txt"),layout(directory/"scene-layout.json");
    if(!summary||!layout)throw std::runtime_error("Cannot open urban import reports.");
    summary.imbue(std::locale::classic());layout.imbue(std::locale::classic());summary<<std::setprecision(17);layout<<std::setprecision(17);
    summary<<"HELIOS++ urban static initial-state import; commit 2b3585aa766213ae9a46c4790ad4f3594ffa213d\n"
        <<"27 unique OBJ, 42 source parts, 53 mesh objects, 2445341 triangles\n"
        <<"18 zero-area source faces skipped; 54 contributions across vehicle placements. Strict OBJ default unchanged.\n"
        <<"Source metre assumption, (x,y,z)->(x,z,-y), no per-asset recenter, no global scene shift.\n"
        <<"Transform scale*R*p+offset, global rotation order preserved; onGround=1 uses a fitted Terrain plane.\n"
        <<"Plane grounding does NOT guarantee contact with the actual nonplanar terrain mesh.\n"
        <<"Two moving cars frozen before dmotion; static geometry and BVHs only, no new scan/PCD yet.\n"
        <<"MTL parsed, not converted to spectral LiDAR reflectance or semantic labels.\n";
    for(const auto& asset:result.assets)summary<<"ASSET "<<asset.file<<" groups="<<asset.meshGroups<<" triangles="<<asset.triangles.importedTriangleCount<<" skipped="<<asset.triangles.skippedDegenerateTriangleCount<<'\n';
    layout<<"{\"coordinate_frame\":\"world_y_up_metres\",\"geometry\":\"part_bounds_only_not_full_mesh\",\"triangle_count\":"<<result.triangleCount<<",\"bounds\":";
    BoundsJson(layout,result.scene->Bounds());layout<<",\"parts\":[";
    for(std::size_t i=0;i<result.parts.size();++i) {
        const auto& part=result.parts[i];if(i)layout<<',';
        layout<<"{\"index\":"<<i<<",\"file\":\""<<part.file<<"\",\"originally_dynamic\":"<<(part.originallyDynamic?"true":"false")
            <<",\"on_ground\":"<<(part.onGround?"true":"false")<<",\"ground_shift_m\":"<<part.groundShiftMeters<<",\"triangles\":"<<part.triangleCount<<",\"bounds\":";
        BoundsJson(layout,part.bounds);layout<<'}';
        summary<<"PART "<<i<<' '<<part.file<<" ground_shift_m="<<part.groundShiftMeters<<'\n';
    }
    layout<<"]}\n";summary.close();layout.close();
    if(!summary||!layout)throw std::runtime_error("Urban import report write failed.");
}
