#include "scene/SceneBuilder.h"
#include "reflection/materials/DiffuseMaterial.h"
#include "lidar/scan/LidarScanDriver.h"
#include "reflection/lidar/LambertianLidarScattering.h"
#include "lidar/emitter/pulse/GaussianPulseProfile.h"
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void Check(const char* name, bool condition)
{
    if (!condition) { std::cerr << "[FAIL] " << name << '\n'; throw std::runtime_error(name); }
    std::cout << "[PASS] " << name << '\n';
}
template<class Function> bool Rejects(Function function)
{
    try { function(); } catch (const std::exception&) { return true; }
    return false;
}
bool Near(const Vector3f& a, const Vector3f& b) { return (a - b).norm() < 1e-5f; }
SceneObjectPose At(float x, float y, float z) { SceneObjectPose pose; pose.translationMeters = {x, y, z}; return pose; }
ImportedMesh Plane(float extent = 2)
{
    std::vector<MeshVertex> vertices{
        {{-extent, -extent, 0}, {0, 0, -1}, {0, 0, 0}},
        {{0, extent, 0}, {0, 0, -1}, {0.5f, 1, 0}},
        {{extent, -extent, 0}, {0, 0, -1}, {1, 0, 0}}};
    return {"plane", std::make_shared<TriangleMesh>(std::move(vertices),
        std::vector<TriangleIndices>{{0, 1, 2}}), {-1}};
}
ImportedMesh Box()
{
    std::vector<MeshVertex> vertices;
    for (const auto p : std::vector<Vector3f>{{-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},
        {-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}}) vertices.push_back({p,{0,0,0},{0,0,0}});
    std::vector<TriangleIndices> triangles{{0,3,2},{0,2,1},{4,5,6},{4,6,7},
        {0,4,7},{0,7,3},{1,2,6},{1,6,5},{0,1,5},{0,5,4},{3,7,6},{3,6,2}};
    return {"box",std::make_shared<TriangleMesh>(std::move(vertices),std::move(triangles)),std::vector<int>(12,-1)};
}
bool Hit(const StaticScene& scene, Vector3f origin, Vector3f direction, HitRecord& hit,
    float tMin = 1e-4f, float tMax = 100)
{
    return scene.hit(Ray(origin, direction), tMin, tMax, hit);
}
class TestPlan final : public LidarScanPlan
{
public:
    std::size_t Size() const override { return 3; }
    LidarFiringEvent Firing(std::size_t i) const override
    {
        const std::array<Vector3f,3> directions{Vector3f(0,0,1),Vector3f(0.4f,0,1),Vector3f(0,1,0)};
        return {100 + static_cast<std::uint64_t>(i),0,1 + double(i)*1e-4,
            {0,0,0},directions.at(i),std::nullopt};
    }
};
}

void RunSceneBuilderAcceptanceTests()
{
    SceneBuilder empty;
    const auto emptyScene = empty.Build(); HitRecord hit;
    Check("scene builder creates an empty queryable world", emptyScene->ObjectCount()==0 && emptyScene->Bounds().IsEmpty());
    Check("empty assembled scene ray misses", !Hit(*emptyScene,{0,0,0},{0,0,1},hit));
    Check("empty scene has no overlap candidates", emptyScene->PotentialBoundsOverlaps().empty());
    Check("scene builder rejects zero mesh leaf capacity", Rejects([&]{ (void)empty.Build(0,4); }));
    Check("scene builder rejects zero scene leaf capacity", Rejects([&]{ (void)empty.Build(4,0); }));
    Check("scene builder rejects null mesh", Rejects([&]{ (void)empty.AddMesh(ImportedMesh{}); }));
    Check("scene builder rejects empty triangle mesh", Rejects([&]{ (void)empty.AddMesh({"empty",
        std::make_shared<TriangleMesh>(std::vector<MeshVertex>{},std::vector<TriangleIndices>{}),{}}); }));
    Check("failed mesh additions leave builder empty", empty.ObjectCount()==0);

    SceneBuilder placed; auto source = Plane();
    const auto index = placed.AddMesh(source,At(0,0,5));
    Check("scene builder returns stable object index", index==0 && placed.ObjectCount()==1);
    Check("scene placement does not move source vertices", Near(source.mesh->Vertices()[0].position,{-2,-2,0}));
    source.mesh->UpdateVertex(0,{100,100,100},{0,0,-1});
    const auto snapshot = placed.Build();
    Check("scene builder snapshots source geometry on add", Hit(*snapshot,{0,0,0},{0,0,1},hit) && std::abs(hit.t-5)<1e-5f);
    Check("scene records name and triangle count", snapshot->Objects()[0].name=="plane" && snapshot->Objects()[0].triangleCount==1);
    Check("scene translated bounds are in world metres", Near(snapshot->Bounds().pMin,{-2,-2,5}) && Near(snapshot->Bounds().pMax,{2,2,5}));
    Check("scene translated normals keep orientation", Near(hit.normal,{0,0,-1}) && Near(hit.geometricNormal,{0,0,-1}));
    Check("scene ray tMax excludes distant target", !Hit(*snapshot,{0,0,0},{0,0,1},hit,1e-4f,4));
    Check("scene ray tMin excludes nearer target", !Hit(*snapshot,{0,0,0},{0,0,1},hit,6,10));
    placed.AddMesh(Plane(),At(0,0,3)); const auto updated = placed.Build(1,1);
    Check("scene rebuilt snapshot includes new object", updated->ObjectCount()==2 && Hit(*updated,{0,0,0},{0,0,1},hit) && std::abs(hit.t-3)<1e-5f);
    Check("previous scene remains independent of builder additions", snapshot->ObjectCount()==1 && Hit(*snapshot,{0,0,0},{0,0,1},hit) && std::abs(hit.t-5)<1e-5f);
    const auto lifetime = []{ SceneBuilder builder; builder.AddMesh(Plane(),At(0,0,7)); return builder.Build(); }();
    Check("assembled scene outlives builder and source mesh", Hit(*lifetime,{0,0,0},{0,0,1},hit) && std::abs(hit.t-7)<1e-5f);

    SceneObjectPose rotated = At(3,0,0); const double rootHalf=std::sqrt(0.5);
    rotated.rotationWxyz={rootHalf,0,rootHalf,0}; SceneBuilder rotationBuilder; rotationBuilder.AddMesh(Plane(),rotated);
    const auto rotationScene=rotationBuilder.Build();
    Check("scene quaternion applies rotation then translation", Hit(*rotationScene,{0,0,0},{1,0,0},hit) && std::abs(hit.t-3)<1e-5f);
    Check("scene rigid pose rotates both normal roles", Near(hit.normal,{-1,0,0}) && Near(hit.geometricNormal,{-1,0,0}));
    Check("scene rigid pose retains UV interpolation", std::abs(hit.u-0.5f)<1e-5f && std::abs(hit.v-0.5f)<1e-5f);
    Check("scene rotated bounds enclose world geometry", Near(rotationScene->Bounds().pMin,{3,-2,-2}) && Near(rotationScene->Bounds().pMax,{3,2,2}));
    SceneBuilder hugeQuaternion; rotated.rotationWxyz={1e308,0,1e308,0}; hugeQuaternion.AddMesh(Plane(),rotated);
    Check("scene normalizes huge finite quaternion safely", Hit(*hugeQuaternion.Build(),{0,0,0},{1,0,0},hit) && std::abs(hit.t-3)<1e-5f);
    SceneBuilder tinyQuaternion; rotated.rotationWxyz={1e-300,0,1e-300,0}; tinyQuaternion.AddMesh(Plane(),rotated);
    Check("scene normalizes tiny finite quaternion safely", Hit(*tinyQuaternion.Build(),{0,0,0},{1,0,0},hit) && std::abs(hit.t-3)<1e-5f);
    Check("scene rejects zero quaternion", Rejects([&]{ SceneObjectPose p; p.rotationWxyz={0,0,0,0}; empty.AddMesh(Plane(),p); }));
    Check("scene rejects nonfinite quaternion", Rejects([&]{ SceneObjectPose p; p.rotationWxyz[0]=std::numeric_limits<double>::infinity(); empty.AddMesh(Plane(),p); }));
    Check("scene rejects nonfinite translation", Rejects([&]{ auto p=At(0,0,0); p.translationMeters.x=std::numeric_limits<float>::quiet_NaN(); empty.AddMesh(Plane(),p); }));
    Check("scene rejects translation collapsing float geometry", Rejects([&]{ empty.AddMesh(Plane(),At(1e30f,1e30f,1e30f)); }));
    Check("scene batch addition is atomic on invalid mesh", Rejects([&]{ empty.AddMeshes({Plane(),ImportedMesh{}}); }) && empty.ObjectCount()==0);
    SceneBuilder batch; batch.AddMeshes({Plane(),Box()},At(0,0,5));
    Check("scene batch placement assembles both meshes", batch.Build()->ObjectCount()==2);

    const auto nearMaterial=std::make_shared<DiffuseMaterial>(Spectrum(0.2f));
    const auto farMaterial=std::make_shared<DiffuseMaterial>(Spectrum(0.8f));
    SceneBuilder overlap; overlap.AddMesh(Box(),At(0,0,5),nearMaterial); overlap.AddMesh(Box(),At(0,0,5.5f),farMaterial);
    const auto overlapScene=overlap.Build(1,1);
    Check("scene permits intersecting closed mesh objects", overlapScene->ObjectCount()==2);
    Check("overlapping objects return closest surface", Hit(*overlapScene,{0,0,0},{0,0,1},hit) && std::abs(hit.t-4)<1e-5f);
    Check("nearest scene surface retains engine material", hit.material==nearMaterial);
    Check("overlap diagnostic reports stable object index pair", overlapScene->PotentialBoundsOverlaps()==std::vector<std::array<std::size_t,2>>{{0,1}});
    SceneBuilder reversed; reversed.AddMesh(Box(),At(0,0,5.5f),farMaterial); reversed.AddMesh(Box(),At(0,0,5),nearMaterial);
    Check("distinct distance nearest hit ignores insertion order", Hit(*reversed.Build(1,1),{0,0,0},{0,0,1},hit) && hit.material==nearMaterial && std::abs(hit.t-4)<1e-5f);
    SceneBuilder disjoint; disjoint.AddMesh(Box(),At(-5,0,5)); disjoint.AddMesh(Box(),At(5,0,5));
    Check("disjoint scene boxes have no overlap candidates", disjoint.Build()->PotentialBoundsOverlaps().empty());
    SceneBuilder touching; touching.AddMesh(Box(),At(0,0,5)); touching.AddMesh(Box(),At(2,0,5));
    Check("touching bounds are diagnostic candidates not rejected", touching.Build()->PotentialBoundsOverlaps().size()==1);
    SceneBuilder falsePositive;
    auto lower=Plane(); lower.mesh=std::make_shared<TriangleMesh>(std::vector<MeshVertex>{
        {{-2,-2,0},{0,0,-1},{}},{{-2,2,0},{0,0,-1},{}},{{2,-2,0},{0,0,-1},{}}},std::vector<TriangleIndices>{{0,1,2}});
    auto upper=Plane(); upper.mesh=std::make_shared<TriangleMesh>(std::vector<MeshVertex>{
        {{2,2,0},{0,0,-1},{}},{{2,-1,0},{0,0,-1},{}},{{-1,2,0},{0,0,-1},{}}},std::vector<TriangleIndices>{{0,1,2}});
    falsePositive.AddMeshes({lower,upper},At(0,0,5)); const auto gap=falsePositive.Build();
    Check("overlapping bounds do not imply triangle collision", gap->PotentialBoundsOverlaps().size()==1 && !Hit(*gap,{0.25f,0.25f,0},{0,0,1},hit));

    SceneBuilder duplicates; duplicates.AddMesh(Plane(),At(0,0,5),nearMaterial); duplicates.AddMesh(Plane(),At(0,0,5),farMaterial);
    const auto duplicateScene=duplicates.Build(1,1);
    Check("coplanar duplicates report potential overlap", duplicateScene->PotentialBoundsOverlaps().size()==1);
    Check("coplanar tie returns one valid surface without priority claim", Hit(*duplicateScene,{0,0,0},{0,0,1},hit) && std::abs(hit.t-5)<1e-5f && (hit.material==nearMaterial || hit.material==farMaterial));
    const LambertianLidarScattering scattering(0.5);
    const LaserEmitter emitter({0,0,0},{0,0,1},532,1e-8f,0);
    const LidarReceiver receiver({0,0,0},{0,0,1},0.02f,0.01f,0.8f);
    IndependentSampler singleSampler(42), duplicateSampler(42);
    const auto singlePulse=LidarIntegrator(*snapshot,emitter,receiver,1,scattering).SimulatePulse(singleSampler,1);
    const auto duplicatePulse=LidarIntegrator(*duplicateScene,emitter,receiver,1,scattering).SimulatePulse(duplicateSampler,1);
    Check("duplicate opaque faces do not create two returns per ray", duplicatePulse.returns.size()==1 && singlePulse.returns.size()==1);
    Check("duplicate opaque faces do not double received energy", duplicatePulse.TotalReceivedEnergyJ()==singlePulse.TotalReceivedEnergyJ());

    const auto imported=[]{
        const auto file=(std::filesystem::path(__FILE__).parent_path()/"fixtures"/"obj_import"/"triangle.obj").string();
        const auto reader=ObjMeshLoader::Read(file); SceneBuilder builder;
        builder.AddMeshes(ObjMeshLoader::Convert(reader),At(0,3,0)); return builder.Build(); }();
    Check("OBJ read convert assemble scene survives reader destruction", Hit(*imported,{0.5f,0,0.5f},{0,1,0},hit) && std::abs(hit.t-3)<1e-5f);
    SceneBuilder scanBuilder; scanBuilder.AddMesh(Plane(),At(0,0,5)); scanBuilder.AddMesh(Plane(10),At(0,0,10));
    const auto scanScene=scanBuilder.Build(1,1); const TestPlan plan; const GaussianPulseProfile profile(4e-9);
    LidarScanDriverConfig config; config.samplesPerPulse=1; config.pulseEnergyJ=1e-8f;
    config.reception.noiseSigmaVoltageV=0; config.reception.adcBitCount=16;
    config.reception.noiseWindow={0,10e-9}; config.reception.searchWindow={20e-9,150e-9};
    config.reception.rangeMethod=LidarReceiverRangeMethod::Centroid;
    config.reception.calibration=LidarRangeCalibration(2e-9); config.rangeSource=LidarPointRangeSource::Corrected;
    const LidarScanDriver driver(*scanScene,scattering,profile,plan,LidarScanPoseConfig{{0,0,0},{0,0,1},{0,1,0}},config);
    const auto frame=driver.SimulateFrame();
    Check("assembled scene feeds unchanged full LiDAR scan pipeline", frame.pulses.size()==3 && frame.points.size()==2);
    Check("assembled scene nearer opaque mesh occludes far target", frame.pulses[0].point && std::abs(frame.pulses[0].point->positionWorld.z-5)<0.05);
    Check("assembled scene off axis pulse measures exposed far target", frame.pulses[1].point && std::abs(frame.pulses[1].point->positionWorld.z-10)<0.05);
    Check("assembled scene sky miss creates no fabricated point", !frame.pulses[2].point && frame.pulses[2].surfaceHitCount==0);
    Check("assembled scene scan replay remains deterministic", driver.SimulatePulse(0).point->rangeMeters==frame.pulses[0].point->rangeMeters);
}
