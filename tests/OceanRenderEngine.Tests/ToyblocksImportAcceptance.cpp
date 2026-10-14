#include "scene/SceneBuilder.h"
#include "lidar/scan/LidarMultiFrameDriver.h"
#include "lidar/scan/LidarRotaryScanPlan.h"
#include "lidar/output/LidarPcdWriter.h"
#include "reflection/lidar/LambertianLidarScattering.h"
#include "lidar/emitter/pulse/GaussianPulseProfile.h"
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
constexpr double Pi = 3.14159265358979323846;
void Check(const char* name, bool condition)
{
    if (!condition) { std::cerr << "[FAIL] " << name << '\n'; throw std::runtime_error(name); }
    std::cout << "[PASS] " << name << '\n';
}
std::filesystem::path AssetDirectory()
{
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
        "models" / "helios_toyblocks";
}
bool AssetsPresent()
{
    for (const auto* stem : {"groundplane", "cube", "sphere", "cylinder"})
        for (const auto* suffix : {".obj", ".mtl"})
            if (!std::filesystem::is_regular_file(AssetDirectory() / (std::string(stem) + suffix))) return false;
    return true;
}
// Experiment-only MODEL resizing, after proper unit/axis conversion. Original
// downloaded files stay intact. This is not a reinterpretation of metresPerUnit.
std::vector<ImportedMesh> ResizeAndFootCenter(const std::vector<ImportedMesh>& source, float scale)
{
    Bounds3f bounds;
    for (const auto& part : source)
        for (const auto& vertex : part.mesh->Vertices()) bounds = Union(bounds, vertex.position);
    const Vector3f pivot{(bounds.pMin.x + bounds.pMax.x) * 0.5f, bounds.pMin.y,
        (bounds.pMin.z + bounds.pMax.z) * 0.5f};
    auto result = source;
    for (auto& part : result)
    {
        auto vertices = part.mesh->Vertices();
        for (auto& vertex : vertices) vertex.position = (vertex.position - pivot) * scale;
        part.mesh = std::make_shared<TriangleMesh>(std::move(vertices), part.mesh->Triangles());
    }
    return result;
}
struct ToyblocksInput
{
    std::shared_ptr<StaticScene> scene;
    std::array<std::size_t, 4> triangleCounts{};
    bool cleanMaterials = true;
};
ToyblocksInput CreateScene()
{
    if (!AssetsPresent()) throw std::runtime_error("Fetch local Toyblocks assets using models/helios_toyblocks/FetchToyblocks.ps1.");
    ToyblocksInput result;
    SceneBuilder builder;
    ObjImportOptions import;
    import.sourceAxesInEngine = {ObjAxisDirection::PositiveX,ObjAxisDirection::NegativeZ,ObjAxisDirection::PositiveY};
    const std::array<std::string, 4> names{"groundplane","cube","sphere","cylinder"};
    const std::array<float, 4> modelScales{20.0f,0.1f,0.05f,0.1f};
    const std::array<Vector3f, 4> placements{Vector3f(0,0,0),Vector3f(0,0,8),Vector3f(-4,0,10),Vector3f(4,0,10)};
    for (std::size_t i = 0; i < names.size(); ++i)
    {
        const auto reader = ObjMeshLoader::Read((AssetDirectory() / (names[i] + ".obj")).string());
        result.cleanMaterials &= reader.Warning().empty() && !reader.GetMaterials().empty();
        auto parts = ObjMeshLoader::Convert(reader, import);
        for (auto& part : parts) { result.triangleCounts[i] += part.mesh->Triangles().size(); part.name = names[i] + ":" + part.name; }
        parts = ResizeAndFootCenter(parts, modelScales[i]);
        SceneObjectPose pose; pose.translationMeters = placements[i]; builder.AddMeshes(parts, pose);
        if (names[i] == "cube") { pose.translationMeters = {0,0,13}; builder.AddMeshes(parts, pose); }
    }
    result.scene = builder.Build();
    return result;
}
LidarScanDriverConfig ReceiverConfig()
{
    LidarScanDriverConfig config;
    config.samplesPerPulse = 1; // Delta beam, geometry/receiver baseline, no footprint variance.
    config.pulseEnergyJ = 1e-8f; config.raySeed = 42; config.noiseSeed = 43;
    config.waveform = {0,0.25e-9,2048};
    config.reception.noiseSigmaVoltageV = 0; config.reception.adcBitCount = 16;
    config.reception.noiseWindow = {0,10e-9}; config.reception.searchWindow = {10e-9,450e-9};
    config.reception.rangeMethod = LidarReceiverRangeMethod::Centroid;
    config.reception.calibration = LidarRangeCalibration(2e-9);
    config.rangeSource = LidarPointRangeSource::Corrected;
    return config;
}
LidarRotaryScanPlan RotaryPlan()
{
    LidarRotaryScanConfig config;
    config.channels = {{0,-20*Pi/180},{1,-10*Pi/180},{2,-3*Pi/180},{3,0},{4,3*Pi/180},{5,10*Pi/180}};
    config.azimuthSamplesPerRevolution = 64; config.rotationFrequencyHz = 10;
    config.startTimeSeconds = 1; config.firstPulseId = 1000;
    return LidarRotaryScanPlan(config);
}
LidarPoseTrajectory Trajectory()
{
    return LidarPoseTrajectory({{1,{{-0.5f,1.5f,0},{0,0,1},{0,1,0}}},
        {1.3,{{0.5f,1.5f,0},{0,0,1},{0,1,0}}}});
}
class DirectedPlan final : public LidarScanPlan
{
public:
    std::size_t Size() const override { return 4; }
    LidarFiringEvent Firing(std::size_t i) const override
    {
        const std::array<Vector3f,4> directions{Vector3f(0,0,1),Vector3f(-4,-0.5f,10),Vector3f(4,0,10),Vector3f(0,1,0)};
        return {100+static_cast<std::uint64_t>(i),0,1+double(i)*1e-4,{0,0,0},directions.at(i),std::nullopt};
    }
};
struct FrameStats
{
    std::size_t geometricHits = 0, misses = 0, measured = 0, clippedRecords = 0;
    double maxErrorMeters = 0, meanErrorMeters = 0;
    bool consistent = true, sensorCoordinates = true;
};
FrameStats Measure(const StaticScene& scene, const LidarSequenceFrameResult& result)
{
    FrameStats stats;
    for (const auto& pulse : result.pulses)
    {
        const auto& p = pulse.scan.pulse;
        HitRecord truth;
        const bool geometricHit = scene.hit(Ray(p.OriginWorld(),p.CenterDirectionWorld()),1e-4f,
            std::numeric_limits<float>::infinity(),truth);
        if (geometricHit) ++stats.geometricHits; else ++stats.misses;
        stats.consistent &= (!pulse.point || geometricHit) && pulse.surfaceHitCount == std::size_t(geometricHit);
        if (pulse.reception.hasAdcOutOfRangeSamples) ++stats.clippedRecords;
        if (pulse.point)
        {
            ++stats.measured;
            stats.consistent &= pulse.point->pulseId == p.PulseId() && pulse.point->rangeSource == LidarPointRangeSource::Corrected;
            if (geometricHit)
            {
                const double error = std::abs(pulse.point->rangeMeters - truth.t);
                stats.maxErrorMeters = std::max(stats.maxErrorMeters,error); stats.meanErrorMeters += error;
            }
            stats.sensorCoordinates &= pulse.point->positionSensor.has_value() &&
                std::abs(double(pulse.point->positionSensor->norm()) - pulse.point->rangeMeters) < 1e-4;
        }
    }
    if (stats.measured) stats.meanErrorMeters /= double(stats.measured);
    stats.consistent &= stats.measured == result.cloud.points.size();
    return stats;
}
bool CheckPcd(const LidarPointCloudFrame& cloud, LidarPcdCoordinates coordinates)
{
    std::ostringstream output; LidarPcdWriter::Write(output,cloud,coordinates);
    const auto text=output.str(); const auto data=text.find("DATA ascii\n");
    if (data==std::string::npos || text.find("POINTS "+std::to_string(cloud.points.size())+"\n")==std::string::npos) return false;
    std::istringstream rows(text.substr(data+11)); std::string line; std::size_t count=0;
    while (std::getline(rows,line)) if (!line.empty())
    {
        std::istringstream row(line); double x,y,z; if (!(row>>x>>y>>z) || count>=cloud.points.size()) return false;
        const auto expected=coordinates==LidarPcdCoordinates::World ? cloud.points[count].positionWorld : *cloud.points[count].positionSensor;
        if ((Vector3f(float(x),float(y),float(z))-expected).norm()>1e-5f) return false;
        ++count;
    }
    return count==cloud.points.size();
}
}

void RunToyblocksImportAcceptanceTests()
{
    if (!AssetsPresent()) { std::cout << "[SKIP] Toyblocks external assets absent; run models/helios_toyblocks/FetchToyblocks.ps1.\n"; return; }
    const auto input=CreateScene();
    Check("Toyblocks OBJ and MTL parse without missing material warnings",input.cleanMaterials);
    Check("Toyblocks ground OBJ imports two triangles",input.triangleCounts[0]==2);
    Check("Toyblocks cube OBJ imports twelve triangles",input.triangleCounts[1]==12);
    Check("Toyblocks sphere OBJ imports nine hundred sixty triangles",input.triangleCounts[2]==960);
    Check("Toyblocks cylinder OBJ imports one hundred twenty four triangles",input.triangleCounts[3]==124);
    Check("Toyblocks scene assembles ground and four targets",input.scene->ObjectCount()==5);
    std::size_t triangles=0; for (const auto& object:input.scene->Objects()) triangles+=object.triangleCount;
    Check("Toyblocks assembled scene retains all eleven hundred ten triangles",triangles==1110);
    Check("Toyblocks adapted test ground is forty metres wide",std::abs(input.scene->Bounds().pMax.x-input.scene->Bounds().pMin.x-40)<1e-5f);
    const LambertianLidarScattering scattering(0.5); const GaussianPulseProfile profile(4e-9);
    const DirectedPlan directed; const auto receiver=ReceiverConfig();
    const LidarScanPoseConfig fixed{{0,1.5f,0},{0,0,1},{0,1,0}};
    const auto targets=LidarScanDriver(*input.scene,scattering,profile,directed,fixed,receiver).SimulateFrame();
    Check("Toyblocks directed scan measures cube sphere and cylinder",targets.points.size()==3);
    Check("Toyblocks front cube hides rear cube in centre pulse",targets.pulses[0].point && std::abs(targets.pulses[0].point->positionWorld.z-7)<0.05);
    Check("Toyblocks sphere pulse produces left target point",targets.pulses[1].point && targets.pulses[1].point->positionWorld.x < -3);
    Check("Toyblocks cylinder pulse produces right target point",targets.pulses[2].point && targets.pulses[2].point->positionWorld.x > 3);
    Check("Toyblocks sky pulse produces no fabricated echo",!targets.pulses[3].point && targets.pulses[3].surfaceHitCount==0);
    const auto plan=RotaryPlan(); const auto motion=Trajectory();
    const LidarMultiFrameDriver driver(*input.scene,scattering,profile,plan,motion,receiver,{3,0.1,0});
    Check("Toyblocks rotary plan has six channels and sixty four columns",plan.Size()==384 && driver.Size()==3);
    std::set<std::uint64_t> ids;
    for (std::size_t i=0;i<driver.Size();++i)
    {
        const auto frame=driver.SimulateFrame(i); const auto stats=Measure(*input.scene,frame);
        Check("Toyblocks complete frame retains all planned pulses",frame.pulses.size()==384);
        Check("Toyblocks frame contains physical returns and sky misses",frame.cloud.points.size()>50 && frame.cloud.points.size()<384 && stats.misses>0);
        Check("Toyblocks ray truth and measured point associations agree",stats.consistent);
        Check("Toyblocks receiver output has no ADC clipping",stats.clippedRecords==0);
        Check("Toyblocks calibrated range errors remain below fifteen cm",stats.maxErrorMeters<0.15 && stats.meanErrorMeters<0.05);
        Check("Toyblocks sensor point radius equals measured range",stats.sensorCoordinates);
        Check("Toyblocks world PCD serialization round trips coordinates",CheckPcd(frame.cloud,LidarPcdCoordinates::World));
        Check("Toyblocks sensor PCD serialization round trips coordinates",CheckPcd(frame.cloud,LidarPcdCoordinates::SensorAtEmission));
        bool unique=true; for (const auto& pulse:frame.pulses) unique &= ids.insert(pulse.scan.pulse.PulseId()).second;
        Check("Toyblocks repeated frame pulse IDs remain unique",unique);
        std::cout<<"[INFO] Toyblocks frame "<<i<<": points="<<stats.measured<<", geometric_hits="<<stats.geometricHits
            <<", misses="<<stats.misses<<", max_error_m="<<stats.maxErrorMeters<<", mean_error_m="<<stats.meanErrorMeters<<'\n';
    }
    const auto first=driver.SimulateFrame(0), replay=driver.SimulateFrame(0);
    std::ostringstream a,b; LidarPcdWriter::Write(a,first.cloud); LidarPcdWriter::Write(b,replay.cloud);
    Check("Toyblocks seeded full frame replay is byte reproducible",a.str()==b.str());
    Check("Toyblocks platform advances between frame starts",driver.SimulateFrame(2).cloud.sensorPose.originWorld.x>first.cloud.sensorPose.originWorld.x);
}

// Experiment export only; ordinary acceptance runs serialize in memory.
void ExportToyblocksImportExperiment(const std::filesystem::path& directory)
{
    const auto input=CreateScene(); const auto plan=RotaryPlan(); const auto motion=Trajectory();
    const LambertianLidarScattering scattering(0.5); const GaussianPulseProfile profile(4e-9);
    const auto config=ReceiverConfig();
    const LidarMultiFrameDriver driver(*input.scene,scattering,profile,plan,motion,config,{3,0.1,0});
    std::filesystem::create_directories(directory);
    std::ofstream summary(directory/"summary.txt"),truth(directory/"ground_truth_frame_poses.txt");
    if (!summary || !truth) throw std::runtime_error("Cannot open Toyblocks experiment output.");
    summary<<std::setprecision(17)<<"Toyblocks adapted static scene: 5 objects, 1110 triangles\n"
        <<"HELIOS commit 2b3585aa766213ae9a46c4790ad4f3594ffa213d; not an XML survey reproduction\n"
        <<"6 channels x 64 columns, 10 Hz, 3 frames; Delta beam, 1 ray/pulse\n"
        <<"532 nm, 10 nJ, Lambert reflectance 0.5, zero voltage noise, 16-bit ADC\n"
        <<"0.25 ns x 2048 bins; search [10,450) ns; centroid with 2 ns fixed delay correction\n"
        <<"World Y-up metres; raw sensor X-right/Y-up/Z-forward, time=relative emission seconds\n"
        <<"Ground truth range is only an evaluation reference, never used to generate measured range.\n";
    truth<<std::setprecision(17)<<"# frame-start ground truth, world Y-up metres\n# timestamp tx ty tz qx qy qz qw\n";
    for (std::size_t i=0;i<driver.Size();++i)
    {
        const auto frame=driver.SimulateFrame(i); const auto stats=Measure(*input.scene,frame);
        std::ostringstream prefix; prefix<<"frame_"<<std::setw(3)<<std::setfill('0')<<i;
        LidarPcdWriter::Save(directory/(prefix.str()+"_world.pcd"),frame.cloud);
        LidarPcdWriter::Save(directory/(prefix.str()+"_sensor.pcd"),frame.cloud,LidarPcdCoordinates::SensorAtEmission);
        const auto q=LidarPoseTrajectory::Quaternion(frame.cloud.sensorPose); const auto p=frame.cloud.sensorPose.originWorld;
        truth<<frame.cloud.startTimeSeconds<<' '<<p.x<<' '<<p.y<<' '<<p.z<<' '<<q[1]<<' '<<q[2]<<' '<<q[3]<<' '<<q[0]<<'\n';
        summary<<"frame="<<i<<" points="<<stats.measured<<" geometric_hits="<<stats.geometricHits<<" misses="<<stats.misses
            <<" clipped="<<stats.clippedRecords<<" max_error_m="<<stats.maxErrorMeters<<" mean_error_m="<<stats.meanErrorMeters<<'\n';
    }
    summary.close(); truth.close();
    if (!summary || !truth) throw std::runtime_error("Toyblocks experiment output write failed.");
    std::cout<<"[OUTPUT] "<<directory.string()<<'\n';
}
