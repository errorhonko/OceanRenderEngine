#include "scene/examples/HeliosUrbanStaticScene.h"
#include "lidar/scan/LidarScanDriver.h"
#include "lidar/scan/LidarRotaryScanPlan.h"
#include "lidar/output/LidarPcdWriter.h"
#include "lidar/emitter/pulse/GaussianPulseProfile.h"
#include "lidar/emitter/beam/GaussianLidarBeamProfile.h"
#include "reflection/lidar/SurfaceLidarScattering.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <numeric>

namespace {
constexpr double ScanPi = 3.14159265358979323846;
using Clock = std::chrono::steady_clock;
double Seconds(Clock::time_point start) { return std::chrono::duration<double>(Clock::now()-start).count(); }
void Require(bool condition, const char* text) {
    if (!condition) throw std::runtime_error(text);
    std::cout << "[PASS] " << text << std::endl;
}
const char* SourceName(ObjLidarBindingSource source) {
    switch (source) {
    case ObjLidarBindingSource::Override: return "override";
    case ObjLidarBindingSource::ReflectancePercent: return "percent";
    case ObjLidarBindingSource::Spectrum: return "spectrum";
    case ObjLidarBindingSource::ExplicitFallback: return "fallback";
    case ObjLidarBindingSource::HeliosDefault: return "assumed_50_percent";
    } return "unknown";
}
const char* KindName(const HitRecord& hit) {
    auto model = dynamic_cast<const HeliosEmpiricalLidarScattering*>(hit.lidarScattering.get());
    if (!model) return hit.lidarScattering ? "other_model" : "unbound";
    switch (model->AngularResponse().Kind()) {
    case HeliosEmpiricalKind::Phong: return "HELIOS_Phong";
    case HeliosEmpiricalKind::Lambert: return "HELIOS_Lambert";
    case HeliosEmpiricalKind::DirectionIndependent: return "HELIOS_direction_independent";
    } return "unknown";
}
LidarScanDriverConfig Config() {
    LidarScanDriverConfig c;
    c.wavelengthNm = 905; c.reception.wavelengthNm = 905;
    c.pulseEnergyJ = 1e-8f;
    c.samplesPerPulse = 16; c.raySamplingMode = LidarRaySamplingMode::Stratified2D;
    c.xStrata = c.yStrata = 4; c.raySeed = 42; c.noiseSeed = 43;
    c.beamProfile = std::make_shared<GaussianLidarBeamProfile>(0.00015,0.00045);
    c.receiverFovHalfAngleRadians = 0.002f;
    c.receiverApertureAreaSquareMeters = 0.002f; c.receiverOpticalEfficiency = 0.8f;
    c.waveform = {-20e-9,0.5e-9,4096};
    c.reception.quantumEfficiency = 0.6;
    c.reception.responseTimeConstantSeconds = 2e-9;
    c.reception.transimpedanceGainVoltsPerAmp = 10000;
    c.reception.noiseSigmaVoltageV = 0;
    c.reception.adcBitCount = 16;
    c.reception.adcMinimumVoltageV = -1; c.reception.adcMaximumVoltageV = 1;
    c.reception.noiseWindow = {-20e-9,-10e-9}; c.reception.searchWindow = {0,2000e-9};
    c.reception.minimumThresholdCodes = 2;
    c.reception.rangeMethod = LidarReceiverRangeMethod::PeakNeighborhoodCentroid;
    c.reception.neighborhood = {12,24};
    c.reception.calibration = LidarRangeCalibration(2e-9);
    c.rangeSource = LidarPointRangeSource::Corrected;
    c.rejectClippedReturns = true;
    return c;
}
std::string PointText(const std::optional<LidarPointSample>& point) {
    if (!point) return "none";
    std::ostringstream out; out << std::setprecision(17) << point->rangeMeters << ','
        << point->peakAboveBaselineCodes << ',' << point->positionWorld.x << ','
        << point->positionWorld.y << ',' << point->positionWorld.z;
    return out.str();
}
}

int wmain(int argc, wchar_t** argv) {
    try {
        if (argc != 3) throw std::runtime_error("Expected repository root and a new output directory.");
        const auto repo = std::filesystem::absolute(argv[1]);
        const auto output = std::filesystem::absolute(argv[2]);
        if (std::filesystem::exists(output)) throw std::runtime_error("Refusing to overwrite an existing run.");
        std::filesystem::create_directories(output);
        const auto loadStart = Clock::now();
        auto options = ObjMaterialBindingOptions::HeliosEmpiricalBaseline(
            repo/"data/materials/spectra/helios");
        options.createDiffuseRenderMaterials = false;
        auto imported = HeliosUrbanStaticScene::Load(repo/"models/helios_urban_static",options,
            {HeliosUrbanGroundingMode::TerrainContact});
        const double loadSeconds = Seconds(loadStart);
        std::ofstream placementReport(output/"placement_report.csv");
        placementReport << std::setprecision(17) << "part,file,source_on_ground,terrain_grounded,tire_supports,support_count,shift_y_m,contact_x,contact_y,contact_z\n";
        for(const auto& part:imported.parts)placementReport << part.sourcePartIndex << ',' << part.file << ',' << part.onGround << ','
            << part.terrainGrounded << ',' << part.groundingUsesTires << ',' << part.terrainSupportPointCount << ',' << part.groundShiftMeters << ','
            << part.groundingReferenceAfter.x << ',' << part.groundingReferenceAfter.y << ',' << part.groundingReferenceAfter.z << '\n';
        placementReport.close();
        Require(imported.triangleCount == 2445341 && imported.assets.size() == 27,"full urban geometry imported");
        std::ofstream materials(output/"material_bindings.tsv");
        materials << "asset\tmaterial\tsource\tspectrum\twarnings\n";
        std::map<std::string,std::size_t> bindingCounts;
        for (const auto& asset : imported.assets) for (const auto& info : asset.materialBindings) {
            ++bindingCounts[SourceName(info.source)];
            materials << asset.file << '\t' << info.name << '\t' << SourceName(info.source) << '\t'
                << info.spectrumPath.generic_string() << '\t';
            for (const auto& warning : info.warnings) materials << warning << " | ";
            materials << '\n';
        }
        Require(bindingCounts["spectrum"] == 1,"terrain spectrum resolved explicitly");
        HitRecord ground;
        Require(imported.scene->hit(Ray({0,100,-70},{0,-1,0}),1e-4f,200,ground),"sensor road height found by geometric intersection");
        Require(dynamic_cast<const HeliosEmpiricalLidarScattering*>(ground.lidarScattering.get()) != nullptr,
            "ground hit carries an empirical material");
        LidarScanPoseConfig pose{{0,ground.point.y+1.8f,-70},{0,0,1},{0,1,0}};
        LidarRotaryScanConfig scan;
        for (std::uint32_t i=0; i<16; ++i) scan.channels.push_back({i,(-25.0+40.0*double(i)/15.0)*ScanPi/180});
        scan.azimuthSamplesPerRevolution = 720; scan.rotationFrequencyHz = 10;
        scan.startTimeSeconds = 1; scan.firstPulseId = 1000;
        const LidarRotaryScanPlan plan(scan);
        const auto config = Config();
        const SurfaceLidarScattering scattering;
        const GaussianPulseProfile profile(4e-9);
        const LidarScanDriver driver(*imported.scene,scattering,profile,plan,pose,config);
        const auto scanStart = Clock::now();
        LidarScanFrameResult frame;
        frame.pulses.reserve(driver.Size()); frame.points.reserve(driver.Size());
        for (std::size_t i=0; i<driver.Size(); ++i) {
            auto pulse = driver.SimulatePulse(i);
            frame.points.insert(frame.points.end(),pulse.points.begin(),pulse.points.end());
            frame.pulses.push_back(std::move(pulse));
            if ((i+1)%1440 == 0) std::cout << "[PROGRESS] " << i+1 << '/' << driver.Size() << " pulses" << std::endl;
        }
        const double scanSeconds = Seconds(scanStart);
        Require(frame.pulses.size() == 11520,"all planned pulses simulated through receiver pipeline");
        Require(!frame.points.empty(),"digital waveform ranging produced points");
        std::ofstream pulseLog(output/"pulses.csv");
        pulseLog << std::setprecision(17) << "pulse_id,ring,azimuth_deg,elevation_deg,center_hit,center_range_m,center_model,center_rho,mc_hits,mc_returns,received_energy_J,recorded_energy_J,detected,adc_out_of_range,baseline_code,threshold_codes,range_m,amplitude_codes,range_minus_center_m\n";
        std::map<std::string,std::size_t> hitKinds;
        std::size_t centerHits=0, noMcHit=0, hitNoReturn=0, noDetection=0, clipped=0;
        std::size_t pointsWithoutCenterHit=0;
        double totalEnergy=0, recordedEnergy=0, minRange=1e100, maxRange=0;
        std::vector<double> absCenterDifferences;
        for (std::size_t i=0; i<frame.pulses.size(); ++i) {
            const auto& pulse = frame.pulses[i];
            const auto& p = pulse.scan.pulse;
            HitRecord truth;
            const bool hit = imported.scene->hit(Ray(p.OriginWorld(),p.CenterDirectionWorld()),1e-4f,
                std::numeric_limits<float>::infinity(),truth);
            std::string kind = "sky"; double rho=0;
            if (hit) {
                ++centerHits; kind=KindName(truth); ++hitKinds[kind];
                auto model=dynamic_cast<const HeliosEmpiricalLidarScattering*>(truth.lidarScattering.get());
                if(!model) throw std::runtime_error("Center hit lacks empirical material binding.");
                rho=model->ReflectanceAtWavelength(905);
            }
            if (!pulse.surfaceHitCount) ++noMcHit;
            if (pulse.surfaceHitCount && !pulse.returnSampleCount) ++hitNoReturn;
            if (!pulse.point) ++noDetection;
            if (pulse.reception.hasAdcOutOfRangeSamples) ++clipped;
            if(!std::isfinite(pulse.receivedEnergyJ) || pulse.receivedEnergyJ<0) throw std::runtime_error("Invalid pulse energy.");
            totalEnergy+=pulse.receivedEnergyJ; recordedEnergy+=pulse.recordedEnergyJ;
            pulseLog << p.PulseId() << ',' << pulse.scan.firing.channelId << ',' << (i/16)*0.5 << ','
                << (-25.0+40.0*double(pulse.scan.firing.channelId)/15.0) << ',' << hit << ',';
            if(hit) pulseLog << truth.t;
            pulseLog << ',' << kind << ',' << rho << ',' << pulse.surfaceHitCount << ',' << pulse.returnSampleCount << ','
                << pulse.receivedEnergyJ << ',' << pulse.recordedEnergyJ << ',' << bool(pulse.point) << ','
                << pulse.reception.hasAdcOutOfRangeSamples << ',' << pulse.reception.baseline.meanCode << ','
                << pulse.reception.thresholdCodes << ',';
            if (pulse.point) {
                const auto& point=*pulse.point;
                minRange=std::min(minRange,point.rangeMeters); maxRange=std::max(maxRange,point.rangeMeters);
                if (!hit) ++pointsWithoutCenterHit;
                pulseLog << point.rangeMeters << ',' << point.peakAboveBaselineCodes << ',';
                if(hit) { const double d=point.rangeMeters-truth.t; pulseLog << d; absCenterDifferences.push_back(std::abs(d)); }
            } else pulseLog << ",,";
            pulseLog << '\n';
        }
        Require(hitKinds["unbound"]==0,"all central surface hits have a LiDAR model");
        Require(recordedEnergy<=totalEnergy*(1+1e-10),"waveform recording does not create energy");
        Require(clipped==0,"no ADC clipping in this configuration");
        for (const std::size_t index : {std::size_t(0),std::size_t(100),std::size_t(1000),std::size_t(5000)}) {
            const auto replay=driver.SimulatePulse(index);
            const auto& old=frame.pulses[index];
            Require(replay.receivedEnergyJ==old.receivedEnergyJ && PointText(replay.point)==PointText(old.point),
                "fixed-seed pulse replay is identical");
        }
        // Explicit sky probe goes through the same optical -> ADC -> ranging chain.
        LidarScanDriverConfig skyConfig=config;
        skyConfig.scan.originWorld=pose.originWorld; skyConfig.scan.forwardWorld={0,1,0}; skyConfig.scan.upHintWorld={0,0,1};
        skyConfig.scan.azimuthCount=skyConfig.scan.elevationCount=1;
        skyConfig.scan.startTimeSeconds=1;
        const auto sky=LidarScanDriver(*imported.scene,scattering,profile,skyConfig).SimulateFrame();
        Require(sky.points.empty() && sky.pulses[0].surfaceHitCount==0,"upward empty-space probe produces no fabricated point");
        LidarPointCloudFrame cloud{0,1,1.1,pose,frame.points};
        LidarPcdWriter::Save(output/"urban_world.pcd",cloud);
        LidarPcdWriter::Save(output/"urban_sensor.pcd",cloud,LidarPcdCoordinates::SensorAtEmission);
        std::ofstream summary(output/"summary.txt");
        summary << std::setprecision(17)
            << "HELIOS urban static geometry + empirical material baseline; own receiver and ranging\n"
            << "TerrainContact placement: actual Terrain BVH, tire supports for all nine vehicles; SourcePlane mode remains available\n"
            << "Not a reproduction of an upstream HELIOS survey or calibrated real sensor\n"
            << "triangles=" << imported.triangleCount << " assets=" << imported.assets.size() << " parts=" << imported.parts.size() << '\n'
            << "pose_world_m=" << pose.originWorld.x << ',' << pose.originWorld.y << ',' << pose.originWorld.z << '\n'
            << "road_height_m=" << ground.point.y << " sensor_height_above_road_m=1.8\n"
            << "fixed_scene_and_pose; 16 channels [-25,+15] deg; 720 columns; 10 Hz; 905 nm; 10 nJ\n"
            << "Gaussian e2 half-angle=0.15 mrad, cutoff=0.45 mrad; 16 rays/pulse; 4x4 stratified; seeds=42,43\n"
            << "aperture=0.002 m2; efficiency=0.8; QE=0.6; receiver FOV half-angle=2 mrad\n"
            << "pulse FWHM=4 ns; response tau=2 ns; gain=10000 V/A; voltage noise=0\n"
            << "ADC 16 bits [-1,+1] V; minimum threshold=2 codes; bin=0.5 ns; bins=4096; start=-20 ns\n"
            << "noise window [-20,-10) ns; search [0,2000) ns (~0..299.8 m before delay correction)\n"
            << "single peak-neighborhood centroid; bins before/after=12/24; fixed delay correction=2 ns\n"
            << "No fitted truth-based range correction. Fixed 2 ns correction is approximate for a finite local window.\n"
            << "pulses=" << frame.pulses.size() << " emitted_rays=" << frame.pulses.size()*config.samplesPerPulse
            << " measured_points=" << frame.points.size() << " center_hits=" << centerHits << '\n'
            << "no_MC_hit=" << noMcHit << " MC_hit_without_positive_return=" << hitNoReturn
            << " no_measured_point=" << noDetection << " clipped_records=" << clipped
            << " point_without_center_hit=" << pointsWithoutCenterHit << '\n'
            << "measured_range_min_m=" << minRange << " max_m=" << maxRange << '\n'
            << "sum_received_energy_J=" << totalEnergy << " sum_recorded_energy_J=" << recordedEnergy << '\n'
            << "load_and_BVH_s=" << loadSeconds << " scan_s=" << scanSeconds << '\n';
        for (const auto& [kind,count]:hitKinds) summary << "center_hits_" << kind << '=' << count << '\n';
        for (const auto& [source,count]:bindingCounts) summary << "material_entries_" << source << '=' << count << '\n';
        std::sort(absCenterDifferences.begin(),absCenterDifferences.end());
        if(!absCenterDifferences.empty()) {
            summary << "center_reference_abs_difference_median_m=" << absCenterDifferences[absCenterDifferences.size()/2]
                << " p95_m=" << absCenterDifferences[std::size_t(0.95*(absCenterDifferences.size()-1))]
                << " max_m=" << absCenterDifferences.back() << '\n';
        }
        summary << "Center-ray comparisons are diagnostic, not pure range errors: the finite footprint may see different surfaces.\n"
            << "PCD intensity is baseline-subtracted ADC amplitude codes, not reflectance. World coordinates are Y-up metres.\n"
            << "No atmospheric loss, receiver noise, vegetation transmission or multiple scattering in this run.\n";
        materials.close(); pulseLog.close(); summary.close();
        Require(materials.good() && pulseLog.good() && summary.good(),"reports and point clouds saved");
        std::cout << "[RESULT] points=" << frame.points.size() << " pulses=" << frame.pulses.size()
            << " load_s=" << loadSeconds << " scan_s=" << scanSeconds << std::endl;
        return 0;
    } catch(const std::exception& e) { std::cerr << "[FAIL] " << e.what() << std::endl; return 1; }
}
