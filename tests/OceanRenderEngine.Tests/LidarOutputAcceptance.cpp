#include "lidar/scan/LidarMultiFrameDriver.h"
#include "lidar/output/LidarPcdWriter.h"
#include "lidar/scan/LidarRotaryScanPlan.h"
#include "scene/examples/LidarStaticTestScene.h"
#include "reflection/lidar/LambertianLidarScattering.h"
#include "lidar/emitter/pulse/GaussianPulseProfile.h"
#include <chrono>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

namespace {
void Check(const char* name,bool ok) {if(!ok)throw std::runtime_error(std::string("[FAIL] ")+name);std::cout<<"[PASS] "<<name<<'\n';}
template<class E,class F> bool Throws(F f) {try{f();}catch(const E&){return true;}return false;}
LidarScanDriverConfig Receiver() {
    LidarScanDriverConfig c;c.pulseEnergyJ=1e-8f;c.samplesPerPulse=3;
    c.reception.noiseSigmaVoltageV=0;c.reception.adcBitCount=16;
    c.reception.noiseWindow={0,10e-9};c.reception.searchWindow={20e-9,150e-9};
    c.reception.rangeMethod=LidarReceiverRangeMethod::Centroid;
    c.reception.calibration=LidarRangeCalibration(2e-9);c.rangeSource=LidarPointRangeSource::Corrected;
    return c;
}
class Events final:public LidarScanPlan {
public:
    std::vector<LidarFiringEvent> events;
    std::size_t Size() const override{return events.size();}
    LidarFiringEvent Firing(std::size_t i) const override{return events.at(i);}
};
struct Row {double x,y,z,intensity;std::uint32_t ring;double time,range,amplitude;std::uint32_t peak,low,high,flags,source,returnNumber,returnCount;};
std::vector<Row> ParseRows(const std::string& data) {
    std::istringstream input(data);input.imbue(std::locale::classic());std::string line;
    while(std::getline(input,line) && line!="DATA ascii"){}
    std::vector<Row> rows;
    while(std::getline(input,line)) {
        if(line.empty())continue;std::istringstream row(line);row.imbue(std::locale::classic());Row r;
        if(!(row>>r.x>>r.y>>r.z>>r.intensity>>r.ring>>r.time>>r.range>>r.amplitude>>r.peak>>r.low>>r.high>>r.flags>>r.source>>r.returnNumber>>r.returnCount))
            throw std::runtime_error("Invalid PCD row.");
        std::string extra;if(row>>extra)throw std::runtime_error("Unexpected PCD field.");rows.push_back(r);
    }
    return rows;
}
struct CommaDecimal:std::numpunct<char> {char do_decimal_point() const override{return ',';}};
}

void RunLidarOutputAcceptanceTests() {
    constexpr double pi=3.14159265358979323846;
    auto scene=CreateLidarStaticTestScene();const LambertianLidarScattering scattering(.5);const GaussianPulseProfile profile(4e-9);
    LidarRotaryScanConfig rotary;rotary.channels={{0,-15*pi/180},{1,-5*pi/180},{2,5*pi/180},{3,15*pi/180}};
    rotary.azimuthSamplesPerRevolution=16;rotary.startTimeSeconds=1;rotary.firstPulseId=123;
    const LidarRotaryScanPlan plan(rotary);const auto receiver=Receiver();const LidarMultiFrameConfig seq{3,.1,20};
    const LidarMultiFrameDriver driver(scene.world,scattering,profile,plan,scene.sensorPose,receiver,seq);
    const auto first=driver.SimulateFrame(0),second=driver.SimulateFrame(1),third=driver.SimulateFrame(2);
    Check("multi frame count and sparse id stride correct",driver.Size()==3 && driver.PulseIdStride()==64);
    Check("multi frame frame ids advance independently of pulse ids",first.cloud.frameId==20 && second.cloud.frameId==21 && third.cloud.frameId==22);
    Check("multi frame timestamps advance at configured period",first.cloud.startTimeSeconds==1 && second.cloud.startTimeSeconds==std::fma(1.0,.1,1.0) && third.cloud.endTimeSeconds==std::fma(3.0,.1,1.0));
    Check("multi frame adjacent boundaries are identical",first.cloud.endTimeSeconds==second.cloud.startTimeSeconds && second.cloud.endTimeSeconds==third.cloud.startTimeSeconds);
    Check("multi frame scans independently retain all pulses",first.pulses.size()==64 && second.pulses.size()==64 && third.pulses.size()==64);
    Check("multi frame zero noise static scene yields thirty eight points per frame",first.cloud.points.size()==38 && second.cloud.points.size()==38 && third.cloud.points.size()==38);
    bool ids=true,times=true,rings=true,geometry=true;
    for(std::size_t i=0;i<64;++i) {
        ids=ids && first.pulses[i].scan.pulse.PulseId()==123+i && second.pulses[i].scan.pulse.PulseId()==187+i;
        times=times && second.pulses[i].scan.pulse.EmissionTimeSeconds()==std::fma(1.0,.1,first.pulses[i].scan.pulse.EmissionTimeSeconds());
        if(second.pulses[i].point)rings=rings && second.pulses[i].point->channelId==second.pulses[i].scan.firing.channelId;
        geometry=geometry && first.pulses[i].receivedEnergyJ==second.pulses[i].receivedEnergyJ;
    }
    Check("multi frame pulse ids never repeat across frames",ids);
    Check("multi frame all firing times shifted not just cloud header",times);
    Check("multi frame independent point samples carry channel ids",rings);
    Check("multi frame static geometry optical energy unchanged",geometry);
    Check("multi frame independent frame replay deterministic",driver.SimulateFrame(1).cloud.points[0].rangeMeters==second.cloud.points[0].rangeMeters);
    Check("multi frame noise seeds change between frames",first.pulses[0].noiseSeed!=second.pulses[0].noiseSeed);
    Check("multi frame out of range index rejected",Throws<std::out_of_range>([&]{driver.SimulateFrame(3);}));
    auto noisy=receiver;noisy.reception.noiseSigmaVoltageV=1e-4;
    const LidarMultiFrameDriver withNoise(scene.world,scattering,profile,plan,scene.sensorPose,noisy,seq);
    const auto n0=withNoise.SimulateFrame(0),n1=withNoise.SimulateFrame(1);
    Check("multi frame enabled noise generates fresh measured statistics",n0.pulses[0].reception.baseline.sigmaCodes!=n1.pulses[0].reception.baseline.sigmaCodes);
    const HittableList empty;
    const auto miss=LidarMultiFrameDriver(empty,scattering,profile,plan,scene.sensorPose,receiver,seq).SimulateFrame(1);
    Check("multi frame empty scene retains frame and misses without points",miss.cloud.frameId==21 && miss.cloud.points.empty() && miss.pulses.size()==64);
    auto bad=seq;bad.frameCount=0;
    Check("multi frame zero frame count rejected",Throws<std::invalid_argument>([&]{LidarMultiFrameDriver d(scene.world,scattering,profile,plan,scene.sensorPose,receiver,bad);}));
    bad=seq;bad.framePeriodSeconds=.01;
    Check("multi frame overlapping firing windows rejected",Throws<std::invalid_argument>([&]{LidarMultiFrameDriver d(scene.world,scattering,profile,plan,scene.sensorPose,receiver,bad);}));
    bad=seq;bad.firstFrameId=std::numeric_limits<std::uint64_t>::max();
    Check("multi frame frame id overflow rejected",Throws<std::invalid_argument>([&]{LidarMultiFrameDriver d(scene.world,scattering,profile,plan,scene.sensorPose,receiver,bad);}));
    Events extreme;extreme.events={plan.Firing(0)};extreme.events[0].pulseId=std::numeric_limits<std::uint64_t>::max();
    Check("multi frame pulse id overflow rejected",Throws<std::invalid_argument>([&]{LidarMultiFrameDriver d(scene.world,scattering,profile,extreme,scene.sensorPose,receiver,seq);}));
    extreme.events={plan.Firing(0),plan.Firing(1)};extreme.events[0].pulseId=2;extreme.events[1].pulseId=100;
    const LidarMultiFrameDriver sparse(scene.world,scattering,profile,extreme,scene.sensorPose,receiver,seq);
    Check("multi frame arbitrary id plan uses range stride not event count",sparse.PulseIdStride()==99 && sparse.SimulateFrame(1).pulses[0].scan.pulse.PulseId()==101);
    Events fine;fine.events={plan.Firing(0),plan.Firing(1)};
    fine.events[1].emissionTimeSeconds=fine.events[0].emissionTimeSeconds+1e-12;
    const LidarMultiFrameConfig longClock{2,1e5,0};
    Check("multi frame rejects distinct firing times collapsing later",Throws<std::invalid_argument>([&]{LidarMultiFrameDriver d(scene.world,scattering,profile,fine,scene.sensorPose,receiver,longClock);}));
    extreme.events={plan.Firing(0)};extreme.events[0].pulseId=std::numeric_limits<std::uint64_t>::max();
    const auto maximal=LidarMultiFrameDriver(scene.world,scattering,profile,extreme,scene.sensorPose,receiver,{1,.1,0}).SimulateFrame(0);
    Check("multi frame single frame accepts maximum valid pulse id",maximal.pulses[0].scan.pulse.PulseId()==std::numeric_limits<std::uint64_t>::max());

    std::ostringstream out;LidarPcdWriter::Write(out,second.cloud);const auto pcd=out.str();const auto rows=ParseRows(pcd);
    Check("PCD standard ordered header and ASCII encoding",pcd.find("VERSION .7\nFIELDS x y z intensity ring time")!=std::string::npos && pcd.find("DATA ascii\n")!=std::string::npos);
    Check("PCD unorganized dimensions match valid point count",pcd.find("WIDTH 38\nHEIGHT 1\n")!=std::string::npos && pcd.find("POINTS 38\nDATA ascii\n")!=std::string::npos && rows.size()==38);
    Check("PCD contains frame metadata and coordinate semantics",pcd.find("# frame_id 21\n")!=std::string::npos && pcd.find("world_y_up_m")!=std::string::npos && pcd.find("ADC_codes_not_reflectivity")!=std::string::npos);
    bool roundtrip=true,relative=true;
    for(std::size_t i=0;i<rows.size();++i) {
        const auto& p=second.cloud.points[i];const auto& r=rows[i];
        roundtrip=roundtrip && r.x==p.positionWorld.x && r.y==p.positionWorld.y && r.z==p.positionWorld.z && r.ring==p.channelId && r.range==p.rangeMeters && r.amplitude==p.peakAboveBaselineCodes && r.intensity==float(p.peakAboveBaselineCodes);
        relative=relative && r.time==p.emissionTimeSeconds-second.cloud.startTimeSeconds && r.time>=0 && r.time<.1;
    }
    Check("PCD ASCII fields independently parse and preserve data",roundtrip);
    Check("PCD per point time is emission offset in seconds",relative);
    Check("PCD actual sensor viewpoint encoded",pcd.find("VIEWPOINT 0 1.5 0 1 0 0 0\n")!=std::string::npos);
    auto large=first.cloud;large.points.resize(1);large.points[0].pulseId=(std::uint64_t(1)<<60)+9;
    large.points[0].channelId=70000;large.points[0].peakOutOfRange=true;large.points[0].hasAdcOutOfRangeSamples=true;
    out.str("");out.clear();LidarPcdWriter::Write(out,large);const auto high=ParseRows(out.str()).front();
    Check("PCD split uint32 fields preserve full uint64 pulse id",(std::uint64_t(high.high)<<32 | high.low)==large.points[0].pulseId);
    Check("PCD ring does not truncate large channel ids",high.ring==70000);
    Check("PCD quality flags and range source retained",high.flags==5 && high.source==unsigned(LidarPointRangeSource::Corrected));
    std::ostringstream emptyOut;LidarPcdWriter::Write(emptyOut,miss.cloud);
    Check("PCD empty frame has zero points no fake origin",emptyOut.str().find("WIDTH 0\nHEIGHT 1\n")!=std::string::npos && ParseRows(emptyOut.str()).empty());
    std::ostringstream localized;localized.imbue(std::locale(std::locale::classic(),new CommaDecimal));
    LidarPcdWriter::Write(localized,second.cloud);
    Check("PCD numeric encoding independent of caller decimal locale",localized.str()==pcd);
    auto invalid=first.cloud;invalid.points[0].positionWorld.x=std::numeric_limits<float>::quiet_NaN();
    std::ostringstream unchanged;unchanged<<"prefix";
    Check("PCD rejects invalid points before modifying output",Throws<std::invalid_argument>([&]{LidarPcdWriter::Write(unchanged,invalid);}) && unchanged.str()=="prefix");
    invalid=first.cloud;invalid.points[0].emissionTimeSeconds=invalid.endTimeSeconds;
    Check("PCD rejects point at exclusive next frame boundary",Throws<std::invalid_argument>([&]{LidarPcdWriter::Write(out,invalid);}));
    invalid=first.cloud;invalid.points[1].pulseId=invalid.points[0].pulseId;
    Check("PCD rejects duplicate pulse ids in point frame",Throws<std::invalid_argument>([&]{LidarPcdWriter::Write(out,invalid);}));
    const auto path=std::filesystem::current_path()/("OceanRenderEngine-PcdAcceptance-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".pcd");
    LidarPcdWriter::Save(path,second.cloud);std::ifstream input(path);std::ostringstream disk;disk<<input.rdbuf();input.close();
    Check("PCD disk save matches validated stream output",disk.str()==pcd);
    Check("PCD refuses unrequested file overwrite",Throws<std::runtime_error>([&]{LidarPcdWriter::Save(path,first.cloud);}));
    LidarPcdWriter::Save(path,first.cloud,true);
    Check("PCD explicit overwrite supported",std::filesystem::file_size(path)>0);
    std::filesystem::remove(path); // 仅删除本测试刚创建的唯一临时文件。
}
