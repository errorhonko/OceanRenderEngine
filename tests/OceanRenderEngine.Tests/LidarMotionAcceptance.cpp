#include "lidar/scan/LidarMultiFrameDriver.h"
#include "lidar/output/LidarPcdWriter.h"
#include "geometry/primitives/Triangle.h"
#include "core/ray/HittableList.h"
#include "reflection/lidar/LambertianLidarScattering.h"
#include "lidar/emitter/pulse/GaussianPulseProfile.h"
#include <iostream>
#include <sstream>

namespace {
void Check(const char* name,bool ok){if(!ok)throw std::runtime_error(std::string("[FAIL] ")+name);std::cout<<"[PASS] "<<name<<'\n';}
bool Near(double a,double b,double eps=1e-5){return std::abs(a-b)<eps;}
template<class E,class F>bool Throws(F f){try{f();}catch(const E&){return true;}return false;}
LidarScanPoseConfig Identity(Vector3f origin={0,0,0}){return {origin,{0,0,1},{0,1,0}};}
LidarPoseKeyframe Yaw(double t,double angle){return {t,{{0,0,0},{float(std::sin(angle)),0,float(std::cos(angle))},{0,1,0}}};}
class Three final:public LidarScanPlan{
public:
    std::size_t Size()const override{return 3;}
    LidarFiringEvent Firing(std::size_t i)const override{
        if(i>=3)throw std::out_of_range("Index");
        return {50+std::uint64_t(i),7,1+double(i)*.025,{0,0,0},{0,0,1},std::nullopt};
    }
};
HittableList Wall(){
    HittableList world;const Vector3f a(-100,-100,10),b(-100,100,10),c(100,-100,10),d(100,100,10),n(0,0,-1),uv(0,0,0);
    world.add(std::make_shared<Triangle>(a,b,c,n,n,n,uv,uv,uv,nullptr));
    world.add(std::make_shared<Triangle>(c,b,d,n,n,n,uv,uv,uv,nullptr));return world;
}
LidarScanDriverConfig Receiver(){
    LidarScanDriverConfig c;c.pulseEnergyJ=1e-8f;c.samplesPerPulse=3;c.reception.noiseSigmaVoltageV=0;c.reception.adcBitCount=16;
    c.reception.noiseWindow={0,10e-9};c.reception.searchWindow={20e-9,150e-9};
    c.reception.rangeMethod=LidarReceiverRangeMethod::Centroid;c.reception.calibration=LidarRangeCalibration(2e-9);
    c.rangeSource=LidarPointRangeSource::Corrected;return c;
}
std::vector<Vector3f> Positions(const std::string& pcd){
    std::istringstream in(pcd);std::string line;while(std::getline(in,line)&&line!="DATA ascii"){}
    std::vector<Vector3f> result;while(std::getline(in,line)){if(line.empty())continue;
        std::istringstream row(line);Vector3f p;if(!(row>>p.x>>p.y>>p.z))throw std::runtime_error("PCD position parse failed.");result.push_back(p);}
    return result;
}
}

void RunLidarMotionAcceptanceTests(){
    constexpr double anglePi=3.14159265358979323846;
    const LidarPoseTrajectory translation({{1,Identity()},{1.2,Identity({0,0,2})}});
    Check("motion keyframe endpoints exactly preserved",translation.PoseAt(1).originWorld.z==0&&translation.PoseAt(1.2).originWorld.z==2);
    Check("motion position interpolated in simulation time",Near(translation.PoseAt(1.1).originWorld.z,1));
    Check("motion coverage is explicit",translation.StartTimeSeconds()==1&&translation.EndTimeSeconds()==1.2);
    const LidarPoseTrajectory turning({Yaw(1,0),Yaw(1.2,anglePi/2)});
    const auto halfway=turning.PoseAt(1.1);
    Check("motion quaternion halfway yaw is forty five degrees",Near(halfway.forwardWorld.x,std::sqrt(.5))&&Near(halfway.forwardWorld.z,std::sqrt(.5)));
    Check("motion quaternion retains up direction",Near(halfway.upHintWorld.y,1)&&Near(halfway.upHintWorld.x,0));
    const LidarPoseTrajectory shortArc({Yaw(1,170*anglePi/180),Yaw(2,-170*anglePi/180)});
    Check("motion interpolation uses short arc across angle wrap",shortArc.PoseAt(1.5).forwardWorld.z<-.999&&std::abs(shortArc.PoseAt(1.5).forwardWorld.x)<1e-5);
    const LidarPoseTrajectory small({Yaw(1,0),Yaw(2,1e-4)});
    Check("motion nearly equal rotations stable",Near(small.PoseAt(1.5).forwardWorld.x,5e-5,1e-8));
    const LidarPoseTrajectory pitch({{1,Identity()},{2,{{0,0,0},{0,0,-1},{0,-1,0}}}});
    Check("motion half turn around X interpolates without singularity",pitch.PoseAt(1.5).forwardWorld.y<-.999);
    const LidarPoseTrajectory roll({{1,Identity()},{2,{{0,0,0},{0,0,1},{0,-1,0}}}});
    Check("motion half turn around Z interpolates without singularity",roll.PoseAt(1.5).upHintWorld.x<-.999);
    auto keys=std::vector<LidarPoseKeyframe>{{1,Identity()},{2,Identity({2,0,0})}};
    const LidarPoseTrajectory snapshot(keys);keys[1].pose.originWorld.x=999;
    Check("motion trajectory owns keyframe snapshot",snapshot.PoseAt(2).originWorld.x==2);
    Check("motion no extrapolation before first time",Throws<std::out_of_range>([&]{translation.PoseAt(.9);}));
    Check("motion no extrapolation after final time",Throws<std::out_of_range>([&]{translation.PoseAt(1.3);}));
    Check("motion nonfinite query rejected",Throws<std::invalid_argument>([&]{translation.PoseAt(std::numeric_limits<double>::quiet_NaN());}));
    Check("motion empty trajectory rejected",Throws<std::invalid_argument>([]{LidarPoseTrajectory p({});}));
    Check("motion duplicate keyframe times rejected",Throws<std::invalid_argument>([]{LidarPoseTrajectory p({{1,Identity()},{1,Identity()}});}));
    Check("motion backward keyframe times rejected",Throws<std::invalid_argument>([]{LidarPoseTrajectory p({{2,Identity()},{1,Identity()}});}));
    Check("motion invalid keyframe orientation rejected",Throws<std::invalid_argument>([]{LidarPoseTrajectory p({{1,{{0,0,0},{0,0,0},{0,1,0}}}});}));
    const auto world=Wall();const LambertianLidarScattering scattering(.5);const GaussianPulseProfile profile(4e-9);const Three plan;const auto config=Receiver();
    const LidarScanDriver driver(world,scattering,profile,plan,translation,config);
    const auto scan=driver.SimulateFrame();
    Check("motion moving sensor actual waveform chain yields three points",scan.points.size()==3);
    bool poses=true,ranges=true,worldPlane=true,raw=true,roundtrip=true;
    for(std::size_t i=0;i<3;++i){
        const auto& p=scan.points[i];const double z=double(i)*.25;
        poses=poses&&Near(scan.pulses[i].scan.pulse.OriginWorld().z,z)&&Near(scan.pulses[i].sensorPose.originWorld.z,z);
        ranges=ranges&&Near(p.rangeMeters,10-z,.05);
        worldPlane=worldPlane&&Near(p.positionWorld.z,10,.05);
        raw=raw&&p.positionSensor&&Near(p.positionSensor->z,10-z,.05);
        if(p.positionSensor){
            const auto restored=LidarScanPose(scan.pulses[i].sensorPose).ToWorld(
                {p.pulseId,p.channelId,p.emissionTimeSeconds,*p.positionSensor,{0,0,1},std::nullopt}).OriginWorld();
            roundtrip=roundtrip&&Near(restored.z,p.positionWorld.z,2e-5);
        }
    }
    Check("motion each pulse queries its own platform pose",poses);
    Check("motion measured range tracks sensor translation",ranges);
    Check("motion truth pose transformed world points stay on wall",worldPlane);
    Check("motion raw local points retain time dependent ranges",raw);
    Check("motion local world coordinate conversion round trips",roundtrip);
    Check("motion untranslated raw cloud shows scan time distortion",scan.points[0].positionSensor->z-scan.points[2].positionSensor->z>.45);
    const auto yawScan=LidarScanDriver(world,scattering,profile,plan,turning,config).SimulateFrame();
    const double yawAngle=anglePi/8;
    Check("motion actual ray tracing follows interpolated orientation",yawScan.points.size()==3&&Near(yawScan.points.back().positionWorld.x,10*std::tan(yawAngle),.05));
    Check("motion turning raw measurement stays along local beam axis",yawScan.points.back().positionSensor&&Near(yawScan.points.back().positionSensor->x,0,2e-5)&&Near(yawScan.points.back().positionSensor->z,10/std::cos(yawAngle),.05));
    const auto detached=[&]{const LidarPoseTrajectory local({{1,Identity()},{1.2,Identity({0,0,2})}});
        return LidarScanDriver(world,scattering,profile,plan,local,config);}();
    Check("motion single frame pose snapshot outlives source trajectory",Near(detached.SimulatePulse(2).sensorPose.originWorld.z,.5));
    const auto staticScan=LidarScanDriver(world,scattering,profile,plan,Identity(),config).SimulateFrame();
    const LidarPoseTrajectory still({{1,Identity()},{1.2,Identity()}});
    const auto stationary=LidarScanDriver(world,scattering,profile,plan,still,config).SimulateFrame();
    Check("motion constant trajectory agrees with fixed pose interface",staticScan.points[2].rangeMeters==stationary.points[2].rangeMeters);
    const LidarMultiFrameDriver multi(world,scattering,profile,plan,translation,config,{2,.1,0});
    const auto second=multi.SimulateFrame(1);
    Check("motion multi frame reference pose follows frame start",Near(second.cloud.sensorPose.originWorld.z,1));
    Check("motion multi frame last pulse uses later pose",Near(second.pulses.back().sensorPose.originWorld.z,1.5));
    Check("motion multi frame replay deterministic",multi.SimulateFrame(1).cloud.points[0].rangeMeters==second.cloud.points[0].rangeMeters);
    Check("motion multi frame missing trajectory coverage rejected",Throws<std::out_of_range>([&]{LidarMultiFrameDriver d(world,scattering,profile,plan,translation,config,{4,.1,0});}));
    const auto owned=[&]{const LidarPoseTrajectory local({{1,Identity()},{1.2,Identity({0,0,2})}});
        return LidarMultiFrameDriver(world,scattering,profile,plan,local,config,{2,.1,0});}();
    Check("motion multi frame owns trajectory snapshot",Near(owned.SimulateFrame(1).pulses[0].sensorPose.originWorld.z,1));
    std::ostringstream worldPcd,rawPcd;LidarPcdWriter::Write(worldPcd,second.cloud);
    LidarPcdWriter::Write(rawPcd,second.cloud,LidarPcdCoordinates::SensorAtEmission);
    const auto w=Positions(worldPcd.str()),r=Positions(rawPcd.str());
    Check("motion PCD world mode preserves true pose transformed points",Near(w[0].z,10,.05));
    Check("motion PCD raw mode preserves local measurement coordinates",Near(r[0].z,9,.05)&&Near(r.back().z,8.5,.05));
    Check("motion PCD raw header explicitly identifies coordinate semantics",rawPcd.str().find("sensor_at_emission")!=std::string::npos&&rawPcd.str().find("VIEWPOINT 0 0 0 1 0 0 0")!=std::string::npos);
    auto invalid=second.cloud;invalid.points[0].positionSensor.reset();
    Check("motion PCD raw mode refuses missing local point data",Throws<std::invalid_argument>([&]{LidarPcdWriter::Write(rawPcd,invalid,LidarPcdCoordinates::SensorAtEmission);}));
    const LidarScanPose rotated({{2,3,4},{1,0,0},{0,0,1}});
    Check("motion inverse transform handles rotated translated mounting",Near(rotated.ToSensor({5,4,6}).x,1)&&Near(rotated.ToSensor({5,4,6}).y,2)&&Near(rotated.ToSensor({5,4,6}).z,3));
}
