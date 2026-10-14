#include "scene/examples/LidarStaticTestScene.h"
#include "lidar/scan/LidarRotaryScanPlan.h"
#include "lidar/scan/LidarScanDriver.h"
#include "reflection/lidar/LambertianLidarScattering.h"
#include "lidar/emitter/pulse/GaussianPulseProfile.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void Check(const char* name,bool ok) {
    if(!ok)throw std::runtime_error(std::string("[FAIL] ")+name);
    std::cout<<"[PASS] "<<name<<'\n';
}
bool Hit(const Hittable& world,Vector3f origin,Vector3f direction,HitRecord& r) {
    return world.hit(Ray(origin,direction),1e-4f,1000,r);
}
}
void RunLidarStaticSceneAcceptanceTests() {
    const auto scene=CreateLidarStaticTestScene();HitRecord r;
    Check("static scene contains twenty eight triangles",scene.world.objects.size()==28);
    Check("static scene central box occludes distant wall",Hit(scene.world,{0,1.5f,0},{0,0,1},r) && std::abs(r.point.z-5)<1e-5 && r.geometricNormal.z<-.99f);
    Check("static scene unoccluded ray reaches twelve meter wall",Hit(scene.world,{3,1.5f,0},{0,0,1},r) && std::abs(r.point.z-12)<1e-5);
    Check("static scene upward facing ground at zero height",Hit(scene.world,{0,1.5f,0},{0,-1,0},r) && std::abs(r.point.y)<1e-5 && r.geometricNormal.y>.99f);
    Check("static scene side box front at eight meters",Hit(scene.world,{5,1.5f,0},{0,0,1},r) && std::abs(r.point.z-8)<1e-5);
    Check("static scene box top winding faces upward",Hit(scene.world,{0,4,6},{0,-1,0},r) && std::abs(r.point.y-2)<1e-5 && r.geometricNormal.y>.99f);
    Check("static scene box back winding faces outward",Hit(scene.world,{0,1,9},{0,0,-1},r) && std::abs(r.point.z-7)<1e-5 && r.geometricNormal.z>.99f);
    Check("static scene box side winding faces outward",Hit(scene.world,{3,1,6},{-1,0,0},r) && std::abs(r.point.x-1)<1e-5 && r.geometricNormal.x>.99f);
    Check("static scene sky direction has no fake target",!Hit(scene.world,{0,1.5f,0},{0,1,0},r));
    constexpr double pi=3.14159265358979323846;
    LidarRotaryScanConfig scan;
    scan.channels={{0,-15*pi/180},{1,-5*pi/180},{2,5*pi/180},{3,15*pi/180}};
    scan.azimuthSamplesPerRevolution=16;scan.startTimeSeconds=1;
    const LidarRotaryScanPlan plan(scan);const LambertianLidarScattering scattering(.5);const GaussianPulseProfile profile(4e-9);
    LidarScanDriverConfig config;config.pulseEnergyJ=1e-8f;config.samplesPerPulse=3;
    config.reception.noiseSigmaVoltageV=0;config.reception.adcBitCount=16;
    config.reception.noiseWindow={0,10e-9};config.reception.searchWindow={20e-9,150e-9};
    config.reception.rangeMethod=LidarReceiverRangeMethod::Centroid;
    config.reception.calibration=LidarRangeCalibration(2e-9);config.rangeSource=LidarPointRangeSource::Corrected;
    const LidarScanDriver driver(scene.world,scattering,profile,plan,scene.sensorPose,config);
    const auto frame=driver.SimulateFrame();
    Check("static scene complete rotary frame retains sixty four pulses",frame.pulses.size()==64);
    Check("static scene rotary cloud contains valid returns and misses",frame.points.size()>=32 && frame.points.size()<64);
    const auto& near=frame.pulses[0];const auto& wall=frame.pulses[3];
    Check("static scene measured near box range within five cm",near.point && std::abs(near.point->rangeMeters-5/std::cos(15*pi/180))<.05);
    Check("static scene measured box point is on near face",near.point && std::abs(near.point->positionWorld.z-5)<.05);
    Check("static scene upper channel sees wall above near box",wall.point && std::abs(wall.point->positionWorld.z-12)<.05);
    bool associated=true;
    for(const auto& pulse:frame.pulses)if(pulse.point)
        associated=associated && pulse.point->pulseId==pulse.scan.pulse.PulseId() && pulse.emittedRayCount==3;
    Check("static scene generated points retain pulse association",associated);
    Check("static scene pulse replay reproduces measured box range",driver.SimulatePulse(0).point->rangeMeters==near.point->rangeMeters);
}
