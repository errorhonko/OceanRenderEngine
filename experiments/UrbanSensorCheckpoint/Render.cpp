#include "scene/examples/HeliosUrbanStaticScene.h"
#include "render/integrators/PathIntegrator.h"
#include "render/lights/DistantLight.h"
#include "render/lights/UniformInfiniteLight.h"
#include "lidar/scan/LidarScanPlan.h"
#include "lidar/scan/LidarPoseTrajectory.h"
#include "core/sampling/IndependentSampler.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

namespace {
constexpr int Width=640, Height=360, Samples=16;
constexpr double RigPi=3.14159265358979323846;
std::string Utf8Path(const std::filesystem::path& path) {const auto s=path.generic_u8string();return {s.begin(),s.end()};}
void Check(bool value,const char* message) {
    if(!value) throw std::runtime_error(message);
    std::cout << "[PASS] " << message << std::endl;
}
struct ScanInput { LidarScanPoseConfig rigPose; std::vector<Vector3f> points; };
ScanInput ReadScan(const std::filesystem::path& file) {
    std::ifstream in(file);if(!in)throw std::runtime_error("Cannot read bound LiDAR scan.");
    ScanInput result;std::string line;bool foundPose=false,foundData=false;
    while(std::getline(in,line)) {
        std::istringstream row(line);std::string tag;row>>tag;
        if(tag=="VIEWPOINT") {
            double x,y,z,w,qx,qy,qz;
            if(!(row>>x>>y>>z>>w>>qx>>qy>>qz))throw std::runtime_error("Invalid PCD pose.");
            const double n=std::sqrt(w*w+qx*qx+qy*qy+qz*qz);
            if(!std::isfinite(n)||n<=0)throw std::runtime_error("Invalid PCD quaternion.");
            w/=n;qx/=n;qy/=n;qz/=n;
            result.rigPose.originWorld={float(x),float(y),float(z)};
            result.rigPose.forwardWorld={float(2*(qx*qz+w*qy)),float(2*(qy*qz-w*qx)),float(1-2*(qx*qx+qy*qy))};
            result.rigPose.upHintWorld={float(2*(qx*qy-w*qz)),float(1-2*(qx*qx+qz*qz)),float(2*(qy*qz+w*qx))};
            foundPose=true;
        }
        if(tag=="DATA") {std::string type;row>>type;if(type!="ascii")throw std::runtime_error("Expected ASCII PCD.");foundData=true;break;}
    }
    Check(foundPose&&foundData,"LiDAR PCD pose and data found");
    while(std::getline(in,line)) {
        if(line.empty())continue;
        std::istringstream row(line);Vector3f p;
        if(!(row>>p.x>>p.y>>p.z))throw std::runtime_error("Invalid PCD position.");
        result.points.push_back(p);
    }
    Check(!result.points.empty(),"terrain-corrected measured point cloud read without changing points");
    return result;
}

// Experiment-only pinhole adapter. The same LidarScanPose maps local X-right,
// Y-up, Z-forward directions for BOTH camera rays and LiDAR firing events.
// The relative camera-to-LiDAR transform is identity, not a second copied pose.
class RigPinholeCamera {
public:
    explicit RigPinholeCamera(const LidarScanPoseConfig& config)
        : pose(config), fx(Width/(2*std::tan(90.0*RigPi/360))),fy(fx),cx(Width/2.0),cy(Height/2.0) {}
    Ray GenerateRay(double pixelX,double pixelY) const {
        auto p=pose.ToWorld({0,0,1,{0,0,0},{float((pixelX-cx)/fx),float((cy-pixelY)/fy),1},std::nullopt});
        return Ray(p.OriginWorld(),p.CenterDirectionWorld());
    }
    bool Project(const Vector3f& world,double& pixelX,double& pixelY,double& cameraZ) const {
        const auto local=pose.ToSensor(world);cameraZ=local.z;
        if(cameraZ<=0)return false;
        pixelX=cx+fx*local.x/cameraZ;pixelY=cy-fy*local.y/cameraZ;
        return pixelX>=0&&pixelX<Width&&pixelY>=0&&pixelY<Height;
    }
    LidarScanPose pose;
    double fx,fy,cx,cy;
};

unsigned char Display(float linear) {
    // Explicit preview exposure=1 and per-channel Reinhard mapping, then sRGB.
    const double mapped=double(linear)/(1+double(linear));
    const double srgb=mapped<=0.0031308?12.92*mapped:1.055*std::pow(mapped,1/2.4)-0.055;
    return static_cast<unsigned char>(std::clamp(std::lround(srgb*255),0L,255L));
}
void SaveLinear(const std::filesystem::path& path,const std::vector<float>& rgb) {
    std::ofstream out(path,std::ios::binary);out<<"PF\n"<<Width<<' '<<Height<<"\n-1.0\n";
    for(int y=Height-1;y>=0;--y)out.write(reinterpret_cast<const char*>(rgb.data()+y*Width*3),Width*3*sizeof(float));
    out.close();Check(out.good(),"unmodified linear RGB radiance saved to PFM");
}
}

int wmain(int argc,wchar_t** argv) {
    try {
        if(argc!=4)throw std::runtime_error("Expected repository root, measured world PCD and a new output directory.");
        const auto Project=std::filesystem::absolute(argv[1]);
        const auto LidarPcd=std::filesystem::absolute(argv[2]);
        const auto output=std::filesystem::absolute(argv[3]);
        if(std::filesystem::exists(output))throw std::runtime_error("Refusing to overwrite previous render.");
        std::filesystem::create_directories(output);
        auto scan=ReadScan(LidarPcd);
        const RigPinholeCamera camera(scan.rigPose);
        const auto central=camera.GenerateRay(Width/2.0,Height/2.0);
        Check((central.orig-scan.rigPose.originWorld).norm()<1e-7f,"camera and LiDAR share the exact world origin");
        Check((central.dir-scan.rigPose.forwardWorld).norm()<1e-6f,"camera optical axis equals LiDAR zero-azimuth forward direction");
        Check(camera.GenerateRay(Width/2.0,0).dir.dot(scan.rigPose.upHintWorld)>0,"image top points toward rig up");
        Check(camera.GenerateRay(Width,Height/2.0).dir.x>0,"image right is positive rig X (not mirrored)");
        double px,py,pz;
        Check(camera.Project(central.orig+central.dir*10,px,py,pz)&&std::abs(px-camera.cx)<1e-5&&std::abs(py-camera.cy)<1e-5,
            "optical-axis point projects to the principal point");
        auto options=ObjMaterialBindingOptions::HeliosEmpiricalBaseline(Project/"data/materials/spectra/helios");
        options.createDiffuseRenderMaterials=true;
        // Only genuinely unassigned faces get an explicitly labelled neutral preview fallback.
        // Preserve the original LiDAR model; never substitute RGB Kd for NIR rho.
        options.unassignedMaterial=std::make_shared<const SurfaceMaterial>(SurfaceMaterial{
            std::make_shared<DiffuseMaterial>(Spectrum(.5f)),options.unassignedMaterial->lidarScattering});
        const auto loadStart=std::chrono::steady_clock::now();
        auto urban=HeliosUrbanStaticScene::Load(Project/"models/helios_urban_static",options,
            {HeliosUrbanGroundingMode::TerrainContact});
        const double loadSeconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-loadStart).count();
        Check(urban.triangleCount==2445341,"same urban geometry loaded with separate RGB and LiDAR materials");
        HitRecord centerHit;
        const bool centerHasHit=urban.scene->hit(central,1e-4f,std::numeric_limits<float>::infinity(),centerHit);
        Check(!centerHasHit || (centerHit.material && centerHit.lidarScattering),"central visible surface carries both RGB and LiDAR models");
        auto sampler=std::make_shared<IndependentSampler>(42);
        std::vector<std::shared_ptr<Light>> lights{
            std::make_shared<DistantLight>(Vector3f(-.45f,.8f,-.35f),Spectrum(3.0f,2.8f,2.5f)),
            std::make_shared<UniformInfiniteLight>(Spectrum(.25f,.35f,.5f))};
        // Existing Camera is fixed-pose legacy state; Render() is currently empty.
        // Generate the actual rays via the shared rig adapter above, call Li() only.
        const PathIntegrator integrator(4,*urban.scene,Camera(Width,Height),sampler,lights);
        std::vector<float> rgb(Width*Height*3);
        const auto start=std::chrono::steady_clock::now();
        double meanLuminance=0;std::size_t nonblack=0;
        for(int y=0;y<Height;++y) {
            for(int x=0;x<Width;++x) {
                IndependentSampler pixelSampler(42U^(std::uint32_t(y*Width+x)*0x9e3779b9U));
                Spectrum sum(0);
                for(int s=0;s<Samples;++s) {
                    auto u=pixelSampler.Get2D();
                    const Ray ray=camera.GenerateRay(x+((s%4)+u.x)/4.0,y+((s/4)+u.y)/4.0);
                    const auto radiance=integrator.Li(ray,pixelSampler);
                    for(float v:radiance.c)if(!std::isfinite(v)||v<0)throw std::runtime_error("Nonfinite/negative RGB radiance.");
                    sum=sum+radiance;
                }
                const Spectrum value=sum/float(Samples);
                for(int c=0;c<3;++c)rgb[(y*Width+x)*3+c]=value.c[c];
                meanLuminance+=value.y();if(!value.IsBlack())++nonblack;
            }
            if((y+1)%40==0)std::cout<<"[PROGRESS] "<<y+1<<'/'<<Height<<" rows"<<std::endl;
        }
        const double renderSeconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
        Check(nonblack>Width*Height/4,"RGB image has nontrivial finite radiance");
        SaveLinear(output/"urban_rgb_linear.pfm",rgb);
        std::ofstream ppm(output/"urban_rgb.ppm",std::ios::binary);
        ppm<<"P6\n"<<Width<<' '<<Height<<"\n255\n";
        for(float value:rgb){auto b=Display(value);ppm.write(reinterpret_cast<const char*>(&b),1);}
        ppm.close();Check(ppm.good(),"RGB preview saved with explicit display transform");
        std::ofstream projections(output/"lidar_projection.csv");
        projections<<std::setprecision(17)<<"point_index,pixel_x,pixel_y,camera_z_m,in_image,center_geometry_consistent,center_range_delta_m\n";
        std::size_t inView=0,consistent=0;
        for(std::size_t i=0;i<scan.points.size();++i) {
            double u=0,v=0,z=0;const bool inside=camera.Project(scan.points[i],u,v,z);
            bool visible=false;double delta=0;
            if(inside) {
                ++inView;const auto local=scan.points[i]-scan.rigPose.originWorld;const double r=local.norm();
                HitRecord hit;
                if(urban.scene->hit(Ray(scan.rigPose.originWorld,local),1e-4f,std::numeric_limits<float>::infinity(),hit)) {
                    delta=r-hit.t;visible=std::abs(delta)<=std::max(.15,.002*r);
                }
                if(visible)++consistent;
            }
            projections<<i<<','<<u<<','<<v<<','<<z<<','<<inside<<','<<visible<<','<<delta<<'\n';
        }
        const auto q=LidarPoseTrajectory::Quaternion(scan.rigPose);
        std::ofstream metadata(output/"camera_rig.json");
        metadata<<std::setprecision(17)<<"{\n  \"source_lidar_pcd\": \""<<Utf8Path(LidarPcd)<<"\",\n"
            <<"  \"coordinate_convention\": \"world Y-up; sensor X-right Y-up Z-forward\",\n"
            <<"  \"rig_origin_world_m\": ["<<scan.rigPose.originWorld.x<<','<<scan.rigPose.originWorld.y<<','<<scan.rigPose.originWorld.z<<"],\n"
            <<"  \"rig_orientation_wxyz\": ["<<q[0]<<','<<q[1]<<','<<q[2]<<','<<q[3]<<"],\n"
            <<"  \"camera_to_lidar_translation_m\": [0,0,0],\n  \"camera_to_lidar_orientation_wxyz\": [1,0,0,0],\n"
            <<"  \"width\": "<<Width<<", \"height\": "<<Height<<", \"fx\": "<<camera.fx<<", \"fy\": "<<camera.fy
            <<", \"cx\": "<<camera.cx<<", \"cy\": "<<camera.cy<<",\n  \"horizontal_fov_degrees\": 90,\n"
            <<"  \"source_scan_static_interval_seconds\": [1,1.1],\n  \"camera_snapshot_seconds\": 1\n}\n";
        std::ofstream summary(output/"summary.txt");
        summary<<std::setprecision(17)<<"RGB urban preview using existing PathIntegrator::Li, maxDepth=4, 16 spp, 640x360\n"
            <<"Same rig pose read from previous LiDAR PCD, identity camera-to-LiDAR extrinsics; pinhole HFOV=90 deg\n"
            <<"Existing Camera/Render not modified; independent rig-ray adapter calls existing Li()\n"
            <<"MTL Kd -> diffuse RGB BSDF; LiDAR empirical model remains separate and unchanged\n"
            <<"Unassigned faces use an explicit assumed RGB gray .5; tree MTL Kd remains its decoded black value\n"
            <<"No textures, glass transmission, metallic/coated RGB reconstruction, motion or sensor response\n"
            <<"Lighting is assumed: directional RGB (3,2.8,2.5) toward (-.45,.8,-.35); uniform environment (.25,.35,.5)\n"
            <<"Display: exposure=1, per-channel Reinhard, standard sRGB encoding; PFM preserves linear radiance\n"
            <<"load_s="<<loadSeconds<<" render_s="<<renderSeconds<<" nonblack_pixels="<<nonblack
            <<" mean_linear_luminance="<<meanLuminance/(Width*Height)<<'\n'
            <<"lidar_points="<<scan.points.size()<<" projected_in_front_view="<<inView<<" center_geometry_consistent="<<consistent<<'\n'
            <<"Overlay consistency filter abs(measured range - central geometry range)<=max(.15 m,.002*range).\n"
            <<"Filter is only for the overlay, not point-cloud modification; finite footprint returns may legitimately disagree.\n"
            <<"Single-thread render avoids shared mutable DiffuseMaterial::bxdfStorage race. No engine changes.\n";
        projections.close();metadata.close();summary.close();
        Check(projections.good()&&metadata.good()&&summary.good(),"rig metadata and projection diagnostics saved");
        std::cout<<"[RESULT] render_s="<<renderSeconds<<" in_view="<<inView<<" consistent="<<consistent<<std::endl;
        return 0;
    }catch(const std::exception& e){std::cerr<<"[FAIL] "<<e.what()<<std::endl;return 1;}
}
