#include "BVHAccel.h"
#include "ElfouhailySpectrum.h"
#include "OceanFFT.h"
#include "OceanSlopeVariance.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <complex>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double pi=std::numbers::pi_v<double>;
struct Config {
    int resolution=256, samples=20000, seeds=8;
    std::uint32_t firstSeed=42;
    double length=4, kMin=8, kMax=32, maxDistance=1, offset=1e-6;
    double observationLength=0; // 0 preserves the original L/4 window.
};
double ObservationLength(const Config& c) {
    return c.observationLength==0 ? c.length/4 : c.observationLength;
}
struct V {
    double x,y,z;
    V operator+(V b) const {return {x+b.x,y+b.y,z+b.z};}
    V operator-(V b) const {return {x-b.x,y-b.y,z-b.z};}
    V operator*(double a) const {return {a*x,a*y,a*z};}
    double Dot(V b) const {return x*b.x+y*b.y+z*b.z;}
    V Cross(V b) const {return {y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x};}
    double Length() const {return std::hypot(x,y,z);}
    V Unit() const {return *this*(1/Length());}
    Vector3f Float() const {return Vector3f(float(x),float(y),float(z));}
    static V From(const Vector3f& v) {return {v.x,v.y,v.z};}
};
double Uniform(std::mt19937_64& rng) {return (double(rng()>>12)+.5)/4503599627370496.0;}
std::uint64_t Hash(std::uint64_t x) {
    x+=0x9e3779b97f4a7c15ULL;
    x=(x^(x>>30))*0xbf58476d1ce4e5b9ULL;
    x=(x^(x>>27))*0x94d049bb133111ebULL;
    return x^(x>>31);
}
struct Mesh {
    std::vector<Vector3f> vertices;
    std::vector<std::array<std::uint32_t,3>> triangles;
    OceanSlopeVariance discreteCov, realizedFourierCov, geometricCov;
    double maxHeight=-std::numeric_limits<double>::infinity(),maxImaginary=0;
};

// Frozen Gaussian realization, not a time animation. Each +/-k pair has
// E|H(k)|^2=Psi(k)*dk^2, H(-k)=conj(H(k)). Mode-keyed seeds preserve the same
// continuous realization when only N changes at fixed L and retained band.
std::shared_ptr<Mesh> BuildMesh(const Config& c,std::uint32_t seed) {
    auto mesh=std::make_shared<Mesh>();
    const int n=c.resolution;
    const std::size_t count=std::size_t(n)*n;
    std::vector<std::complex<float>> h(count);
    const ElfouhailySpectrum spectrum(ElfouhailyConfig{});
    const double dk=2*pi/c.length;
    const int limit=int(std::ceil(c.kMax/dk));
    auto index=[&](int x,int z){return std::size_t((z+n)%n)*n+(x+n)%n;};
    for(int iz=-limit;iz<=limit;++iz) for(int ix=-limit;ix<=limit;++ix) {
        if(iz<0 || (iz==0 && ix<=0)) continue; // one representative of each pair
        const double kx=ix*dk,kz=iz*dk,k=std::hypot(kx,kz);
        if(k<c.kMin || k>=c.kMax) continue;
        // Real height covariance requires an even PSD; current Elfouhaily PSD is even.
        const double psi=.5*(spectrum.CartesianSpectrum(float(kx),float(kz))+
            spectrum.CartesianSpectrum(float(-kx),float(-kz)));
        if(!std::isfinite(psi) || psi<0) throw std::runtime_error("Invalid height PSD.");
        const std::uint64_t key=(std::uint64_t(std::uint32_t(ix))<<32)|std::uint32_t(iz);
        std::mt19937_64 rng(Hash(key^Hash(seed)));
        const double radius=std::sqrt(-2*std::log(Uniform(rng))),phi=2*pi*Uniform(rng);
        const double scale=std::sqrt(.5*psi*dk*dk);
        const std::complex<float> value(float(scale*radius*std::cos(phi)),float(scale*radius*std::sin(phi)));
        h[index(ix,iz)]=value; h[index(-ix,-iz)]=std::conj(value);
        const double ensemble=2*psi*dk*dk,realized=2*double(std::norm(value));
        mesh->discreteCov.varianceX+=ensemble*kx*kx;
        mesh->discreteCov.varianceZ+=ensemble*kz*kz;
        mesh->discreteCov.covarianceXZ+=ensemble*kx*kz;
        mesh->realizedFourierCov.varianceX+=realized*kx*kx;
        mesh->realizedFourierCov.varianceZ+=realized*kz*kz;
        mesh->realizedFourierCov.covarianceXZ+=realized*kx*kz;
    }
    if(mesh->discreteCov.TotalVariance()<=0) throw std::runtime_error("Band contains no positive-energy discrete modes.");
    OceanFFT::Inverse2D(h,n);
    mesh->vertices.reserve(std::size_t(n+1)*(n+1));
    for(int z=0;z<=n;++z) for(int x=0;x<=n;++x) {
        const auto value=h[std::size_t(z%n)*n+x%n]*float(count);
        mesh->maxImaginary=std::max(mesh->maxImaginary,std::abs(double(value.imag())));
        const float y=value.real();
        mesh->maxHeight=std::max(mesh->maxHeight,double(y));
        mesh->vertices.emplace_back(float(-c.length*.5+x*c.length/n),y,float(-c.length*.5+z*c.length/n));
    }
    mesh->triangles.reserve(2*count);
    for(int z=0;z<n;++z) for(int x=0;x<n;++x) {
        const auto a=std::uint32_t(z*(n+1)+x),b=a+1,d=a+n+1,e=d+1;
        mesh->triangles.push_back({a,d,b});
        mesh->triangles.push_back({e,b,d});
    }
    for(const auto& t:mesh->triangles) {
        const V a=V::From(mesh->vertices[t[0]]),b=V::From(mesh->vertices[t[1]]),d=V::From(mesh->vertices[t[2]]);
        const V normal=(b-a).Cross(d-a);
        const double sx=-normal.x/normal.y,sz=-normal.z/normal.y;
        // Equal horizontal area for all regular-grid triangles. Full periodic mean slope is zero.
        mesh->geometricCov.varianceX+=sx*sx/mesh->triangles.size();
        mesh->geometricCov.varianceZ+=sz*sz/mesh->triangles.size();
        mesh->geometricCov.covarianceXZ+=sx*sz/mesh->triangles.size();
    }
    return mesh;
}

// Experiment-only primitive: use double triangle arithmetic and a scale-relative
// parallel tolerance, rather than the production MeshTriangle fixed determinant cutoff.
// Reuses the production BVH. Does not alter renderer intersection behavior.
class StudyTriangle final:public Hittable {
    std::shared_ptr<const Mesh> mesh;
    std::size_t index;
public:
    StudyTriangle(std::shared_ptr<const Mesh> m,std::size_t i):mesh(std::move(m)),index(i) {}
    Bounds3f Bounds() const override {
        const auto& t=mesh->triangles[index];
        return Union(Bounds3f(mesh->vertices[t[0]],mesh->vertices[t[1]]),mesh->vertices[t[2]]);
    }
    bool hit(const Ray& ray,float tMin,float tMax,HitRecord& rec) const override {
        const auto& t=mesh->triangles[index];
        const V a=V::From(mesh->vertices[t[0]]),e1=V::From(mesh->vertices[t[1]])-a,e2=V::From(mesh->vertices[t[2]])-a;
        const V direction=V::From(ray.dir),pv=direction.Cross(e2);
        const double det=e1.Dot(pv);
        if(std::abs(det)<=1e-12*e1.Length()*e2.Length()) return false;
        const V delta=V::From(ray.orig)-a;
        const double u=delta.Dot(pv)/det;
        if(u < -1e-10 || u > 1+1e-10) return false;
        const V qv=delta.Cross(e1);
        const double v=direction.Dot(qv)/det;
        if(v < -1e-10 || u+v > 1+1e-10) return false;
        const double distance=e2.Dot(qv)/det;
        if(distance<tMin || distance>tMax) return false;
        rec=HitRecord{}; // BVH copies the entire record, including unused scalar fields.
        rec.t=float(distance);
        return true; // Visibility only: no material or shading data needed.
    }
};
struct Point {V position,normal;};
Point SamplePoint(const Mesh& mesh,const Config& c,std::mt19937_64& rng) {
    // Uniform horizontal sampling in a centered square, independent of tile size if specified.
    const double fraction=ObservationLength(c)/c.length,start=.5-.5*fraction;
    const double gx=c.resolution*(start+fraction*Uniform(rng)),gz=c.resolution*(start+fraction*Uniform(rng));
    const int x=int(gx),z=int(gz);
    const double u=gx-x,v=gz-z;
    const auto cell=std::size_t(z)*c.resolution+x;
    const auto& tri=mesh.triangles[2*cell+(u+v>1 ? 1:0)];
    const V a=V::From(mesh.vertices[tri[0]]),b=V::From(mesh.vertices[tri[1]]),d=V::From(mesh.vertices[tri[2]]);
    const double wb=u+v<=1?v:1-v,wd=u+v<=1?u:1-u;
    return {a*(1-wb-wd)+b*wb+d*wd,(b-a).Cross(d-a).Unit()};
}
enum class Visibility {Blocked,Clear,Censored};
Visibility Trace(const Point& point,V w,const Mesh& mesh,const BVHAccel& bvh,const Config& c) {
    if(std::hypot(w.x,w.z)<1e-14) return Visibility::Clear; // upward vertical heightfield ray
    // Offset upwards in heightfield coordinates. Sensitivity must be tested separately.
    const Ray ray((point.position+V{0,c.offset,0}).Float(),w.Float());
    double end=c.maxDistance/std::hypot(double(ray.dir.x),double(ray.dir.z));
    const double bound=c.length*.5;
    for(int axis=0;axis<2;++axis) {
        const double o=axis==0?ray.orig.x:ray.orig.z,d=axis==0?ray.dir.x:ray.dir.z;
        if(d>0) end=std::min(end,(bound-o)/d);
        if(d<0) end=std::min(end,(-bound-o)/d);
    }
    HitRecord hit;
    if(bvh.hit(ray,0.0f,float(end),hit)) return Visibility::Blocked;
    // Above global mesh maximum certifies escape for periodic repetition of this tile.
    // Otherwise neither a boundary exit nor distance limit counts as visible.
    if(double(ray.orig.y)+end*ray.dir.y>mesh.maxHeight+2*c.offset) return Visibility::Clear;
    return Visibility::Censored;
}
double Smith(const OceanSlopeVariance& c,V w) {
    const double sigma=std::sqrt(std::max(0.0,w.x*w.x*c.varianceX+2*w.x*w.z*c.covarianceXZ+w.z*w.z*c.varianceZ));
    if(w.y<=0) return 0;
    if(sigma==0) return 1;
    const double a=w.y/sigma;
    return w.y/(sigma*std::exp(-.5*a*a)/std::sqrt(2*pi)+w.y*.5*std::erfc(-a/std::sqrt(2.0)));
}
// Executed only after explicit --run, before generating experimental surfaces.
void CheckIntersectionKernel() {
    for(float scale:{1.0f,1e-4f}) {
        auto mesh=std::make_shared<Mesh>();
        mesh->vertices={Vector3f(0,0,0),Vector3f(0,0,scale),Vector3f(scale,0,0)};
        mesh->triangles.push_back({0,1,2});
        auto tri=std::make_shared<StudyTriangle>(mesh,0);
        const BVHAccel bvh({tri});
        HitRecord a{},b{};
        const Ray down(Vector3f(.25f*scale,scale,.25f*scale),Vector3f(0,-1,0));
        if(!tri->hit(down,0,2*scale,a) || !bvh.hit(down,0,2*scale,b) ||
            std::abs(a.t-scale)>1e-5f*scale || std::abs(b.t-a.t)>1e-5f*scale)
            throw std::runtime_error("Intersection kernel scale check failed.");
        const Ray away(Vector3f(.25f*scale,1e-4f*scale,.25f*scale),Vector3f(1,1,0));
        if(bvh.hit(away,0,2*scale,b)) throw std::runtime_error("Intersection kernel false self hit.");
    }
    std::cout<<"[PASS] experiment triangle/BVH scale and planar self-hit checks\n";
}
struct Stats {
    std::uint64_t count=0,blocked=0,clear=0,censored=0;
    double weight=0,visibleWeight=0,unknownWeight=0;
    void Add(Visibility state,double a) {
        ++count;weight+=a;
        if(state==Visibility::Clear) {++clear;visibleWeight+=a;}
        else if(state==Visibility::Blocked) ++blocked;
        else {++censored;unknownWeight+=a;}
    }
};
constexpr int bins=108,allBin=108,halfBin=109,totalBins=110;
int NormalBin(V m) {
    const int polar=std::min(8,int(std::acos(std::clamp(m.y,0.0,1.0))*180/pi/10));
    double azimuth=std::atan2(m.z,m.x);if(azimuth<0) azimuth+=2*pi;
    return polar*12+std::min(11,int(azimuth*180/pi/30));
}
std::ofstream Output(const std::filesystem::path& path) {
    std::ofstream file(path);
    file.exceptions(std::ios::badbit|std::ios::failbit);
    file<<std::setprecision(17);
    return file;
}
void Validate(const Config& c) {
    if(!std::isfinite(c.observationLength) || c.observationLength<0 || ObservationLength(c)>=c.length)
        throw std::invalid_argument("Observation length must be zero (L/4 default) or positive and smaller than L.");
    if(c.resolution<32 || c.resolution>1024 || (c.resolution&(c.resolution-1))!=0 ||
        c.samples<100 || c.samples>1000000 || c.seeds<1 || c.seeds>64 ||
        c.length<=0 || c.kMin<=0 || c.kMax<=c.kMin || c.maxDistance<=0 || c.offset<=0 ||
        c.kMax>=pi*c.resolution/c.length || c.offset>=.01*c.length/c.resolution ||
        std::uint64_t(c.firstSeed)+c.seeds-1>std::numeric_limits<std::uint32_t>::max())
        throw std::invalid_argument("Invalid config: require power-of-two N in [32,1024], valid band below Nyquist and small positive offset.");
    if(2*pi*c.resolution/(c.length*c.kMax)<8)
        throw std::invalid_argument("Need at least 8 grid intervals per shortest wavelength; 12+ recommended.");
}
void Plan(const Config& c) {
    std::cout<<"PLAN ONLY unless --run is specified\nN="<<c.resolution<<" L="<<c.length<<"m band=["<<c.kMin<<','<<c.kMax
        <<") rad/m triangles="<<2ULL*c.resolution*c.resolution<<" seeds="<<c.seeds<<" samples/seed="<<c.samples
        <<" directions=13 max rays="<<13ULL*c.samples*c.seeds<<"\nShortest wavelength grid intervals="
        <<2*pi*c.resolution/(c.length*c.kMax)<<" max horizontal trace="<<c.maxDistance<<"m offset="<<c.offset
        <<"m observation square side="<<ObservationLength(c)<<"m\n";
}
void Run(const Config& c,const std::filesystem::path& directory) {
    if(std::filesystem::exists(directory)) throw std::runtime_error("Choose a new output directory; existing paths are never overwritten.");
    CheckIntersectionKernel();
    std::filesystem::create_directories(directory);
    auto manifest=Output(directory/"parameters.txt");
    manifest<<"OceanVisibilityStudy v2; status=started; flat macro surface; frozen realization\n"
        <<"N="<<c.resolution<<"\nL_m="<<c.length<<"\nk_min="<<c.kMin<<"\nk_max_exclusive="<<c.kMax
        <<"\nsamples="<<c.samples<<"\nseeds="<<c.seeds<<"\nfirst_seed="<<c.firstSeed<<"\nmax_horizontal_distance_m="<<c.maxDistance
        <<"\noffset_world_y_m="<<c.offset<<"\nU10=10; friction_velocity=.38; inverse_wave_age=.84; wind_direction=0\n"
        <<"gravity=9.81; water_density=1000; surface_tension=.074; ensemble Psi covariance, not continuous integral\n"
        <<"observation_length_m="<<ObservationLength(c)<<"\n"
        <<"Observation square: centered; angles from world +Y: 0,30,45,60,75,80,85 deg; azimuths +X=0, +Z=90 deg (vertical once)\n"
        <<"Normals: triangle geometric; bins=10deg polar x 30deg azimuth; half-vector diagnostic cone=3deg, monostatic h=w\n"
        <<"Unblocked but below maximum at trace end: censored; no Naive binomial confidence intervals\n";
    manifest.flush();
    auto output=Output(directory/"visibility_by_seed.csv");
    auto diagnostics=Output(directory/"surface_diagnostics.csv");
    diagnostics<<"seed,discrete_vx,discrete_vz,discrete_cxz,realized_fourier_vx,realized_fourier_vz,realized_fourier_cxz,mesh_vx,mesh_vz,mesh_cxz,max_imaginary_m,max_height_m\n";
    output<<"seed,theta_deg,azimuth_deg,bin,normal_polar_lo,normal_azimuth_lo,count,blocked,clear,censored,area_weight,visible_lower,visible_upper,unknown_area_fraction,smith_discrete,smith_mesh_diagnostic,lower_minus_smith,upper_minus_smith\n";
    struct View {double theta,phi;V w;};
    std::vector<View> views;
    for(double theta:{0.,30.,45.,60.,75.,80.,85.}) for(double phi:{0.,90.}) {
        if(theta==0 && phi==90) continue;
        const double t=theta*pi/180,f=phi*pi/180;
        views.push_back({theta,phi,V::From(Ray(Vector3f(),V{std::sin(t)*std::cos(f),std::cos(t),std::sin(t)*std::sin(f)}.Float()).dir).Unit()});
    }
    // Keep seed-level ratios rather than treating correlated points as iid.
    std::vector<std::vector<std::array<double,3>>> seedRatios(views.size()*totalBins);
    const double coneCos=std::cos(3*pi/180);
    for(int s=0;s<c.seeds;++s) {
        const auto start=std::chrono::steady_clock::now();
        const auto seed=c.firstSeed+std::uint32_t(s);
        std::cout<<"Building seed "<<seed<<"..."<<std::endl;
        const auto mesh=BuildMesh(c,seed);
        diagnostics<<seed;
        for(const auto& cov:{mesh->discreteCov,mesh->realizedFourierCov,mesh->geometricCov})
            diagnostics<<','<<cov.varianceX<<','<<cov.varianceZ<<','<<cov.covarianceXZ;
        diagnostics<<','<<mesh->maxImaginary<<','<<mesh->maxHeight<<'\n';
        std::vector<std::shared_ptr<Hittable>> primitives;primitives.reserve(mesh->triangles.size());
        for(std::size_t i=0;i<mesh->triangles.size();++i) primitives.push_back(std::make_shared<StudyTriangle>(mesh,i));
        const BVHAccel bvh(std::move(primitives));
        std::vector<std::array<Stats,totalBins>> stats(views.size());
        // Same horizontal points for convergence runs; independent of Fourier generator.
        std::mt19937_64 rng(Hash(std::uint64_t(seed)^0x632BE59BD9B4E019ULL));
        for(int i=0;i<c.samples;++i) {
            const auto point=SamplePoint(*mesh,c,rng);
            const int bin=NormalBin(point.normal);
            const double weight=1/point.normal.y;
            for(std::size_t v=0;v<views.size();++v) {
                const V w=views[v].w;
                if(w.Dot(point.normal)<=0) continue;
                const auto visible=Trace(point,w,*mesh,bvh,c);
                stats[v][bin].Add(visible,weight);stats[v][allBin].Add(visible,weight);
                if(w.Dot(point.normal)>=coneCos) stats[v][halfBin].Add(visible,weight);
            }
        }
        for(std::size_t v=0;v<views.size();++v) for(int b=0;b<totalBins;++b) {
            const auto& a=stats[v][b];
            const double nan=std::numeric_limits<double>::quiet_NaN();
            const double lower=a.weight>0?a.visibleWeight/a.weight:nan;
            const double upper=a.weight>0?(a.visibleWeight+a.unknownWeight)/a.weight:nan;
            const double prediction=Smith(mesh->discreteCov,views[v].w);
            const std::string name=b<bins?"normal_bin":(b==allBin?"all_front":"monostatic_half_cone");
            output<<seed<<','<<views[v].theta<<','<<views[v].phi<<','<<name<<','<<(b<bins?(b/12)*10:-1)<<','<<(b<bins?(b%12)*30:-1)
                <<','<<a.count<<','<<a.blocked<<','<<a.clear<<','<<a.censored<<','<<a.weight<<','<<lower<<','<<upper<<','<<(upper-lower)
                <<','<<prediction<<','<<Smith(mesh->geometricCov,views[v].w)<<','<<(lower-prediction)<<','<<(upper-prediction)<<'\n';
            if(a.count>0) seedRatios[v*totalBins+b].push_back({lower,upper,prediction});
        }
        output.flush();diagnostics.flush();
        std::cout<<"Finished seed "<<seed<<" seconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<std::endl;
    }
    auto summary=Output(directory/"summary.csv");
    summary<<"theta_deg,azimuth_deg,bin,normal_polar_lo,normal_azimuth_lo,nonempty_seeds,mean_lower,mean_upper,between_seed_se_lower,between_seed_se_upper,mean_smith\n";
    for(std::size_t v=0;v<views.size();++v) for(int b=0;b<totalBins;++b) {
        const auto& values=seedRatios[v*totalBins+b];
        const double nan=std::numeric_limits<double>::quiet_NaN();
        double lo=0,hi=0,pred=0;
        for(auto a:values) {lo+=a[0];hi+=a[1];pred+=a[2];}
        if(!values.empty()) {lo/=values.size();hi/=values.size();pred/=values.size();} else {lo=hi=pred=nan;}
        double varLo=0,varHi=0;
        for(auto a:values) {varLo+=(a[0]-lo)*(a[0]-lo);varHi+=(a[1]-hi)*(a[1]-hi);}
        const double denom=double(values.size())*(double(values.size())-1);
        summary<<views[v].theta<<','<<views[v].phi<<','<<(b<bins?"normal_bin":(b==allBin?"all_front":"monostatic_half_cone"))<<','
            <<(b<bins?(b/12)*10:-1)<<','<<(b<bins?(b%12)*30:-1)<<','<<values.size()<<','<<lo<<','<<hi<<','
            <<(values.size()>1?std::sqrt(varLo/denom):nan)<<','<<(values.size()>1?std::sqrt(varHi/denom):nan)<<','<<pred<<'\n';
    }
    output.close();diagnostics.close();summary.close();
    manifest<<"status=completed\n";manifest.close();
}
#include "PairedStudy.h"

double Number(const char* text) {
    std::size_t end=0;const std::string s(text);const double x=std::stod(s,&end);
    if(end!=s.size() || !std::isfinite(x)) throw std::invalid_argument("Expected finite numeric argument.");
    return x;
}
int Integer(double x) {
    if(x!=std::floor(x) || x<0 || x>1000000) throw std::invalid_argument("Expected integer in [0,1000000].");
    return int(x);
}
}
int main(int argc,char** argv) try {
    Config c;bool run=false;int pairedResolution=0;std::filesystem::path output;
    for(int i=1;i<argc;++i) {
        const std::string option=argv[i];
        if(option=="--run") {if(++i>=argc) throw std::invalid_argument("--run requires new output directory");run=true;output=argv[i];continue;}
        if(i+1>=argc) throw std::invalid_argument("Missing option value.");
        const double x=Number(argv[++i]);
        if(option=="--resolution") c.resolution=Integer(x);
        else if(option=="--paired-resolution") {pairedResolution=Integer(x);if(pairedResolution==0) throw std::invalid_argument("Paired resolution must be positive.");}
        else if(option=="--samples") c.samples=Integer(x);
        else if(option=="--seeds") c.seeds=Integer(x);
        else if(option=="--first-seed") c.firstSeed=std::uint32_t(Integer(x));
        else if(option=="--length") c.length=x;
        else if(option=="--observation-length") c.observationLength=x;
        else if(option=="--k-min") c.kMin=x;
        else if(option=="--k-max") c.kMax=x;
        else if(option=="--max-distance") c.maxDistance=x;
        else if(option=="--offset") c.offset=x;
        else throw std::invalid_argument("Unknown option: "+option);
    }
    Validate(c);
    if(!pairedResolution) Plan(c);
    if(pairedResolution) {Config other=c;other.resolution=pairedResolution;Validate(other);std::cout<<"Paired diagnostic N="<<c.resolution<<"/"<<pairedResolution<<"; theta=85, azimuth=0,90 only\n";}
    if(run && pairedResolution) RunPaired(c,pairedResolution,output);
    else if(run) Run(c,output);
    else std::cout<<"No simulation executed. Add --run NEW_OUTPUT_DIRECTORY after reviewing README.\n";
    return 0;
} catch(const std::exception& e) {std::cerr<<"[FAIL] "<<e.what()<<'\n';return 1;}
