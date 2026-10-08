#include "OceanNormalDistribution.h"
#include "ElfouhailySpectrum.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double pi = std::numbers::pi_v<double>;
constexpr double rad = pi / 180.0;
struct V {
    double x, y, z;
    V operator+(V b) const { return {x+b.x,y+b.y,z+b.z}; }
    V operator*(double s) const { return {x*s,y*s,z*s}; }
    double Dot(V b) const { return x*b.x+y*b.y+z*b.z; }
    V Cross(V b) const { return {y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x}; }
    V Unit() const { return *this * (1.0/std::hypot(x,y,z)); }
};
// Double precision reference; same projected-world-X frame convention as production.
struct Geometry {
    double p, q;
    V n, t, b;
    Geometry(double p, double q) : p(p), q(q) {
        n = V{-p,1,-q}.Unit();
        t = (V{1,0,0}+n*(-n.x)).Unit();
        b = n.Cross(t).Unit();
    }
    V Exact(double dp, double dq) const { return V{-p-dp,1,-q-dq}.Unit(); }
    V Linear(double dp, double dq) const {
        const double sx=n.y*(t.x*dp+t.z*dq), sy=n.y*(b.x*dp+b.z*dq);
        return (n+t*(-sx)+b*(-sy)).Unit();
    }
    double Epsilon(double dp, double dq) const { return (p*dp+q*dq)/(1+p*p+q*q); }
};
double Angle(V a, V b) { const V c=a.Cross(b); return std::atan2(std::hypot(c.x,c.y,c.z),a.Dot(b))/rad; }
void Check(bool ok, const char* name) {
    if (!ok) throw std::runtime_error(name);
    std::cout << "[PASS] " << name << '\n';
}
void SelfTest() {
    const Geometry flat(0,0), tilt(1,0);
    Check(Angle(flat.Exact(.3,-.2),flat.Linear(.3,-.2))<1e-12,"flat exact equals linear");
    Check(Angle(tilt.Exact(0,.3),tilt.Linear(0,.3))<1e-12,"perpendicular perturbation is exact");
    Check(std::abs(Angle(tilt.Exact(.2,0),tilt.Linear(.2,0))-.51616423)<1e-7,"45 degree analytic example");
    const Geometry general(.6,-.3);
    const V e=general.Exact(.2,.1), l=general.Linear(.2,.1);
    const double factor=1+general.Epsilon(.2,.1);
    Check(std::abs((-e.Dot(general.t)/e.Dot(general.n))*factor+l.Dot(general.t)/l.Dot(general.n))<1e-12,
        "exact rational slope identity");
    Check(tilt.Exact(-3,0).Dot(tilt.n)<0 && std::isfinite(Angle(tilt.Exact(-3,0),tilt.Linear(-3,0))),
        "horizon crossing retains finite normal comparison");
    const OceanSlopeVariance c{.04,.01,.006};
    const auto production=OceanNormalDistribution::FromWorldSlopeVarianceLinearized(c,Vector3f(-.6f,1,.3f));
    const double a=general.n.y*general.t.x,b=general.n.y*general.t.z;
    const double d=general.n.y*general.b.x,f=general.n.y*general.b.z;
    const auto& local=production.LocalDistribution();
    Check(std::abs(local.VarianceX()-(a*a*c.varianceX+2*a*b*c.covarianceXZ+b*b*c.varianceZ))<1e-8 &&
          std::abs(local.VarianceY()-(d*d*c.varianceX+2*d*f*c.covarianceXZ+f*f*c.varianceZ))<1e-8 &&
          std::abs(local.CovarianceXY()-(a*d*c.varianceX+(a*f+b*d)*c.covarianceXZ+b*f*c.varianceZ))<1e-8,
          "double reference agrees with production covariance");
}
// Explicit Box-Muller: fixed engine sequence, no implementation-defined normal_distribution.
std::vector<std::array<double,2>> Samples(std::size_t count, std::uint64_t seed) {
    std::mt19937_64 rng(seed);
    auto uniform=[&] { return (double(rng()>>12)+.5)/4503599627370496.0; };
    std::vector<std::array<double,2>> result;
    result.reserve(count);
    for (std::size_t i=0;i<count;++i) {
        const double r=std::sqrt(-2*std::log(uniform())), phi=2*pi*uniform();
        result.push_back({r*std::cos(phi),r*std::sin(phi)});
    }
    return result;
}
double Quantile(const std::vector<double>& sorted, double p) {
    const double pos=p*double(sorted.size()-1);
    const auto lo=static_cast<std::size_t>(pos), hi=std::min(lo+1,sorted.size()-1);
    return sorted[lo]+(pos-double(lo))*(sorted[hi]-sorted[lo]);
}
std::uint64_t ParseInteger(const char* arg) {
    const std::string text(arg);
    if (text.empty() || text.find_first_not_of("0123456789")!=std::string::npos)
        throw std::invalid_argument("Expected unsigned integer.");
    return std::stoull(text);
}
}

int main(int argc, char** argv) try {
    SelfTest();
    if (argc==2 && std::string(argv[1])=="--self-test") return 0;
    if (argc<2 || argc>4) {
        std::cerr << "Usage: OceanNormalStudy output.csv [samples=100000] [seed=42]\n"
                     "       OceanNormalStudy --self-test\n";
        return 2;
    }
    const auto count=argc>2?ParseInteger(argv[2]):100000;
    const auto seed=argc>3?ParseInteger(argv[3]):42;
    if (count<100 || count>10000000) throw std::invalid_argument("Samples must be in [100,10000000].");
    const std::filesystem::path path(argv[1]);
    if (std::filesystem::exists(path)) throw std::runtime_error("Output exists; choose a new filename.");
    const auto samples=Samples(static_cast<std::size_t>(count),seed);
    std::ofstream out(path);
    if (!out) throw std::runtime_error("Cannot open CSV output.");
    out << std::setprecision(17);
    out << "model,u10_mps,friction_velocity_mps,inverse_wave_age,wind_direction_rad,kc_rad_m,kmax_rad_m,radial_samples,angular_samples,variance_x,variance_z,covariance_xz,tilt_deg,azimuth_deg,samples,seed,epsilon_rms,angle_mean_deg,angle_p50_deg,angle_p95_deg,angle_p99_deg,angle_max_deg,lower_fraction,lower_se,cone_polar_deg,cone_azimuth_deg,cone_half_angle_deg,cone_exact,cone_linear,cone_delta,cone_delta_paired_se\n";
    const ElfouhailyConfig config{};
    const ElfouhailySpectrum spectrum(config);
    constexpr double kmax=10000, coneHalf=2;
    constexpr int radial=512, angular=128;
    const OceanCartesianSpectrum psi=[&](double x,double z) { return spectrum.CartesianSpectrum(float(x),float(z)); };
    std::size_t cases=0;
    for (double kc : {1.0,4.0,16.0}) {
        const auto cov=IntegrateSlopeVariance(psi,kc,kmax,radial,angular);
        const BeckmannDistribution validate(cov.varianceX,cov.varianceZ,cov.covarianceXZ);
        const double sx=std::sqrt(cov.varianceX), sy=std::sqrt(cov.varianceZ);
        const double rho=cov.covarianceXZ/(sx*sy), residual=std::sqrt((1-rho)*(1+rho));
        for (double tilt : {0.0,15.0,30.0,45.0,60.0,75.0,80.0})
        for (double azimuth : {0.0,45.0,90.0}) {
            const Geometry g(std::tan(tilt*rad)*std::cos(azimuth*rad),std::tan(tilt*rad)*std::sin(azimuth*rad));
            // Diagnostic cones in macro-local coordinates, not a radiometric estimator.
            constexpr std::array<double,5> polar{0,10,10,10,10}, azi{0,0,90,180,270};
            std::array<V,5> targets;
            for (std::size_t j=0;j<5;++j)
                targets[j]=g.n*std::cos(polar[j]*rad)+(g.t*std::cos(azi[j]*rad)+g.b*std::sin(azi[j]*rad))*std::sin(polar[j]*rad);
            std::array<double,5> exactCounts{},linearCounts{},differentCounts{};
            std::vector<double> angles; angles.reserve(samples.size());
            double angleSum=0, lower=0;
            for (const auto& z:samples) {
                const double dp=sx*z[0],dq=sy*(rho*z[0]+residual*z[1]);
                const V e=g.Exact(dp,dq), l=g.Linear(dp,dq);
                const double angle=Angle(e,l); angles.push_back(angle); angleSum+=angle;
                lower+=e.Dot(g.n)<=0?1:0;
                for (std::size_t j=0;j<5;++j) {
                    const bool insideE=e.Dot(targets[j])>=std::cos(coneHalf*rad);
                    const bool insideL=l.Dot(targets[j])>=std::cos(coneHalf*rad);
                    exactCounts[j]+=insideE; linearCounts[j]+=insideL; differentCounts[j]+=(insideE!=insideL);
                }
            }
            std::sort(angles.begin(),angles.end());
            const double n=double(count), lowerP=lower/n;
            const double epsilonRms=std::sqrt(g.p*g.p*cov.varianceX+2*g.p*g.q*cov.covarianceXZ+g.q*g.q*cov.varianceZ)/(1+g.p*g.p+g.q*g.q);
            for (std::size_t j=0;j<5;++j) {
                const double delta=(linearCounts[j]-exactCounts[j])/n;
                const double se=std::sqrt(std::max(0.0,differentCounts[j]/n-delta*delta)/(n-1));
                out << "Elfouhaily," << config.windSpeed10m << ',' << config.frictionVelocity << ',' << config.inverseWaveAge << ',' << config.windDirection << ','
                    << kc << ',' << kmax << ',' << radial << ',' << angular << ',' << cov.varianceX << ',' << cov.varianceZ << ',' << cov.covarianceXZ << ','
                    << tilt << ',' << azimuth << ',' << count << ',' << seed << ',' << epsilonRms << ',' << angleSum/n << ','
                    << Quantile(angles,.5) << ',' << Quantile(angles,.95) << ',' << Quantile(angles,.99) << ',' << angles.back() << ','
                    << lowerP << ',' << std::sqrt(lowerP*(1-lowerP)/n) << ',' << polar[j] << ',' << azi[j] << ',' << coneHalf << ','
                    << exactCounts[j]/n << ',' << linearCounts[j]/n << ',' << delta << ',' << se << '\n';
            }
            ++cases;
        }
    }
    out.flush();
    if (!out) throw std::runtime_error("CSV write failed.");
    std::cout << "Wrote " << cases << " cases (5 cones each) to " << path << '\n';
    return 0;
} catch (const std::exception& e) {
    std::cerr << "[FAIL] " << e.what() << '\n';
    return 1;
}
