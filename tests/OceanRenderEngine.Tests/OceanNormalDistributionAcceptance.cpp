#include "OceanNormalDistribution.h"
#include "ElfouhailySpectrum.h"
#include "OceanFrequencyField.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

namespace
{
void Expect(const std::string& name, bool ok)
{
    if (!ok) throw std::runtime_error(name + " failed");
    std::cout << "[PASS] " << name << '\n';
}
bool Near(double a, double b, double rel = 1e-5, double abs = 1e-10)
{
    return std::fabs(a - b) <= abs + rel * std::fabs(b);
}
template <typename F> void Reject(const std::string& name, F f)
{
    bool rejected = false;
    try { f(); } catch (const std::invalid_argument&) { rejected = true; }
    Expect(name, rejected);
}
}

void RunOceanNormalDistributionAcceptanceTests()
{
    constexpr double pi = std::numbers::pi_v<double>;
    BeckmannDistribution diagonal(0.02, 0.005);
    Expect("Beckmann RMS to alpha and normal incidence",
        Near(diagonal.AlphaMajor(), 0.2) && Near(diagonal.AlphaMinor(), 0.1) &&
        Near(diagonal.D(Vector3f(0, 0, 1)), 1.0 / (pi * 0.2 * 0.1)));

    // Integrate directly in solid angle, independently of the slope-PDF formula.
    const BeckmannDistribution correlated(0.18, 0.045, 0.03);
    double mass = 0.0, meanX = 0.0, meanY = 0.0;
    double xx = 0.0, yy = 0.0, xy = 0.0;
    constexpr int nt = 1024, np = 256;
    const double dt = 0.5 * pi / nt, dp = 2.0 * pi / np;
    for (int t = 0; t < nt; ++t)
    {
        const double theta = (t + 0.5) * dt;
        const double st = std::sin(theta), ct = std::cos(theta);
        for (int p = 0; p < np; ++p)
        {
            const double phi = (p + 0.5) * dp;
            const double x = st * std::cos(phi), y = st * std::sin(phi);
            const double weight = correlated.D(Vector3f(float(x), float(y), float(ct))) * ct * st * dt * dp;
            const double sx = -x / ct, sy = -y / ct;
            mass += weight;
            meanX += sx * weight; meanY += sy * weight;
            xx += sx * sx * weight; yy += sy * sy * weight; xy += sx * sy * weight;
        }
    }
    Expect("Beckmann projected-area normalization", Near(mass, 1.0, 3e-5));
    Expect("Beckmann NDF recovers input slope moments",
        Near(xx, 0.18, 3e-5) && Near(yy, 0.045, 3e-5) && Near(xy, 0.03, 3e-5) &&
        std::fabs(meanX) < 1e-8 && std::fabs(meanY) < 1e-8);
    const double angle = correlated.PrincipalAngle();
    const double major = correlated.AlphaMajor() * correlated.AlphaMajor() / 2.0;
    const double minor = correlated.AlphaMinor() * correlated.AlphaMinor() / 2.0;
    const double c = std::cos(angle), s = std::sin(angle);
    Expect("Beckmann principal axes reconstruct covariance",
        Near(major*c*c + minor*s*s, 0.18) && Near(major*s*s + minor*c*c, 0.045) &&
        Near((major-minor)*c*s, 0.03));
    const Vector3f wm = Vector3f(0.2f, -0.1f, 1.0f).normalize();
    Expect("Beckmann normal PDF includes projection cosine",
        Near(correlated.PdfNormal(wm), correlated.D(wm) * wm.z));
    Expect("Beckmann lower hemisphere and grazing vanish",
        correlated.D(Vector3f(0, 0, -1)) == 0.0 &&
        correlated.D(Vector3f(1, 0, 0)) == 0.0 &&
        std::isfinite(correlated.D(Vector3f(1, 0, 1e-20f))));

    const OceanSlopeVariance stats{0.04, 0.01, 0.006};
    const auto flat = OceanNormalDistribution::FromWorldSlopeVarianceLinearized(stats, Vector3f(0, 1, 0));
    const double p = 0.2, q = -0.1;
    const double det = stats.varianceX * stats.varianceZ - stats.covarianceXZ * stats.covarianceXZ;
    const double quadratic = (stats.varianceZ*p*p - 2.0*stats.covarianceXZ*p*q + stats.varianceX*q*q) / det;
    const double expected = std::exp(-0.5 * quadratic) /
        (2.0*pi*std::sqrt(det)) * std::pow(1.0+p*p+q*q, 2);
    Expect("ocean flat NDF preserves world slope covariance and axis signs",
        Near(flat.DWorld(Vector3f(float(-p), 1, float(-q))), expected) &&
        Near(flat.LocalDistribution().CovarianceXY(), -stats.covarianceXZ));

    const OceanSlopeVariance rotatedStats{stats.varianceZ, stats.varianceX, -stats.covarianceXZ};
    const auto rotated = OceanNormalDistribution::FromWorldSlopeVarianceLinearized(rotatedStats, Vector3f(0, 1, 0));
    Expect("ocean NDF rotates with wind covariance",
        Near(rotated.DWorld(Vector3f(float(q), 1, float(-p))), expected));

    const auto tilted = OceanNormalDistribution::FromWorldSlopeVarianceLinearized(stats, Vector3f(-0.6f, 1, -0.3f));
    const Frame& frame = tilted.SurfaceFrame();
    const double p0 = -double(frame.z.x) / frame.z.y;
    const double q0 = -double(frame.z.z) / frame.z.y;
    const auto exactLocalSlope = [&](const Vector3f& axis, double pw, double qw)
    {
        const double numerator = -axis.x*pw + axis.y - axis.z*qw;
        const double denominator = -frame.z.x*pw + frame.z.y - frame.z.z*qw;
        return -numerator / denominator;
    };
    constexpr double eps = 1e-5;
    const double a = (exactLocalSlope(frame.x,p0+eps,q0)-exactLocalSlope(frame.x,p0-eps,q0))/(2*eps);
    const double b = (exactLocalSlope(frame.x,p0,q0+eps)-exactLocalSlope(frame.x,p0,q0-eps))/(2*eps);
    const double cc = (exactLocalSlope(frame.y,p0+eps,q0)-exactLocalSlope(frame.y,p0-eps,q0))/(2*eps);
    const double d = (exactLocalSlope(frame.y,p0,q0+eps)-exactLocalSlope(frame.y,p0,q0-eps))/(2*eps);
    const auto& local = tilted.LocalDistribution();
    Expect("ocean tilted covariance matches slope-transform derivative",
        Near(local.VarianceX(), a*a*stats.varianceX+2*a*b*stats.covarianceXZ+b*b*stats.varianceZ) &&
        Near(local.VarianceY(), cc*cc*stats.varianceX+2*cc*d*stats.covarianceXZ+d*d*stats.varianceZ) &&
        Near(local.CovarianceXY(), a*cc*stats.varianceX+(a*d+b*cc)*stats.covarianceXZ+b*d*stats.varianceZ));
    Expect("ocean NDF local frame uses resolved normal",
        Near(frame.x.dot(frame.z), 0.0, 0.0, 1e-6) &&
        Near(frame.x.cross(frame.y).dot(frame.z), 1.0) &&
        Near(tilted.DWorld(frame.z), local.D(Vector3f(0,0,1))));

    ElfouhailySpectrum sea(ElfouhailyConfig{});
    OceanFrequencyConfig config;
    config.resolution = 8; config.patchLength = 8;
    OceanFrequencyField frequency(config, [](float,float){return 0.0f;}, [](float){return 0.0f;});
    OceanHeightField height(frequency);
    const auto unresolved = IntegrateUnresolvedSlopeVariance(
        [&](double x,double z){ return sea.CartesianSpectrum(float(x),float(z)); }, height, 10000.0);
    const auto seaNormals = OceanNormalDistribution::FromWorldSlopeVarianceLinearized(unresolved, Vector3f(0,1,0));
    Expect("Elfouhaily unresolved statistics construct normal distribution",
        seaNormals.LocalDistribution().AlphaMinor() > 0.0 &&
        std::isfinite(seaNormals.DWorld(Vector3f(0,1,0))));

    Reject("Beckmann rejects zero and singular variance", []{ BeckmannDistribution v(0,0); });
    Reject("Beckmann rejects rank-one covariance", []{ BeckmannDistribution v(1,1,1); });
    Reject("Beckmann rejects indefinite covariance", []{ BeckmannDistribution v(1,1,2); });
    Reject("Beckmann rejects nonfinite covariance", []{ BeckmannDistribution v(1,1,std::numeric_limits<double>::quiet_NaN()); });
    Reject("Beckmann rejects zero normal", [&]{ correlated.D(Vector3f()); });
    Reject("ocean NDF rejects downward macro normal", [&]{
        OceanNormalDistribution::FromWorldSlopeVarianceLinearized(stats, Vector3f(0,-1,0)); });
}
