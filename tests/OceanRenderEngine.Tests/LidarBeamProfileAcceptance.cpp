#include "lidar/emitter/beam/LidarBeamProfile.h"
#include "lidar/emitter/beam/GaussianLidarBeamProfile.h"
#include "lidar/emitter/beam/DeltaLidarBeamProfile.h"
#include "lidar/emitter/beam/UniformConeLidarBeamProfile.h"
#include "lidar/emitter/LaserEmitter.h"
#include "core/sampling/IndependentSampler.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace {
constexpr double TestPi = 3.14159265358979323846;

static_assert(std::is_abstract_v<LidarBeamProfile>);
static_assert(std::has_virtual_destructor_v<LidarBeamProfile>);
static_assert(std::is_same_v<decltype(LidarBeamDirectionSample{}.pdfPerSr), double>);
static_assert(std::is_same_v<decltype(LidarBeamDirectionSample{}.energyWeight), double>);

void Check(const char* name, bool ok)
{
    if (!ok)
        throw std::runtime_error(std::string("[FAIL] ") + name);
    std::cout << "[PASS] " << name << '\n';
}

bool Near(double a, double b, double tolerance = 1e-12)
{
    return std::abs(a - b) <= tolerance;
}

bool UnitDirection(const Vector3f& direction)
{
    const double lengthSquared = double(direction.x) * direction.x +
        double(direction.y) * direction.y + double(direction.z) * direction.z;
    return std::isfinite(lengthSquared) && Near(lengthSquared, 1.0, 2e-7);
}

// Test fixtures only: these are not production beam models or Gaussian models.
class DeltaFixture final : public LidarBeamProfile
{
public:
    explicit DeltaFixture(bool& destroyed) : destroyed(destroyed) {}
    ~DeltaFixture() override { destroyed = true; }

    LidarBeamDirectionSample SampleDirection(const Point2f&) const override
    {
        return { Vector3f(0, 0, 1), 0.0, 1.0, true };
    }
    double EnergyDensityPerSr(const Vector3f&) const override { return 0.0; }
    double PdfDirection(const Vector3f&) const override { return 0.0; }
    bool IsDeltaDirection() const override { return true; }

private:
    bool& destroyed;
};

// q = 1/(2*pi): uniform solid-angle sampling of the forward hemisphere.
// b = (1+z)/(3*pi): a different, normalized physical energy distribution.
// No surface cosine is added here; b is already energy per solid angle.
class ContinuousFixture final : public LidarBeamProfile
{
public:
    LidarBeamDirectionSample SampleDirection(const Point2f& u) const override
    {
        const double z = u.x;
        const double radius = std::sqrt(std::max(0.0, 1.0 - z * z));
        const double phi = 2.0 * TestPi * u.y;
        const Vector3f direction(float(radius * std::cos(phi)),
            float(radius * std::sin(phi)), float(z));
        const double pdf = PdfDirection(direction);
        return { direction, pdf, EnergyDensityPerSr(direction) / pdf, false };
    }
    double EnergyDensityPerSr(const Vector3f& direction) const override
    {
        return direction.z < 0 ? 0.0 : (1.0 + direction.z) / (3.0 * TestPi);
    }
    double PdfDirection(const Vector3f& direction) const override
    {
        return direction.z < 0 ? 0.0 : 1.0 / (2.0 * TestPi);
    }
    bool IsDeltaDirection() const override { return false; }
};
}

void RunLidarBeamProfileAcceptanceTests()
{
    const LidarBeamDirectionSample defaults;
    Check("beam interface default local axis is positive Z",
        defaults.directionLocal.x == 0 && defaults.directionLocal.y == 0 &&
        defaults.directionLocal.z == 1);
    Check("beam interface default weights are explicit",
        defaults.pdfPerSr == 0 && defaults.energyWeight == 1 && !defaults.deltaDirection);

    bool destroyed = false;
    {
        const std::unique_ptr<LidarBeamProfile> delta =
            std::make_unique<DeltaFixture>(destroyed);
        const auto sample = delta->SampleDirection(Point2f(0.3f, 0.7f));
        Check("beam interface delta dispatch and flag",
            delta->IsDeltaDirection() && sample.deltaDirection);
        Check("beam interface delta direction is a unit local axis",
            UnitDirection(sample.directionLocal) && sample.directionLocal.z == 1);
        Check("beam interface delta has no ordinary direction density",
            sample.pdfPerSr == 0 && delta->PdfDirection(sample.directionLocal) == 0 &&
            delta->EnergyDensityPerSr(sample.directionLocal) == 0);
        Check("beam interface delta energy weight is dimensionless unity",
            sample.energyWeight == 1);
    }
    Check("beam interface base destruction invokes derived destructor", destroyed);

    const ContinuousFixture continuous;
    const LidarBeamProfile& beam = continuous;
    const auto sample = beam.SampleDirection(Point2f(0.25f, 0.5f));
    Check("beam interface continuous dispatch and flag",
        !beam.IsDeltaDirection() && !sample.deltaDirection);
    Check("beam interface continuous direction is finite and normalized",
        UnitDirection(sample.directionLocal) && sample.directionLocal.z == 0.25f);
    Check("beam interface sampled PDF matches queried solid angle PDF",
        sample.pdfPerSr > 0 && std::isfinite(sample.pdfPerSr) &&
        Near(sample.pdfPerSr, beam.PdfDirection(sample.directionLocal)));
    Check("beam interface physical density can differ from sampling PDF",
        !Near(beam.EnergyDensityPerSr(sample.directionLocal), sample.pdfPerSr));
    Check("beam interface energy weight is physical density divided by PDF",
        Near(sample.energyWeight, 5.0 / 6.0) &&
        Near(sample.energyWeight * sample.pdfPerSr,
            beam.EnergyDensityPerSr(sample.directionLocal)));
    Check("beam interface both densities vanish outside forward support",
        beam.PdfDirection(Vector3f(0, 0, -1)) == 0 &&
        beam.EnergyDensityPerSr(Vector3f(0, 0, -1)) == 0);
    Check("beam interface repeated unit samples are deterministic",
        beam.SampleDirection(Point2f(0.25f, 0.5f)).energyWeight == sample.energyWeight);

    bool endpointsValid = true;
    for (const Point2f u : { Point2f(0, 0), Point2f(0, 1), Point2f(1, 0), Point2f(1, 1) }) {
        const auto endpoint = beam.SampleDirection(u);
        endpointsValid = endpointsValid && UnitDirection(endpoint.directionLocal) &&
            std::isfinite(endpoint.energyWeight) && std::isfinite(endpoint.pdfPerSr);
    }
    Check("beam interface fixture handles closed unit square endpoints", endpointsValid);

    // Deterministic midpoint quadrature: dOmega = dPhi dz on the hemisphere.
    // For b above, integral b*dOmega = 1 and integral z*b*dOmega = 5/9.
    // This tests the interface weighting convention, not a Gaussian implementation.
    double densityIntegral = 0, pdfIntegral = 0, meanWeight = 0, weightedZ = 0;
    constexpr int ZCount = 256;
    constexpr int PhiCount = 32;
    constexpr int SampleCount = ZCount * PhiCount;
    const double solidAnglePerCell = 2.0 * TestPi / SampleCount;
    for (int iz = 0; iz < ZCount; ++iz) {
        for (int iphi = 0; iphi < PhiCount; ++iphi) {
            const auto value = beam.SampleDirection(Point2f(
                float((iz + 0.5) / ZCount), float((iphi + 0.5) / PhiCount)));
            densityIntegral += beam.EnergyDensityPerSr(value.directionLocal) * solidAnglePerCell;
            pdfIntegral += beam.PdfDirection(value.directionLocal) * solidAnglePerCell;
            meanWeight += value.energyWeight / SampleCount;
            weightedZ += value.energyWeight * value.directionLocal.z / SampleCount;
        }
    }
    Check("beam interface fixture physical density integrates to unity", Near(densityIntegral, 1, 1e-10));
    Check("beam interface fixture PDF integrates to unity", Near(pdfIntegral, 1, 1e-10));
    Check("beam interface weighting preserves total energy without extra cosine", Near(meanWeight, 1, 1e-10));
    Check("beam interface weighted directional response matches analytic integral", Near(weightedZ, 5.0 / 9.0, 1e-6));
}

namespace {
template<class F>
bool RejectsGaussianArgument(F&& action)
{
    try { action(); }
    catch (const std::invalid_argument&) { return true; }
    return false;
}

Vector3f DirectionAtSlope(double s, double phi = 0.0)
{
    const double invLength = 1.0 / std::sqrt(1.0 + s * s);
    return Vector3f(float(s * std::cos(phi) * invLength),
        float(s * std::sin(phi) * invLength), float(invLength));
}

double IntegrateGaussianDensity(const GaussianLidarBeamProfile& beam, double maxSlope)
{
    // Independent measure: t=s^2, so dOmega=pi*dt/(1+t)^(3/2).
    // This exercises the production direction density, not random sampling.
    constexpr int CellCount = 16384;
    const double dt = maxSlope * maxSlope / CellCount;
    double integral = 0.0;
    for (int i = 0; i < CellCount; ++i) {
        const double t = (i + 0.5) * dt;
        integral += beam.EnergyDensityPerSr(DirectionAtSlope(std::sqrt(t))) *
            TestPi * dt / std::pow(1.0 + t, 1.5);
    }
    return integral;
}
}

void RunGaussianLidarBeamDensityAcceptanceTests()
{
    const GaussianLidarBeamProfile beam(0.001, 0.003);
    const LidarBeamProfile& base = beam;
    const double a = std::tan(0.001);
    const double maxSlope = std::tan(0.003);
    const double expectedZ = -std::expm1(-2 * (maxSlope / a) * (maxSlope / a));
    const double expectedCenter = 2.0 / (TestPi * a * a * expectedZ);
    const Vector3f center(0, 0, 1);
    Check("gaussian beam constructor retains angle parameters",
        beam.E2HalfAngleRadians() == 0.001 && beam.MaxHalfAngleRadians() == 0.003);
    Check("gaussian beam retained fraction matches analytic integral", Near(beam.RetainedFraction(), expectedZ));
    const GaussianLidarBeamProfile equal(0.001, 0.001);
    Check("gaussian beam equal cutoff has expected retained fraction", Near(equal.RetainedFraction(), -std::expm1(-2.0)));
    Check("gaussian beam is continuous", !base.IsDeltaDirection());
    Check("gaussian beam center density has normalized units", Near(base.EnergyDensityPerSr(center) / expectedCenter, 1, 1e-12));
    const auto offAxis = DirectionAtSlope(a);
    const double sx = double(offAxis.x) / offAxis.z;
    const double t = sx * sx;
    const double expectedOffAxis = expectedCenter * std::exp(-2 * t / (a * a)) * std::pow(1 + t, 1.5);
    Check("gaussian beam off axis density includes solid angle Jacobian", Near(base.EnergyDensityPerSr(offAxis) / expectedOffAxis, 1, 1e-12));
    Check("gaussian beam physical density equals sampling PDF", base.PdfDirection(offAxis) == base.EnergyDensityPerSr(offAxis));
    Check("gaussian beam circular symmetry", Near(base.PdfDirection(DirectionAtSlope(a, TestPi / 2)) / base.PdfDirection(offAxis), 1, 1e-6));
    Check("gaussian beam backward direction outside support", base.PdfDirection(Vector3f(0, 0, -1)) == 0);
    Check("gaussian beam horizontal direction outside support", base.PdfDirection(Vector3f(1, 0, 0)) == 0);
    Check("gaussian beam clearly beyond cutoff outside support", base.PdfDirection(DirectionAtSlope(maxSlope * 1.001)) == 0);
    Check("gaussian beam float cutoff boundary remains finite and positive",
        std::isfinite(base.PdfDirection(DirectionAtSlope(maxSlope))) && base.PdfDirection(DirectionAtSlope(maxSlope)) > 0);
    Check("gaussian beam invalid zero direction rejected", RejectsGaussianArgument([&] { base.PdfDirection(Vector3f(0, 0, 0)); }));
    Check("gaussian beam nonunit direction rejected", RejectsGaussianArgument([&] { base.PdfDirection(Vector3f(0, 0, 2)); }));
    Check("gaussian beam nonfinite direction rejected", RejectsGaussianArgument([&] { base.PdfDirection(Vector3f(std::numeric_limits<float>::quiet_NaN(), 0, 1)); }));
    Check("gaussian beam zero width rejected", RejectsGaussianArgument([] { GaussianLidarBeamProfile b(0, 0.001); }));
    Check("gaussian beam negative width rejected", RejectsGaussianArgument([] { GaussianLidarBeamProfile b(-0.001, 0.001); }));
    Check("gaussian beam cutoff below width rejected", RejectsGaussianArgument([] { GaussianLidarBeamProfile b(0.002, 0.001); }));
    Check("gaussian beam half pi cutoff rejected", RejectsGaussianArgument([] { GaussianLidarBeamProfile b(0.001, TestPi / 2); }));
    Check("gaussian beam nonfinite width rejected", RejectsGaussianArgument([] { GaussianLidarBeamProfile b(std::numeric_limits<double>::quiet_NaN(), 0.001); }));
    Check("gaussian beam infinite cutoff rejected", RejectsGaussianArgument([] { GaussianLidarBeamProfile b(0.001, std::numeric_limits<double>::infinity()); }));
    Check("gaussian beam unrepresentable width rejected", RejectsGaussianArgument([] { GaussianLidarBeamProfile b(1e-200, 0.001); }));
    Check("gaussian beam narrow direction density integrates to unity", Near(IntegrateGaussianDensity(beam, maxSlope), 1, 1e-6));
    const GaussianLidarBeamProfile broad(0.3, 0.8);
    Check("gaussian beam geometric mapping density integrates to unity", Near(IntegrateGaussianDensity(broad, std::tan(0.8)), 1, 1e-6));
    const GaussianLidarBeamProfile tiny(1e-6, 3e-6);
    Check("gaussian beam tiny cone density integrates to unity", Near(IntegrateGaussianDensity(tiny, std::tan(3e-6)), 1, 1e-6));
    Check("gaussian beam tiny cone support does not depend on rounded cosines", tiny.PdfDirection(DirectionAtSlope(std::tan(3.1e-6))) == 0);
}

namespace {
double SampleSlopeSquared(const LidarBeamDirectionSample& sample)
{
    const auto& d = sample.directionLocal;
    const double sx = double(d.x) / d.z;
    const double sy = double(d.y) / d.z;
    return sx * sx + sy * sy;
}
bool RejectsGaussianSample(const GaussianLidarBeamProfile& beam, Point2f u)
{
    try { beam.SampleDirection(u); }
    catch (const std::out_of_range&) { return true; }
    return false;
}
}

void RunGaussianLidarBeamSamplingAcceptanceTests()
{
    const GaussianLidarBeamProfile beam(0.001, 0.003);
    const LidarBeamProfile& base = beam;
    const double a = std::tan(beam.E2HalfAngleRadians());
    const double maxSlope = std::tan(beam.MaxHalfAngleRadians());
    const double aSquared = a * a;
    const auto axis = base.SampleDirection(Point2f(0, 0.7f));
    Check("gaussian sampling zero radius lies on local positive Z",
        axis.directionLocal.x == 0 && axis.directionLocal.y == 0 && axis.directionLocal.z == 1);
    const auto quarter = base.SampleDirection(Point2f(0.5f, 0.25f));
    Check("gaussian sampling quarter azimuth points toward local positive Y",
        quarter.directionLocal.y > 0 && std::abs(quarter.directionLocal.x) < 1e-10);
    Check("gaussian sampling is continuous and dimensionless equal weight",
        !quarter.deltaDirection && quarter.energyWeight == 1);
    Check("gaussian sampling reported PDF matches actual float direction",
        quarter.pdfPerSr == beam.PdfDirection(quarter.directionLocal) &&
        quarter.pdfPerSr == beam.EnergyDensityPerSr(quarter.directionLocal));
    bool inverseCorrect = true;
    for (const float u : {0.01f, 0.25f, 0.5f, 0.9f, 0.99f, 0.9999f}) {
        const auto sample = beam.SampleDirection(Point2f(u, 0.37f));
        const double cdf = -std::expm1(-2 * SampleSlopeSquared(sample) / aSquared) / beam.RetainedFraction();
        inverseCorrect = inverseCorrect && Near(cdf, u, 2e-7);
    }
    Check("gaussian sampling inverse CDF agrees with analytic encircled energy", inverseCorrect);
    bool endpointsValid = true;
    for (const Point2f u : {Point2f(0,0), Point2f(0,1), Point2f(1,0), Point2f(1,1)}) {
        const auto sample = beam.SampleDirection(u);
        endpointsValid = endpointsValid && UnitDirection(sample.directionLocal) &&
            sample.directionLocal.z > 0 && std::isfinite(sample.pdfPerSr) && sample.pdfPerSr > 0 &&
            SampleSlopeSquared(sample) <= maxSlope * maxSlope * (1 + 1e-6);
    }
    Check("gaussian sampling closed unit square endpoints are stable", endpointsValid);
    const GaussianLidarBeamProfile longTail(0.001, 0.1);
    const auto tailEndpoint = longTail.SampleDirection(Point2f(1, 1));
    Check("gaussian sampling rounded unity retained fraction avoids log zero",
        longTail.RetainedFraction() == 1 && UnitDirection(tailEndpoint.directionLocal) &&
        std::isfinite(tailEndpoint.pdfPerSr) && tailEndpoint.pdfPerSr > 0);
    Check("gaussian sampling upper endpoint maps inward in cumulative probability",
        Near(-std::expm1(-2 * SampleSlopeSquared(tailEndpoint) / aSquared),
            std::nextafter(1.0f, 0.0f), 2e-7));
    Check("gaussian sampling negative radial sample rejected", RejectsGaussianSample(beam, Point2f(-0.1f, 0.5f)));
    Check("gaussian sampling radial sample above one rejected", RejectsGaussianSample(beam, Point2f(1.1f, 0.5f)));
    Check("gaussian sampling negative azimuth sample rejected", RejectsGaussianSample(beam, Point2f(0.5f, -0.1f)));
    Check("gaussian sampling azimuth sample above one rejected", RejectsGaussianSample(beam, Point2f(0.5f, 1.1f)));
    Check("gaussian sampling nonfinite radial sample rejected", RejectsGaussianSample(beam, Point2f(std::numeric_limits<float>::quiet_NaN(), 0.5f)));
    Check("gaussian sampling infinite azimuth sample rejected", RejectsGaussianSample(beam, Point2f(0.5f, std::numeric_limits<float>::infinity())));

    IndependentSampler random(42), replay(42);
    constexpr int Count = 65536;
    int innerHalfCount = 0, innerScaleCount = 0;
    int quadrants[4] = {};
    double meanXSlope = 0, meanYSlope = 0, meanSlopeSquared = 0, weightSum = 0;
    bool valid = true, deterministic = true, pdfMatches = true;
    for (int i = 0; i < Count; ++i) {
        const auto sample = beam.SampleDirection(random.Get2D());
        const auto repeated = beam.SampleDirection(replay.Get2D());
        const auto& d = sample.directionLocal;
        const double sx = double(d.x) / d.z, sy = double(d.y) / d.z;
        const double t = sx * sx + sy * sy;
        valid = valid && UnitDirection(d) && d.z > 0 && t <= maxSlope * maxSlope * (1 + 1e-6) &&
            std::isfinite(sample.pdfPerSr) && sample.pdfPerSr > 0 && sample.energyWeight == 1;
        deterministic = deterministic && d.x == repeated.directionLocal.x && d.y == repeated.directionLocal.y &&
            d.z == repeated.directionLocal.z && sample.pdfPerSr == repeated.pdfPerSr;
        pdfMatches = pdfMatches && sample.pdfPerSr == beam.PdfDirection(d);
        innerHalfCount += t < aSquared / 4;
        innerScaleCount += t < aSquared;
        ++quadrants[(d.x < 0 ? 1 : 0) + (d.y < 0 ? 2 : 0)];
        meanXSlope += sx / Count;
        meanYSlope += sy / Count;
        meanSlopeSquared += t / Count;
        weightSum += sample.energyWeight / Count;
    }
    Check("gaussian sampling fixed seed replay is identical", deterministic);
    Check("gaussian sampling all directions have unit length positive PDF and cutoff support", valid);
    Check("gaussian sampling all returned PDFs match query", pdfMatches);
    Check("gaussian sampling half scale encircled fraction matches analytic energy",
        Near(double(innerHalfCount) / Count, -std::expm1(-0.5) / beam.RetainedFraction(), 0.005));
    Check("gaussian sampling e2 scale encircled fraction matches analytic energy",
        Near(double(innerScaleCount) / Count, -std::expm1(-2.0) / beam.RetainedFraction(), 0.005));
    bool symmetricQuadrants = true;
    for (const int count : quadrants)
        symmetricQuadrants = symmetricQuadrants && Near(double(count) / Count, 0.25, 0.007);
    Check("gaussian sampling azimuth quadrants are uniform", symmetricQuadrants);
    Check("gaussian sampling transverse slope means are zero", std::abs(meanXSlope) < a * 0.01 && std::abs(meanYSlope) < a * 0.01);
    const double T = 2 * maxSlope * maxSlope / aSquared;
    const double expectedMoment = aSquared / 2 * (1 - (T + 1) * std::exp(-T)) / beam.RetainedFraction();
    Check("gaussian sampling radial second moment matches truncated Gaussian", Near(meanSlopeSquared / expectedMoment, 1, 0.02));
    Check("gaussian sampling total energy has no duplicate weighting or sample count", Near(weightSum, 1));

    bool otherScalesValid = true;
    for (const double scale : {1e-6, 0.3}) {
        const GaussianLidarBeamProfile other(scale, 3 * scale);
        for (int i = 0; i < 256; ++i) {
            const auto sample = other.SampleDirection(Point2f(float((i + 0.5) / 256), float((i + 0.5) / 256)));
            otherScalesValid = otherScalesValid && UnitDirection(sample.directionLocal) &&
                std::isfinite(sample.pdfPerSr) && sample.pdfPerSr > 0 &&
                SampleSlopeSquared(sample) <= std::pow(std::tan(3 * scale), 2) * (1 + 1e-6);
        }
    }
    Check("gaussian sampling tiny and broad geometric test scales remain stable", otherScalesValid);
}

void RunBasicLidarBeamProfileAcceptanceTests()
{
    const DeltaLidarBeamProfile delta;
    const LidarBeamProfile& deltaBase = delta;
    const auto sample = deltaBase.SampleDirection(Point2f(1, 1));
    Check("basic beam Delta returns positive local Z and unit weight",
        sample.directionLocal.x==0 && sample.directionLocal.y==0 && sample.directionLocal.z==1 && sample.energyWeight==1);
    Check("basic beam Delta direction is probability mass not ordinary PDF",
        deltaBase.IsDeltaDirection() && sample.deltaDirection && sample.pdfPerSr==0 &&
        deltaBase.PdfDirection({0,0,1})==0 && deltaBase.EnergyDensityPerSr({0,0,1})==0);
    bool badDelta = false;
    try { delta.SampleDirection(Point2f(-.1f,0)); }
    catch(const std::out_of_range&) { badDelta=true; }
    Check("basic beam Delta validates direct samples", badDelta);

    const float angle=.1f;
    const UniformConeLidarBeamProfile cone(angle);
    const LidarBeamProfile& coneBase=cone;
    Check("basic beam uniform cone exposes width and continuous flag",cone.HalfAngleRadians()==angle && !coneBase.IsDeltaDirection());
    const double solidAngle=4*TestPi*std::pow(std::sin(double(angle)/2),2);
    Check("basic beam uniform cone PDF integrates over solid angle to unity",Near(cone.PdfDirection({0,0,1})*solidAngle,1));
    Check("basic beam uniform cone density equals sampling PDF",cone.EnergyDensityPerSr({0,0,1})==cone.PdfDirection({0,0,1}));
    const auto center=cone.SampleDirection(Point2f(0,.37f));
    Check("basic beam uniform cone radial zero returns local axis",center.directionLocal.x==0 && center.directionLocal.y==0 && center.directionLocal.z==1);
    const auto edge=cone.SampleDirection(Point2f(1,.25f));
    Check("basic beam uniform cone edge has specified half angle",
        Near(std::atan2(std::hypot(double(edge.directionLocal.x),double(edge.directionLocal.y)),double(edge.directionLocal.z)),angle,1e-8));
    Check("basic beam uniform cone float boundary accepted by PDF",cone.PdfDirection(edge.directionLocal)==edge.pdfPerSr);
    Check("basic beam uniform cone backward and horizontal directions excluded",
        cone.PdfDirection({0,0,-1})==0 && cone.PdfDirection({1,0,0})==0);
    Check("basic beam uniform cone beyond cutoff excluded",cone.PdfDirection(DirectionAtSlope(std::tan(double(angle)*1.01)))==0);
    Check("basic beam uniform cone zero angle requires Delta",RejectsGaussianArgument([]{UniformConeLidarBeamProfile b(0);}));
    Check("basic beam uniform cone negative angle rejected",RejectsGaussianArgument([]{UniformConeLidarBeamProfile b(-.1f);}));
    Check("basic beam uniform cone half pi rejected",RejectsGaussianArgument([]{UniformConeLidarBeamProfile b(float(TestPi/2));}));
    Check("basic beam uniform cone infinite angle rejected",RejectsGaussianArgument([]{UniformConeLidarBeamProfile b(std::numeric_limits<float>::infinity());}));
    Check("basic beam uniform cone invalid unit direction rejected",RejectsGaussianArgument([&]{cone.PdfDirection({0,0,2});}));
    bool badCone=false;
    try { cone.SampleDirection(Point2f(.5f,std::numeric_limits<float>::quiet_NaN())); }
    catch(const std::out_of_range&) { badCone=true; }
    Check("basic beam uniform cone nonfinite sample rejected",badCone);

    bool gridValid=true, constantPdf=true;
    double meanCos=0;
    constexpr int GridCount=4096;
    for(int i=0;i<GridCount;++i){
        const auto s=cone.SampleDirection(Point2f(float((i+.5)/GridCount),float((i%64+.5)/64)));
        gridValid=gridValid && UnitDirection(s.directionLocal) && cone.PdfDirection(s.directionLocal)>0;
        constantPdf=constantPdf && s.pdfPerSr==center.pdfPerSr && s.energyWeight==1 && !s.deltaDirection;
        meanCos+=double(s.directionLocal.z)/GridCount;
    }
    Check("basic beam uniform cone all deterministic directions stay inside support",gridValid);
    Check("basic beam uniform cone all samples have constant PDF and unit weight",constantPdf);
    Check("basic beam uniform cone mean cosine matches analytic uniform solid angle",
        Near(meanCos,(1+std::cos(double(angle)))/2,1e-7));
    const UniformConeLidarBeamProfile narrow(1e-6f);
    const auto narrowEdge=narrow.SampleDirection(Point2f(1,.5f));
    Check("basic beam narrow cone retains transverse direction despite rounded Z",
        narrowEdge.directionLocal.z==1 && std::hypot(double(narrowEdge.directionLocal.x),double(narrowEdge.directionLocal.y))>0);
    Check("basic beam narrow cone finite positive PDF",std::isfinite(narrowEdge.pdfPerSr) && narrowEdge.pdfPerSr>0 && narrow.PdfDirection(narrowEdge.directionLocal)>0);
    Check("basic beam narrow cone excludes genuinely outside direction",narrow.PdfDirection(DirectionAtSlope(std::tan(1.1e-6)))==0);

    const Vector3f axis=Vector3f(1,-2,3).normalize(), origin(0,10,0);
    const LaserEmitter oldDelta(origin,axis,532,1e-8f,0.0f);
    const LaserEmitter explicitDelta(origin,axis,532,1e-8f,std::make_shared<DeltaLidarBeamProfile>());
    const LaserEmitter oldCone(origin,axis,532,1e-8f,angle);
    const LaserEmitter explicitCone(origin,axis,532,1e-8f,std::make_shared<UniformConeLidarBeamProfile>(angle));
    const Point2f u(.8f,.2f);
    const auto d1=oldDelta.SampleRay(u),d2=explicitDelta.SampleRay(u);
    const auto c1=oldCone.SampleRay(u),c2=explicitCone.SampleRay(u);
    Check("basic beam legacy and explicit Delta use identical emission",
        d1.ray.dir.x==d2.ray.dir.x && d1.ray.dir.y==d2.ray.dir.y && d1.ray.dir.z==d2.ray.dir.z && d1.directionPdf==d2.directionPdf && d1.energyWeightJ==d2.energyWeightJ);
    Check("basic beam legacy and explicit cone use identical emission",
        c1.ray.dir.x==c2.ray.dir.x && c1.ray.dir.y==c2.ray.dir.y && c1.ray.dir.z==c2.ray.dir.z && c1.directionPdf==c2.directionPdf && c1.energyWeightJ==c2.energyWeightJ);
    Check("basic beam emitters always own nonnull profiles",oldDelta.BeamProfile() && explicitDelta.BeamProfile() && oldCone.BeamProfile() && explicitCone.BeamProfile());
    Check("basic beam legacy divergence accessor preserved",oldDelta.DivergenceHalfAngleRadians()==0 && oldCone.DivergenceHalfAngleRadians()==angle);
    const auto tinyEmission=LaserEmitter(origin,axis,532,1e-8f,1e-6f).SampleRay(u);
    Check("basic beam legacy narrow cone now yields valid continuous emission",!tinyEmission.deltaDirection && tinyEmission.directionPdf>0 && std::isfinite(tinyEmission.directionPdf));
}
