#include "LidarScatteringModel.h"
#include "LambertianLidarScattering.h"
#include "RoughDielectricLidarScattering.h"
#include "OceanBeckmannLidarScattering.h"
#include "WhitecapCoverage.h"
#include "WhitecapLidarScattering.h"
#include "ElfouhailySpectrum.h"
#include "OceanFrequencyField.h"
#include "OceanSurfaceMesh.h"
#include "TriangleMeshAggregate.h"
#include "LidarIntegrator.h"
#include "LidarWaveform.h"
#include "HittableList.h"
#include "IndependentSampler.h"
#include "Sphere.h"
#include <memory>

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace
{
void ExpectTrue(
    const std::string& testName,
    bool condition)
{
    if (!condition)
    {
        std::cerr << "[FAIL] " << testName << '\n';
        throw std::runtime_error(testName + " failed");
    }

    std::cout << "[PASS] " << testName << '\n';
}

template <typename Function>
void ExpectInvalidArgument(
    const std::string& testName,
    Function&& function)
{
    bool threwInvalidArgument = false;

    try
    {
        function();
    }
    catch (const std::invalid_argument&)
    {
        threwInvalidArgument = true;
    }

    ExpectTrue(testName, threwInvalidArgument);
}

bool Near(
    double actual,
    double expected,
    double tolerance = 1.0e-15)
{
    return std::fabs(actual - expected) <= tolerance;
}

bool RelativeNear(
    double actual,
    double expected,
    double relativeTolerance = 1.0e-6,
    double absoluteTolerance = 1.0e-12)
{
    const double error =
        std::fabs(actual - expected);

    const double scale =
        std::fmax(
            std::fabs(actual),
            std::fabs(expected));

    return error <= std::fmax(
        absoluteTolerance,
        relativeTolerance * scale);
}
}

void RunWhitecapCoverageAcceptanceTests()
{
    ExpectTrue("whitecap zero wind zero fraction", MonahanWhitecapCoverage(0.0) == 0.0);
    ExpectTrue("whitecap unit wind coefficient", Near(MonahanWhitecapCoverage(1.0), 3.84e-6));
    ExpectTrue("whitecap 5 m/s reference", Near(MonahanWhitecapCoverage(5.0), 0.00092857917397726967));
    ExpectTrue("whitecap 10 m/s reference fraction", Near(MonahanWhitecapCoverage(10.0), 0.0098703198058324396));
    ExpectTrue("whitecap 15 m/s reference", Near(MonahanWhitecapCoverage(15.0), 0.039337107059105036));
    bool monotonic = true, bounded = true;
    double previous = -1.0;
    for (int i = 0; i <= 166; ++i)
    {
        const double value = MonahanWhitecapCoverage(i * 0.1);
        monotonic = monotonic && value > previous;
        bounded = bounded && std::isfinite(value) && value >= 0 && value <= 1;
        previous = value;
    }
    ExpectTrue("whitecap moderate wind sweep monotonic", monotonic);
    ExpectTrue("whitecap moderate wind sweep finite bounded", bounded);
    ExpectInvalidArgument("whitecap rejects negative wind", [] { MonahanWhitecapCoverage(-1); });
    ExpectInvalidArgument("whitecap rejects NaN wind", [] {
        MonahanWhitecapCoverage(std::numeric_limits<double>::quiet_NaN());
    });
    ExpectInvalidArgument("whitecap rejects infinite wind", [] {
        MonahanWhitecapCoverage(std::numeric_limits<double>::infinity());
    });
    auto domainRejected = [](double wind) {
        try { MonahanWhitecapCoverage(wind); }
        catch (const std::domain_error&) { return true; }
        return false;
    };
    ExpectTrue("whitecap excessive fraction rejected not clamped", domainRejected(100.0));
    ExpectTrue("whitecap power overflow rejected", domainRejected(std::numeric_limits<double>::max()));
}

void RunOceanBeckmannLidarAcceptanceTests();

void RunSpatialWhitecapAcceptanceTests()
{
    auto field = std::make_shared<OceanWhitecapField>(2, 4);
    bool uninitialized = false;
    try { field->CoverageAt(0,0); } catch (const std::logic_error&) { uninitialized = true; }
    ExpectTrue("spatial whitecap requires update", uninitialized);
    field->UpdateCellHeights({1,2,3,4}, .375);
    ExpectTrue("spatial whitecap known ranking", field->Coverages() == std::vector<double>({0,0,.5,1}));
    ExpectTrue("spatial whitecap area mean", Near(field->MeanCoverage(), .375));
    ExpectTrue("spatial whitecap mesh centered coordinates", field->CoverageAt(-1,1)==.5 && field->CoverageAt(1,1)==1);
    ExpectTrue("spatial whitecap periodic seams", field->CoverageAt(3,1)==.5 && field->CoverageAt(2,2)==0);
    const auto before = field->Coverages();
    field->UpdateCellHeights({1,2,3,4}, .375);
    ExpectTrue("spatial whitecap deterministic", field->Coverages()==before);
    ExpectInvalidArgument("spatial whitecap rejects invalid coverage", [&]{field->UpdateCellHeights({1,2,3,4}, -1);});
    ExpectInvalidArgument("spatial whitecap rejects wrong cell count", [&]{field->UpdateCellHeights({1}, .5);});
    ExpectInvalidArgument("spatial whitecap rejects NaN height", [&]{field->UpdateCellHeights({1,2,3,std::numeric_limits<double>::quiet_NaN()}, .5);});
    ExpectTrue("spatial whitecap failed update retains snapshot", field->Coverages()==before);
    ExpectInvalidArgument("spatial whitecap rejects invalid coordinate", [&]{field->CoverageAt(std::numeric_limits<double>::infinity(),0);});
    ExpectInvalidArgument("spatial whitecap rejects invalid grid", []{OceanWhitecapField bad(1,4);});
    ExpectInvalidArgument("spatial whitecap rejects invalid length", []{OceanWhitecapField bad(2,0);});
    field->UpdateCellHeights({4,4,4,4}, .5);
    ExpectTrue("spatial whitecap flat field explicitly differs from target", !field->HasHeightContrast() && field->MeanCoverage()==0 && field->TargetCoverage()==.5);
    field->UpdateCellHeights({1,2,3,4},0);
    ExpectTrue("spatial whitecap zero endpoint",field->MeanCoverage()==0);
    field->UpdateCellHeights({1,2,3,4},1);
    ExpectTrue("spatial whitecap full endpoint",field->MeanCoverage()==1);
    field->UpdateCellHeights({1,2,3,4},.375);
    auto water = std::make_shared<ConstantLidarScattering>(.2);
    WhitecapLidarScattering mixture(water,field,.6);
    HitRecord hit{}; hit.geometricNormal=hit.normal=Vector3f(0,1,0);
    const Vector3f up(0,1,0);
    hit.point=Vector3f(-1,0,-1);
    ExpectTrue("whitecap scattering uncovered water recovery", Near(mixture.Evaluate(hit,up,up,532),.2));
    hit.point=Vector3f(1,0,1);
    const double foam=.6/std::numbers::pi_v<double>;
    ExpectTrue("whitecap scattering covered Lambert recovery", Near(mixture.Evaluate(hit,up,up,532),foam));
    hit.point=Vector3f(-1,0,1);
    ExpectTrue("whitecap scattering fractional area mixture", Near(mixture.Evaluate(hit,up,up,532),.5*(.2+foam)));
    ExpectTrue("whitecap scattering geometric backside", mixture.Evaluate(hit,-up,up,532)==0);
    const Vector3f wi(.2f,1,.1f),wo(-.3f,1,.4f);
    ExpectTrue("whitecap scattering reciprocal baseline", Near(mixture.Evaluate(hit,wi,wo,532),mixture.Evaluate(hit,wo,wi,532)));
    ExpectInvalidArgument("whitecap scattering rejects missing water", [&]{WhitecapLidarScattering bad(nullptr,field,.6);});
    ExpectInvalidArgument("whitecap scattering rejects reflectance", [&]{WhitecapLidarScattering bad(water,field,1.1);});
    ExpectInvalidArgument("whitecap scattering rejects wavelength", [&]{mixture.Evaluate(hit,up,up,0);});
    ExpectInvalidArgument("whitecap scattering rejects direction", [&]{mixture.Evaluate(hit,Vector3f(0,0,0),up,532);});

    ElfouhailyConfig config; config.resolution=32; config.patchLength=32; config.seed=42;
    ElfouhailySpectrum spectrum(config);
    OceanFrequencyField frequency({32,32,42},[&](float x,float z){return spectrum.CartesianSpectrum(x,z);},
        [&](float k){return k*spectrum.PhaseSpeed(k);});
    OceanHeightField height(frequency,.5f); height.Update(1.25f);
    auto seaField=std::make_shared<OceanWhitecapField>(32,32);
    seaField->Update(height,config.windSpeed10m);
    ExpectTrue("spatial whitecap real Elfouhaily target mean", Near(seaField->MeanCoverage(),MonahanWhitecapCoverage(10),1e-12));
    const auto frozen=seaField->Coverages(); seaField->Update(height,10);
    ExpectTrue("spatial whitecap frozen repeat",seaField->Coverages()==frozen);
    ExpectInvalidArgument("spatial whitecap rejects mismatched height grid",[&]{field->Update(height,10);});
    // 用实际三角网格，选择完全覆盖的单元中心，验证接收能量之比。
    const auto chosen=std::find(frozen.begin(),frozen.end(),1.0)-frozen.begin();
    ExpectTrue("spatial whitecap actual sea has covered cell",chosen<frozen.size());
    const float x=-16+float(chosen%32)+.5f,z=-16+float(chosen/32)+.5f;
    OceanSurfaceMesh surface(32,32); surface.Update(height);
    TriangleMeshAggregate world(surface.Mesh(),nullptr);
    const Vector3f sensor(x,20,z);
    LaserEmitter emitter(sensor,-up,532,1e-3f,0);
    LidarReceiver receiver(sensor,-up,.25f,.01f,.8f);
    WhitecapLidarScattering seaMixture(water,seaField,.6);
    LidarIntegrator mixedIntegrator(world,emitter,receiver,1,seaMixture);
    LidarIntegrator waterIntegrator(world,emitter,receiver,1,*water);
    IndependentSampler mixedSampler(42),waterSampler(42);
    const auto mixedPulse=mixedIntegrator.SimulatePulse(mixedSampler);
    const auto waterPulse=waterIntegrator.SimulatePulse(waterSampler);
    ExpectTrue("whitecap lidar real ocean hit and return",mixedPulse.surfaceHitCount==1 && mixedPulse.returns.size()==1 && waterPulse.returns.size()==1);
    ExpectTrue("whitecap lidar mixture energy ratio",RelativeNear(mixedPulse.TotalReceivedEnergyJ()/waterPulse.TotalReceivedEnergyJ(),foam/.2));
    LidarWaveform waveform({0,1e-9,256});
    ExpectTrue("whitecap lidar waveform conservation",RelativeNear(waveform.AccumulatePulse(mixedPulse),mixedPulse.TotalReceivedEnergyJ()));
}

void RunLidarScatteringAcceptanceTests()
{
    RunWhitecapCoverageAcceptanceTests();
    RunSpatialWhitecapAcceptanceTests();
    RunOceanBeckmannLidarAcceptanceTests();
    static_assert(
        std::is_abstract_v<LidarScatteringModel>);

    static_assert(
        std::is_final_v<ConstantLidarScattering>);

    const ConstantLidarScattering constantModel(0.25);
    const LidarScatteringModel& scatteringModel =
        constantModel;

    const HitRecord hit{};
    const Vector3f incidentDirection(
        0.0f,
        0.0f,
        1.0f);
    const Vector3f outgoingDirection(
        0.0f,
        1.0f,
        1.0f);

    ExpectTrue(
        "constant lidar scattering exposes BRDF",
        constantModel.BrdfAtWavelengthPerSr() == 0.25);

    ExpectTrue(
        "constant lidar scattering dispatches through interface",
        scatteringModel.Evaluate(
            hit,
            incidentDirection,
            outgoingDirection,
            532.0f) == 0.25);

    const ConstantLidarScattering zeroModel(0.0);

    ExpectTrue(
        "constant lidar scattering accepts zero BRDF",
        zeroModel.Evaluate(
            hit,
            incidentDirection,
            outgoingDirection,
            1064.0f) == 0.0);

    ExpectInvalidArgument(
        "constant lidar scattering rejects negative BRDF",
        []
        {
            ConstantLidarScattering invalid(-1.0);
        });

    ExpectInvalidArgument(
        "constant lidar scattering rejects NaN BRDF",
        []
        {
            ConstantLidarScattering invalid(
                std::numeric_limits<double>::quiet_NaN());
        });

    ExpectInvalidArgument(
        "constant lidar scattering rejects positive infinity",
        []
        {
            ConstantLidarScattering invalid(
                std::numeric_limits<double>::infinity());
        });

    ExpectInvalidArgument(
        "constant lidar scattering rejects negative infinity",
        []
        {
            ConstantLidarScattering invalid(
                -std::numeric_limits<double>::infinity());
        });

    static_assert(
        std::is_final_v<LambertianLidarScattering>);

    const LambertianLidarScattering lambertianModel(0.5);
    const LidarScatteringModel& lambertianInterface =
        lambertianModel;

    ExpectTrue(
        "Lambertian lidar scattering exposes reflectance",
        Near(
            lambertianModel.ReflectanceAtWavelength(),
            0.5));

    ExpectTrue(
        "Lambertian lidar scattering returns rho over pi",
        Near(
            lambertianInterface.Evaluate(
                hit,
                incidentDirection,
                outgoingDirection,
                532.0f),
            0.5 / std::numbers::pi_v<double>));

    const LambertianLidarScattering zeroLambertian(0.0);
    const LambertianLidarScattering unitLambertian(1.0);

    ExpectTrue(
        "Lambertian lidar scattering accepts reflectance bounds",
        Near(
            zeroLambertian.Evaluate(
                hit,
                incidentDirection,
                outgoingDirection,
                532.0f),
            0.0) &&
        Near(
            unitLambertian.Evaluate(
                hit,
                incidentDirection,
                outgoingDirection,
                532.0f),
            1.0 / std::numbers::pi_v<double>));

    ExpectInvalidArgument(
        "Lambertian lidar scattering rejects negative reflectance",
        []
        {
            LambertianLidarScattering invalid(-0.01);
        });

    ExpectInvalidArgument(
        "Lambertian lidar scattering rejects reflectance above one",
        []
        {
            LambertianLidarScattering invalid(1.01);
        });

    ExpectInvalidArgument(
        "Lambertian lidar scattering rejects NaN reflectance",
        []
        {
            LambertianLidarScattering invalid(
                std::numeric_limits<double>::quiet_NaN());
        });

    ExpectInvalidArgument(
        "Lambertian lidar scattering rejects infinite reflectance",
        []
        {
            LambertianLidarScattering invalid(
                std::numeric_limits<double>::infinity());
        });

    static_assert(
        std::is_final_v<RoughDielectricLidarScattering>);

    HitRecord roughHit{};
    roughHit.geometricNormal =
        Vector3f(0.0f, 0.0f, 1.0f);
    roughHit.normal =
        roughHit.geometricNormal;

    constexpr double etaIncident = 1.0;
    constexpr double etaTransmitted = 1.5;
    constexpr float alpha = 0.2f;

    const RoughDielectricLidarScattering roughModel(
        etaIncident,
        etaTransmitted,
        alpha,
        alpha);

    const LidarScatteringModel& roughInterface =
        roughModel;

    const Vector3f normalDirection(
        0.0f,
        0.0f,
        1.0f);

    const double normalAmplitude =
        (etaIncident - etaTransmitted) /
        (etaIncident + etaTransmitted);

    const double normalFresnel =
        normalAmplitude * normalAmplitude;

    const double expectedNormalBrdf =
        normalFresnel /
        (4.0 *
         std::numbers::pi_v<double> *
         static_cast<double>(alpha) *
         static_cast<double>(alpha));

    const double normalBrdf =
        roughInterface.Evaluate(
            roughHit,
            normalDirection,
            normalDirection,
            532.0f);

    ExpectTrue(
        "rough dielectric lidar scattering normal incidence",
        RelativeNear(
            normalBrdf,
            expectedNormalBrdf));

    const Vector3f angledIncident =
        Vector3f(0.3f, 0.0f, 1.0f).normalize();

    const Vector3f angledOutgoing =
        Vector3f(-0.1f, 0.2f, 1.0f).normalize();

    const double forwardBrdf =
        roughModel.Evaluate(
            roughHit,
            angledIncident,
            angledOutgoing,
            532.0f);

    const double reverseBrdf =
        roughModel.Evaluate(
            roughHit,
            angledOutgoing,
            angledIncident,
            532.0f);

    ExpectTrue(
        "rough dielectric lidar scattering is reciprocal",
        forwardBrdf > 0.0 &&
        RelativeNear(
            forwardBrdf,
            reverseBrdf));

    ExpectTrue(
        "rough dielectric lidar scattering rejects lower hemisphere",
        Near(
            roughModel.Evaluate(
                roughHit,
                normalDirection,
                Vector3f(0.0f, 0.0f, -1.0f),
                532.0f),
            0.0));

    HitRecord fallbackNormalHit{};
    fallbackNormalHit.geometricNormal =
        normalDirection;

    HitRecord flippedNormalHit = roughHit;
    flippedNormalHit.normal =
        -normalDirection;

    ExpectTrue(
        "rough dielectric lidar scattering handles shading normals",
        RelativeNear(
            roughModel.Evaluate(
                fallbackNormalHit,
                normalDirection,
                normalDirection,
                532.0f),
            normalBrdf) &&
        RelativeNear(
            roughModel.Evaluate(
                flippedNormalHit,
                normalDirection,
                normalDirection,
                532.0f),
            normalBrdf));

    const RoughDielectricLidarScattering smootherModel(
        etaIncident,
        etaTransmitted,
        0.1f,
        0.1f);

    const RoughDielectricLidarScattering rougherModel(
        etaIncident,
        etaTransmitted,
        0.3f,
        0.3f);

    ExpectTrue(
        "rough dielectric lidar scattering roughness broadens peak",
        smootherModel.Evaluate(
            roughHit,
            normalDirection,
            normalDirection,
            532.0f) >
        rougherModel.Evaluate(
            roughHit,
            normalDirection,
            normalDirection,
            532.0f));

    ExpectInvalidArgument(
        "rough dielectric lidar scattering rejects invalid index",
        []
        {
            RoughDielectricLidarScattering invalid(
                0.0,
                1.5,
                0.2f,
                0.2f);
        });

    ExpectInvalidArgument(
        "rough dielectric lidar scattering rejects delta roughness",
        []
        {
            RoughDielectricLidarScattering invalid(
                1.0,
                1.5,
                1.0e-4f,
                0.2f);
        });

    ExpectInvalidArgument(
        "rough dielectric lidar scattering rejects nonfinite roughness",
        []
        {
            RoughDielectricLidarScattering invalid(
                1.0,
                1.5,
                std::numeric_limits<float>::quiet_NaN(),
                0.2f);
        });

    ExpectInvalidArgument(
        "rough dielectric lidar scattering rejects invalid direction",
        [&]
        {
            roughModel.Evaluate(
                roughHit,
                Vector3f(),
                normalDirection,
                532.0f);
        });

    ExpectInvalidArgument(
        "rough dielectric lidar scattering rejects invalid normal",
        [&]
        {
            const HitRecord invalidHit{};

            roughModel.Evaluate(
                invalidHit,
                normalDirection,
                normalDirection,
                532.0f);
        });
}

void RunOceanBeckmannLidarAcceptanceTests()
{
    const OceanSlopeVariance slopes{0.02, 0.045, 0.005};
    const OceanBeckmannLidarScattering model(slopes, 1.0, 1.5);
    const Vector3f up(0, 1, 0);
    HitRecord flat{};
    flat.normal = up;
    flat.geometricNormal = up;
    const double determinant = slopes.varianceX * slopes.varianceZ -
        slopes.covarianceXZ * slopes.covarianceXZ;
    const double expected = 0.04 /
        (8.0 * std::numbers::pi_v<double> * std::sqrt(determinant));
    auto eval = [&](const HitRecord& hit, const Vector3f& wi, const Vector3f& wo) {
        return model.Evaluate(hit, wi, wo, 532.0f);
    };
    ExpectTrue("ocean Beckmann correlated flat normal analytic BRDF",
        RelativeNear(eval(flat, up, up), expected));
    const OceanBeckmannLidarScattering isotropic({0.02, 0.02, 0}, 1, 1.5);
    ExpectTrue("ocean Beckmann isotropic normal analytic BRDF",
        RelativeNear(isotropic.Evaluate(flat, up, up, 532),
            0.04 / (8 * std::numbers::pi_v<double> * 0.02)));

    const Vector3f wi(0.3f, 1, 0.2f), wo(-0.2f, 1, 0.4f);
    const double oblique = eval(flat, wi, wo);
    ExpectTrue("ocean Beckmann oblique finite positive",
        std::isfinite(oblique) && oblique > 0);
    ExpectTrue("ocean Beckmann reciprocal",
        RelativeNear(oblique, eval(flat, wo, wi)));
    ExpectTrue("ocean Beckmann direction scale invariant",
        RelativeNear(oblique, eval(flat, wi * 3.0f, wo * 7.0f)));

    HitRecord tilted = flat;
    tilted.normal = Vector3f(-0.4f, 1, -0.2f).normalize();
    // Independently: det(J)=+-n.y^3, so sqrt(det(local covariance)) =
    // n.y^3 * sqrt(det(world covariance)).
    const double tiltedExpected = expected / std::pow(double(tilted.normal.y), 3);
    ExpectTrue("ocean Beckmann tilted covariance determinant analytic BRDF",
        RelativeNear(eval(tilted, tilted.normal, tilted.normal), tiltedExpected));

    HitRecord reversed = tilted;
    reversed.normal = -tilted.normal;
    ExpectTrue("ocean Beckmann corrects shading normal hemisphere",
        RelativeNear(eval(reversed, wi, wo), eval(tilted, wi, wo)));
    HitRecord missing = flat;
    missing.normal = Vector3f(0, 0, 0);
    ExpectTrue("ocean Beckmann absent shading normal fallback",
        RelativeNear(eval(missing, up, up), expected));
    missing.normal = Vector3f(std::numeric_limits<float>::quiet_NaN(), 0, 0);
    ExpectTrue("ocean Beckmann invalid shading normal fallback",
        RelativeNear(eval(missing, up, up), expected));
    ExpectTrue("ocean Beckmann rejects geometric incident backside",
        eval(flat, -up, up) == 0);
    ExpectTrue("ocean Beckmann rejects geometric outgoing backside",
        eval(flat, up, -up) == 0);
    ExpectTrue("ocean Beckmann geometric horizon returns zero",
        eval(flat, Vector3f(1, 0, 0), up) == 0);
    ExpectTrue("ocean Beckmann shading backside returns zero",
        eval(tilted, Vector3f(1, 0.1f, 0), up) == 0);
    const OceanBeckmannLidarScattering matched(slopes, 1, 1);
    ExpectTrue("ocean Beckmann matched indices zero reflection",
        matched.Evaluate(flat, up, up, 532) == 0);
    ExpectTrue("ocean Beckmann fixed indices wavelength independent baseline",
        RelativeNear(model.Evaluate(flat, wi, wo, 905), oblique));
    OceanSlopeVariance mutableSlopes = slopes;
    const OceanBeckmannLidarScattering copied(mutableSlopes, 1, 1.5);
    mutableSlopes.varianceX = 2;
    ExpectTrue("ocean Beckmann owns slope statistics",
        RelativeNear(copied.Evaluate(flat, up, up, 532), expected));

    ExpectInvalidArgument("ocean Beckmann rejects zero variance", [&] {
        OceanBeckmannLidarScattering bad({0, 0.02, 0}, 1, 1.5);
    });
    ExpectInvalidArgument("ocean Beckmann rejects singular covariance", [&] {
        OceanBeckmannLidarScattering bad({1, 1, 1}, 1, 1.5);
    });
    ExpectInvalidArgument("ocean Beckmann rejects invalid index", [&] {
        OceanBeckmannLidarScattering bad(slopes, 0, 1.5);
    });
    ExpectInvalidArgument("ocean Beckmann rejects zero direction", [&] {
        eval(flat, Vector3f(0, 0, 0), up);
    });
    ExpectInvalidArgument("ocean Beckmann rejects infinite direction", [&] {
        eval(flat, Vector3f(std::numeric_limits<float>::infinity(), 1, 0), up);
    });
    ExpectInvalidArgument("ocean Beckmann rejects invalid geometric normal", [&] {
        HitRecord bad = flat;
        bad.geometricNormal = Vector3f(0, 0, 0);
        eval(bad, up, up);
    });
    ExpectInvalidArgument("ocean Beckmann rejects zero wavelength", [&] {
        model.Evaluate(flat, up, up, 0);
    });
    ExpectInvalidArgument("ocean Beckmann rejects NaN wavelength", [&] {
        model.Evaluate(flat, up, up, std::numeric_limits<float>::quiet_NaN());
    });
    ExpectInvalidArgument("ocean Beckmann rejects non height-field shading normal", [&] {
        HitRecord bad = flat;
        bad.normal = Vector3f(1, 0, 0);
        eval(bad, up, up);
    });

    // A sphere's top hit has exactly the flat +Y frame above.
    // This checks adapter -> return estimator -> 1/N -> waveform independently
    // of ocean mesh generation, not a frozen Elfouhaily integration test.
    HittableList world(std::make_shared<Sphere>(Vector3f(0, -1, 0), 1, nullptr));
    const Vector3f sensor(0, 15, 0);
    LaserEmitter emitter(sensor, -up, 532, 1e-3f, 0);
    LidarReceiver receiver(sensor, -up, 0.25f, 0.01f, 0.8f);
    LidarIntegrator single(world, emitter, receiver, 1, model);
    LidarIntegrator many(world, emitter, receiver, 16, model);
    IndependentSampler oneSampler(42), manySampler(42), repeatSampler(42);
    const auto pulse = single.SimulatePulse(oneSampler);
    const auto multi = many.SimulatePulse(manySampler);
    const double energyExpected = double(1e-3f) * expected *
        double(0.01f) / (15.0 * 15.0) * double(0.8f);
    ExpectTrue("ocean Beckmann lidar hit and return counts",
        pulse.surfaceHitCount == 1 && pulse.returns.size() == 1 &&
        multi.surfaceHitCount == 16 && multi.returns.size() == 16);
    ExpectTrue("ocean Beckmann lidar analytic return energy",
        RelativeNear(pulse.TotalReceivedEnergyJ(), energyExpected));
    ExpectTrue("ocean Beckmann lidar sample count compensation",
        RelativeNear(multi.TotalReceivedEnergyJ(), pulse.TotalReceivedEnergyJ()));
    ExpectTrue("ocean Beckmann lidar two-way arrival time",
        std::fabs(pulse.returns.at(0).arrivalTimeSeconds - 30.0 / 299792458.0) < 1e-12);
    LidarWaveform waveform({0, 1e-9, 256});
    ExpectTrue("ocean Beckmann waveform energy conserved",
        RelativeNear(waveform.AccumulatePulse(multi), multi.TotalReceivedEnergyJ()));
    const auto repeat = many.SimulatePulse(repeatSampler);
    ExpectTrue("ocean Beckmann fixed seed pulse repeat",
        repeat.TotalReceivedEnergyJ() == multi.TotalReceivedEnergyJ() &&
        repeat.returns.at(0).arrivalTimeSeconds == multi.returns.at(0).arrivalTimeSeconds);
}
