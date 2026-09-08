#include "LaserEmitter.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
void ExpectTrue(
    const std::string& testName,
    bool condition)
{
    if (!condition)
        throw std::runtime_error(testName + " failed");

    std::cout << "[PASS] " << testName << '\n';
}

template <typename Function>
void ExpectThrows(
    const std::string& testName,
    Function&& function)
{
    bool threw = false;

    try
    {
        function();
    }
    catch (const std::exception&)
    {
        threw = true;
    }

    ExpectTrue(testName, threw);
}

bool Near(
    float a,
    float b,
    float epsilon = 1e-5f)
{
    return std::fabs(a - b) <= epsilon;
}

bool VectorNear(
    const Vector3f& a,
    const Vector3f& b,
    float epsilon = 1e-5f)
{
    return
        Near(a.x, b.x, epsilon) &&
        Near(a.y, b.y, epsilon) &&
        Near(a.z, b.z, epsilon);
}
}

void RunLaserEmitterAcceptanceTests()
{
    const Vector3f position(
        1.0f,
        15.0f,
        -2.0f);

    LaserEmitter pencilLaser(
        position,
        Vector3f(0.0f, -2.0f, 0.0f),
        532.0f,
        1.5e-3f,
        0.0f);

    ExpectTrue(
        "laser emitter normalized configuration",
        VectorNear(
            pencilLaser.Position(),
            position) &&
        VectorNear(
            pencilLaser.Direction(),
            Vector3f(0.0f, -1.0f, 0.0f)) &&
        Near(pencilLaser.WavelengthNm(), 532.0f) &&
        Near(pencilLaser.PulseEnergyJ(), 1.5e-3f) &&
        Near(
            pencilLaser.DivergenceHalfAngleRadians(),
            0.0f) &&
        pencilLaser.IsDeltaDirection());

    const LaserEmissionSample pencilSample =
        pencilLaser.SampleRay(
            Point2f(0.25f, 0.75f),
            2.5e-6f);

    ExpectTrue(
        "laser emitter delta sample",
        VectorNear(pencilSample.ray.orig, position) &&
        VectorNear(
            pencilSample.ray.dir,
            pencilLaser.Direction()) &&
        Near(pencilSample.wavelengthNm, 532.0f) &&
        Near(
            pencilSample.emissionTimeSeconds,
            2.5e-6f) &&
        Near(pencilSample.energyWeightJ, 1.5e-3f) &&
        Near(pencilSample.directionPdf, 0.0f) &&
        pencilSample.deltaDirection);

    const Vector3f tiltedDirection =
        Vector3f(1.0f, -2.0f, 3.0f).normalize();

    const float halfAngle = 0.1f;

    LaserEmitter coneLaser(
        position,
        tiltedDirection,
        1064.0f,
        2.0e-3f,
        halfAngle);

    const LaserEmissionSample coneCenterSample =
        coneLaser.SampleRay(
            Point2f(0.0f, 0.37f),
            0.0f);

    ExpectTrue(
        "laser cone center follows main direction",
        VectorNear(
            coneCenterSample.ray.dir,
            tiltedDirection) &&
        !coneCenterSample.deltaDirection);

    const float cosThetaMax =
        std::cos(halfAngle);

    const float expectedPdf =
        1.0f /
        (2.0f * Pi *
         (1.0f - cosThetaMax));

    const LaserEmissionSample coneBoundarySample =
        coneLaser.SampleRay(
            Point2f(1.0f, 0.25f),
            4.0e-6f);

    ExpectTrue(
        "laser cone boundary and PDF",
        Near(
            coneBoundarySample.ray.dir.dot(
                tiltedDirection),
            cosThetaMax,
            2e-5f) &&
        Near(
            coneBoundarySample.ray.dir.norm(),
            1.0f) &&
        Near(
            coneBoundarySample.directionPdf,
            expectedPdf,
            2e-3f) &&
        Near(
            coneBoundarySample.energyWeightJ,
            2.0e-3f) &&
        Near(
            coneBoundarySample.emissionTimeSeconds,
            4.0e-6f));

    bool allSamplesInsideCone = true;
    bool allSamplesHaveConstantPdf = true;

    for (int uIndex = 0; uIndex <= 8; ++uIndex)
    {
        for (int vIndex = 0; vIndex <= 8; ++vIndex)
        {
            const Point2f u(
                static_cast<float>(uIndex) / 8.0f,
                static_cast<float>(vIndex) / 8.0f);

            const LaserEmissionSample sample =
                coneLaser.SampleRay(u);

            const float cosTheta =
                sample.ray.dir.dot(
                    tiltedDirection);

            allSamplesInsideCone =
                allSamplesInsideCone &&
                Near(sample.ray.dir.norm(), 1.0f) &&
                cosTheta >= cosThetaMax - 2e-5f &&
                cosTheta <= 1.0f + 2e-5f;

            allSamplesHaveConstantPdf =
                allSamplesHaveConstantPdf &&
                Near(
                    sample.directionPdf,
                    expectedPdf,
                    2e-3f);
        }
    }

    ExpectTrue(
        "laser cone samples stay inside rotated cone",
        allSamplesInsideCone);

    ExpectTrue(
        "laser cone uniform PDF",
        allSamplesHaveConstantPdf &&
        Near(
            expectedPdf *
                2.0f * Pi *
                (1.0f - cosThetaMax),
            1.0f));

    const float nan =
        std::numeric_limits<float>::quiet_NaN();

    ExpectThrows(
        "laser emitter rejects invalid position",
        [nan]
        {
            LaserEmitter emitter(
                Vector3f(nan, 0.0f, 0.0f),
                Vector3f(0.0f, -1.0f, 0.0f),
                532.0f,
                1.0f,
                0.0f);
        });

    ExpectThrows(
        "laser emitter rejects zero direction",
        []
        {
            LaserEmitter emitter(
                Vector3f(0.0f, 0.0f, 0.0f),
                Vector3f(0.0f, 0.0f, 0.0f),
                532.0f,
                1.0f,
                0.0f);
        });

    ExpectThrows(
        "laser emitter rejects invalid wavelength",
        []
        {
            LaserEmitter emitter(
                Vector3f(0.0f, 0.0f, 0.0f),
                Vector3f(0.0f, -1.0f, 0.0f),
                0.0f,
                1.0f,
                0.0f);
        });

    ExpectThrows(
        "laser emitter rejects invalid pulse energy",
        []
        {
            LaserEmitter emitter(
                Vector3f(0.0f, 0.0f, 0.0f),
                Vector3f(0.0f, -1.0f, 0.0f),
                532.0f,
                0.0f,
                0.0f);
        });

    ExpectThrows(
        "laser emitter rejects invalid divergence",
        []
        {
            LaserEmitter emitter(
                Vector3f(0.0f, 0.0f, 0.0f),
                Vector3f(0.0f, -1.0f, 0.0f),
                532.0f,
                1.0f,
                PiOver2);
        });

    ExpectThrows(
        "laser emitter rejects invalid sample",
        [&coneLaser]
        {
            coneLaser.SampleRay(
                Point2f(1.1f, 0.5f));
        });

    ExpectThrows(
        "laser emitter rejects invalid emission time",
        [&coneLaser]
        {
            coneLaser.SampleRay(
                Point2f(0.5f, 0.5f),
                -1.0f);
        });
}
