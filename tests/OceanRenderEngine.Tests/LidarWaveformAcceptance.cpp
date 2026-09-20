#include "LidarWaveform.h"
#include "GaussianPulseProfile.h"

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
    {
        std::cerr << "[FAIL] " << testName << '\n';
        throw std::runtime_error(testName + " failed");
    }

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
    double actual,
    double expected,
    double tolerance = 1e-15)
{
    return std::fabs(actual - expected) <= tolerance;
}

class FailingPulseProfile final
    : public LidarPulseProfile
{
public:
    double FractionBetween(
        double,
        double) const override
    {
        if (callCount++ == 0)
            return 0.25;

        return std::numeric_limits<double>::quiet_NaN();
    }

private:
    mutable std::size_t callCount = 0;
};
}

void RunLidarWaveformAcceptanceTests()
{
    LidarWaveform boundaryWaveform({
        10.0,
        0.25,
        4
    });

    ExpectTrue(
        "lidar waveform exposes bins and centers",
        boundaryWaveform.EnergyBinsJ().size() == 4 &&
        Near(boundaryWaveform.BinCenterTimeSeconds(0),
             10.125) &&
        Near(boundaryWaveform.BinCenterTimeSeconds(3),
             10.875));

    const bool acceptedStart =
        boundaryWaveform.Accumulate({
            1.0,
            10.0,
            532.0f
        });

    const bool acceptedBoundary =
        boundaryWaveform.Accumulate({
            2.0,
            10.25,
            532.0f
        });

    const bool rejectedBeforeStart =
        !boundaryWaveform.Accumulate({
            4.0,
            9.75,
            532.0f
        });

    const bool rejectedAtEnd =
        !boundaryWaveform.Accumulate({
            8.0,
            11.0,
            532.0f
        });

    const auto& boundaryBins =
        boundaryWaveform.EnergyBinsJ();

    ExpectTrue(
        "lidar waveform uses half-open bin boundaries",
        acceptedStart &&
        acceptedBoundary &&
        rejectedBeforeStart &&
        rejectedAtEnd &&
        Near(boundaryBins[0], 1.0) &&
        Near(boundaryBins[1], 2.0) &&
        Near(boundaryBins[2], 0.0) &&
        Near(boundaryBins[3], 0.0));

    LidarWaveform accumulationWaveform({
        0.0,
        1.0,
        2
    });

    accumulationWaveform.Accumulate({
        0.25,
        0.1,
        532.0f
    });

    accumulationWaveform.Accumulate({
        0.75,
        0.9,
        1064.0f
    });

    ExpectTrue(
        "lidar waveform sums returns in one bin",
        Near(accumulationWaveform.EnergyBinsJ()[0],
             1.0) &&
        Near(accumulationWaveform.TotalEnergyJ(),
             1.0));

    LidarPulseResult pulse;
    pulse.emittedRayCount = 8;
    pulse.surfaceHitCount = 3;
    pulse.returns = {
        {0.125, 20.1, 532.0f},
        {0.250, 20.6, 532.0f},
        {0.500, 20.9, 532.0f}
    };

    LidarWaveform pulseWaveform({
        20.0,
        0.5,
        2
    });

    pulseWaveform.Accumulate(pulse);

    ExpectTrue(
        "lidar waveform preserves pulse energy",
        Near(pulseWaveform.EnergyBinsJ()[0],
             0.125) &&
        Near(pulseWaveform.EnergyBinsJ()[1],
             0.750) &&
        Near(pulseWaveform.TotalEnergyJ(),
             pulse.TotalReceivedEnergyJ()));

    pulseWaveform.Clear();

    ExpectTrue(
        "lidar waveform clear resets all bins",
        Near(pulseWaveform.EnergyBinsJ()[0], 0.0) &&
        Near(pulseWaveform.EnergyBinsJ()[1], 0.0) &&
        Near(pulseWaveform.TotalEnergyJ(), 0.0));

    const bool acceptedZeroEnergy =
        pulseWaveform.Accumulate({
            0.0,
            20.25,
            532.0f
        });

    ExpectTrue(
        "lidar waveform accepts zero energy",
        acceptedZeroEnergy &&
        Near(pulseWaveform.TotalEnergyJ(), 0.0));

    ExpectThrows(
        "lidar waveform rejects invalid bin index",
        [&]
        {
            pulseWaveform.BinCenterTimeSeconds(2);
        });

    ExpectThrows(
        "lidar waveform rejects nonfinite start time",
        []
        {
            LidarWaveform invalid({
                std::numeric_limits<double>::quiet_NaN(),
                1.0,
                1
            });
        });

    ExpectThrows(
        "lidar waveform rejects zero bin width",
        []
        {
            LidarWaveform invalid({
                0.0,
                0.0,
                1
            });
        });

    ExpectThrows(
        "lidar waveform rejects nonfinite bin width",
        []
        {
            LidarWaveform invalid({
                0.0,
                std::numeric_limits<double>::infinity(),
                1
            });
        });

    ExpectThrows(
        "lidar waveform rejects zero bin count",
        []
        {
            LidarWaveform invalid({
                0.0,
                1.0,
                0
            });
        });

    ExpectThrows(
        "lidar waveform rejects overflowing duration",
        []
        {
            LidarWaveform invalid({
                0.0,
                std::numeric_limits<double>::max(),
                2
            });
        });

    ExpectThrows(
        "lidar waveform rejects nonfinite return time",
        [&]
        {
            pulseWaveform.Accumulate({
                1.0,
                std::numeric_limits<double>::infinity(),
                532.0f
            });
        });

    ExpectThrows(
        "lidar waveform rejects negative return energy",
        [&]
        {
            pulseWaveform.Accumulate({
                -1.0,
                20.25,
                532.0f
            });
        });

    ExpectThrows(
        "lidar waveform rejects nonfinite return energy",
        [&]
        {
            pulseWaveform.Accumulate({
                std::numeric_limits<double>::infinity(),
                20.25,
                532.0f
            });
        });

    LidarWaveform overflowWaveform({
        0.0,
        1.0,
        1
    });

    overflowWaveform.Accumulate({
        std::numeric_limits<double>::max(),
        0.5,
        532.0f
    });

    ExpectThrows(
        "lidar waveform detects energy overflow",
        [&]
        {
            overflowWaveform.Accumulate({
                std::numeric_limits<double>::max(),
                0.5,
                532.0f
            });
        });

    constexpr double gaussianFwhmSeconds =
        2.3548200450309493;

    GaussianPulseProfile gaussian(
        gaussianFwhmSeconds);

    const double sigma =
        gaussian.StandardDeviationSeconds();

    LidarWaveform gaussianWaveform({
        -3.0 * sigma,
        sigma,
        6
    });

    const double recordedGaussianEnergy =
        gaussianWaveform.Accumulate(
            {10.0, 0.0, 532.0f},
            gaussian);

    const double expectedGaussianEnergy =
        10.0 * gaussian.FractionBetween(
            -3.0 * sigma,
            3.0 * sigma);

    const auto& gaussianBins =
        gaussianWaveform.EnergyBinsJ();

    ExpectTrue(
        "lidar waveform distributes Gaussian return symmetrically",
        Near(gaussianBins[0], gaussianBins[5], 1e-12) &&
        Near(gaussianBins[1], gaussianBins[4], 1e-12) &&
        Near(gaussianBins[2], gaussianBins[3], 1e-12));

    ExpectTrue(
        "lidar waveform records Gaussian energy in window",
        Near(recordedGaussianEnergy,
             expectedGaussianEnergy,
             1e-12) &&
        Near(gaussianWaveform.TotalEnergyJ(),
             recordedGaussianEnergy,
             1e-12) &&
        recordedGaussianEnergy < 10.0);

    LidarWaveform clippedWaveform({
        0.0,
        sigma,
        1
    });

    const double clippedEnergy =
        clippedWaveform.Accumulate(
            {10.0, 0.0, 532.0f},
            gaussian);

    ExpectTrue(
        "lidar waveform preserves Gaussian window clipping",
        Near(clippedEnergy,
             10.0 * gaussian.FractionBetween(
                 0.0,
                 sigma),
             1e-12) &&
        clippedEnergy > 0.0 &&
        clippedEnergy < 10.0);

    LidarWaveform tailWaveform({
        0.0,
        sigma,
        1
    });

    const double tailEnergy =
        tailWaveform.Accumulate(
            {10.0, -sigma, 532.0f},
            gaussian);

    ExpectTrue(
        "lidar waveform accepts Gaussian tail outside window",
        Near(tailEnergy,
             10.0 * gaussian.FractionBetween(
                 sigma,
                 2.0 * sigma),
             1e-12) &&
        tailEnergy > 0.0);

    LidarWaveform firstReturnWaveform({
        -2.0 * sigma,
        sigma,
        4
    });

    LidarWaveform secondReturnWaveform({
        -2.0 * sigma,
        sigma,
        4
    });

    LidarWaveform overlappingWaveform({
        -2.0 * sigma,
        sigma,
        4
    });

    const LidarReturnSample firstReturn{
        2.0,
        -0.25 * sigma,
        532.0f
    };

    const LidarReturnSample secondReturn{
        3.0,
        0.25 * sigma,
        532.0f
    };

    firstReturnWaveform.Accumulate(
        firstReturn,
        gaussian);

    secondReturnWaveform.Accumulate(
        secondReturn,
        gaussian);

    overlappingWaveform.Accumulate(
        firstReturn,
        gaussian);

    overlappingWaveform.Accumulate(
        secondReturn,
        gaussian);

    bool overlapIsLinear = true;

    for (std::size_t i = 0; i < 4; ++i)
    {
        overlapIsLinear =
            overlapIsLinear &&
            Near(
                overlappingWaveform.EnergyBinsJ()[i],
                firstReturnWaveform.EnergyBinsJ()[i] +
                    secondReturnWaveform.EnergyBinsJ()[i],
                1e-12);
    }

    ExpectTrue(
        "lidar waveform linearly adds overlapping returns",
        overlapIsLinear);

    LidarWaveform atomicWaveform({
        0.0,
        1.0,
        3
    });

    atomicWaveform.Accumulate({
        1.0,
        0.5,
        532.0f
    });

    const std::vector<double> binsBeforeFailure =
        atomicWaveform.EnergyBinsJ();

    FailingPulseProfile failingProfile;

    ExpectThrows(
        "lidar waveform rejects invalid pulse profile",
        [&]
        {
            atomicWaveform.Accumulate(
                {2.0, 0.5, 532.0f},
                failingProfile);
        });

    ExpectTrue(
        "lidar waveform profile failure is atomic",
        atomicWaveform.EnergyBinsJ() ==
            binsBeforeFailure);
}
