#pragma once

#include "lidar/receiver/analog/FirstOrderLidarResponse.h"
#include "lidar/receiver/analog/IdealTransimpedanceAmplifier.h"
#include "lidar/receiver/adc/IdealUniformAdc.h"
#include "lidar/receiver/analog/LidarAnalogWaveform.h"
#include "lidar/signal_processing/LidarDigitalBaselineEstimator.h"
#include "lidar/signal_processing/LidarDigitalRangeEstimator.h"
#include "lidar/signal_processing/LidarDigitalMultiEchoEstimator.h"
#include "lidar/receiver/adc/LidarDigitalWaveform.h"
#include "lidar/signal_processing/LidarRangeCalibration.h"
#include "lidar/receiver/analog/LidarVoltageNoise.h"
#include "lidar/receiver/analog/LidarVoltageWaveform.h"
#include "lidar/simulation/LidarWaveform.h"
#include "lidar/receiver/analog/LinearPhotodetector.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

enum class LidarReceiverRangeMethod
{
    Peak,
    Centroid,
    PeakNeighborhoodCentroid
};

struct LidarReceiverPipelineConfig
{
    // 单色输入；LidarWaveform 不保存波长，调用方保证匹配。
    float wavelengthNm = 532.0f;
    double quantumEfficiency = 0.6;
    double responseTimeConstantSeconds = 2.0e-9;
    double transimpedanceGainVoltsPerAmp = 10000.0;
    double noiseSigmaVoltageV = 1.0e-3;

    int adcBitCount = 12;
    double adcMinimumVoltageV = -1.0;
    double adcMaximumVoltageV = 1.0;

    // 必须由调用方显式设置为有效、互不重叠的窗口。
    // 使用与输入波形相同的时间坐标，按箱中心选取 [start,end)。
    // 调用方还需保证 noiseWindow 不含回波或上一脉冲拖尾。
    LidarReturnWindow noiseWindow;
    LidarReturnWindow searchWindow;

    // 实际门限 = max(最低门限, 倍数 * 估计的数字码标准差)。
    // 倍数是实验参数，不保证固定误报率。
    double thresholdMultiplier = 6.0;
    double minimumThresholdCodes = 0.0;
    LidarReceiverRangeMethod rangeMethod =
        LidarReceiverRangeMethod::PeakNeighborhoodCentroid;
    // 仅是可调整的默认候选：当前 0.25 ns 箱宽下为 18 ns。
    LidarPeakNeighborhood neighborhood{24, 47};

    // 已完成的固定标定；Process 不重新拟合。
    std::optional<LidarRangeCalibration> calibration;

    // 仅供显式 ProcessMultiEcho 使用，不改变 Process 的旧单回波算法。
    // 与单回波 neighborhood 独立；改变窗口/算法后须验证 calibration 是否适用。
    LidarDigitalMultiEchoConfig multiEcho;
};

struct LidarReceiverPipelineResult
{
    LidarDigitalBaselineEstimate baseline;
    double thresholdCodes = 0.0;
    // 整张数字记录的量程诊断，不等于只看测距搜索区。
    bool hasAdcOutOfRangeSamples = false;
    std::optional<LidarDigitalRangeEstimate> measured;
    std::optional<LidarCalibratedRangeEstimate> corrected;
};

struct LidarReceiverEchoResult
{
    // 保留检峰平台、局部窗口、原始测距和各层级削顶诊断，不自动丢弃削顶回波。
    LidarDigitalEchoEstimate estimate;
    // 无标定，或校正后传播时间为负时为空；原始测量仍保留。
    std::optional<LidarCalibratedRangeEstimate> corrected;
};

struct LidarReceiverMultiEchoResult
{
    LidarDigitalBaselineEstimate baseline;
    double thresholdCodes = 0.0;
    bool hasAdcOutOfRangeSamples = false; // 整张数字记录，不只是某个回波窗口。
    std::vector<LidarReceiverEchoResult> echoes; // 时间顺序，空集合表示未检出。
};

// 只组织接收与数字处理，不生成海面、不追踪几何、不再次应用脉冲轮廓。
// 每次处理都是独立记录：接收响应零初始状态，噪声由显式 seed 决定。
// 仍采用箱平均电压数字化，不模拟独立 ADC 采样时钟。
class LidarReceiverPipeline
{
public:
    explicit LidarReceiverPipeline(const LidarReceiverPipelineConfig& config)
        : config(config),
          detector(config.wavelengthNm, config.quantumEfficiency),
          response(config.responseTimeConstantSeconds),
          amplifier(config.transimpedanceGainVoltsPerAmp),
          noise(config.noiseSigmaVoltageV),
          adc(config.adcBitCount, config.adcMinimumVoltageV,
              config.adcMaximumVoltageV),
          multiEchoEstimator(config.multiEcho)
    {
        ValidateWindow(config.noiseWindow);
        ValidateWindow(config.searchWindow);
        if (config.noiseWindow.startTimeSeconds < config.searchWindow.endTimeSeconds &&
            config.searchWindow.startTimeSeconds < config.noiseWindow.endTimeSeconds)
        {
            throw std::invalid_argument("Noise and search windows must not overlap.");
        }
        if (config.transimpedanceGainVoltsPerAmp <= 0.0)
            throw std::invalid_argument("Receiver pipeline requires positive pulse polarity.");
        if (!std::isfinite(config.thresholdMultiplier) || config.thresholdMultiplier < 0.0 ||
            !std::isfinite(config.minimumThresholdCodes) || config.minimumThresholdCodes < 0.0)
        {
            throw std::invalid_argument("Threshold parameters must be finite and nonnegative.");
        }
        switch (config.rangeMethod)
        {
        case LidarReceiverRangeMethod::Peak:
        case LidarReceiverRangeMethod::Centroid:
        case LidarReceiverRangeMethod::PeakNeighborhoodCentroid:
            break;
        default:
            throw std::invalid_argument("Unknown receiver range method.");
        }
    }

    LidarReceiverPipelineResult Process(
        const LidarWaveform& opticalWaveform,
        double emissionTimeSeconds,
        std::uint64_t noiseSeed) const
    {
        const auto record = PrepareRecord(opticalWaveform, emissionTimeSeconds, noiseSeed);
        const auto& digital = record.digital;
        LidarReceiverPipelineResult result;
        result.baseline = record.baseline;
        result.hasAdcOutOfRangeSamples = record.hasAdcOutOfRangeSamples;
        result.thresholdCodes = record.thresholdCodes;

        // 严格超过门限才能检出；门限达到最大可用幅值时，不可能检出。
        // 保留基线及门限诊断，返回空测量，而不是把有效的高门限当错误。
        if (result.thresholdCodes >= double(adc.MaximumCode()) - result.baseline.meanCode)
            return result;

        const LidarDigitalRangeEstimator estimator(result.baseline.meanCode, result.thresholdCodes);
        switch (config.rangeMethod)
        {
        case LidarReceiverRangeMethod::Peak:
            result.measured = estimator.Estimate(digital, emissionTimeSeconds, config.searchWindow);
            break;
        case LidarReceiverRangeMethod::Centroid:
            result.measured = estimator.EstimateCentroid(digital, emissionTimeSeconds, config.searchWindow);
            break;
        case LidarReceiverRangeMethod::PeakNeighborhoodCentroid:
            result.measured = estimator.EstimatePeakNeighborhoodCentroid(
                digital, emissionTimeSeconds, config.neighborhood, config.searchWindow);
            break;
        default:
            throw std::logic_error("Unexpected receiver range method.");
        }
        if (result.measured && config.calibration)
            result.corrected = config.calibration->Apply(*result.measured);
        return result;
    }

    // 显式多回波入口：复用同一模拟接收/ADC链路，不自动填入旧的单回波结果。
    // 扫描驱动按 echoMode 选择入口，多回波集合由点云构建器逐个投影。
    LidarReceiverMultiEchoResult ProcessMultiEcho(
        const LidarWaveform& opticalWaveform,
        double emissionTimeSeconds,
        std::uint64_t noiseSeed) const
    {
        const auto record = PrepareRecord(opticalWaveform, emissionTimeSeconds, noiseSeed);
        LidarReceiverMultiEchoResult result;
        result.baseline = record.baseline;
        result.thresholdCodes = record.thresholdCodes;
        result.hasAdcOutOfRangeSamples = record.hasAdcOutOfRangeSamples;
        if (result.thresholdCodes >= double(adc.MaximumCode()) - result.baseline.meanCode)
            return result;

        const auto estimates = multiEchoEstimator.Estimate(record.digital, emissionTimeSeconds,
            config.searchWindow, result.baseline.meanCode, result.thresholdCodes);
        result.echoes.reserve(estimates.size());
        for (const auto& estimate : estimates)
        {
            LidarReceiverEchoResult echo{estimate, std::nullopt};
            if (config.calibration)
                echo.corrected = config.calibration->Apply(estimate.measured);
            result.echoes.push_back(std::move(echo));
        }
        return result;
    }

    const LidarReceiverPipelineConfig& Config() const { return config; }

private:
    struct PreparedRecord
    {
        LidarDigitalWaveform digital;
        LidarDigitalBaselineEstimate baseline;
        double thresholdCodes;
        bool hasAdcOutOfRangeSamples;
    };

    // 两种处理入口共享前端：每次调用只生成一次噪声、一次 ADC、一次基线估计。
    PreparedRecord PrepareRecord(const LidarWaveform& opticalWaveform,
        double emissionTimeSeconds, std::uint64_t noiseSeed) const
    {
        if (!std::isfinite(emissionTimeSeconds) || emissionTimeSeconds < 0.0)
            throw std::invalid_argument("Emission time must be finite and nonnegative.");
        const auto current = LidarAnalogWaveform::FromOpticalWaveform(opticalWaveform, detector);
        const auto received = response.Apply(current);
        const auto voltage = LidarVoltageWaveform::FromCurrentWaveform(received, amplifier);
        const auto noisyVoltage = noise.Apply(voltage, noiseSeed);
        auto digital = LidarDigitalWaveform::FromVoltageWaveform(noisyVoltage, adc);
        const auto baseline = LidarDigitalBaselineEstimator::Estimate(digital, config.noiseWindow);
        const bool clipped = digital.HasOutOfRangeSamples();
        const double noiseThreshold = config.thresholdMultiplier * baseline.sigmaCodes;
        if (!std::isfinite(noiseThreshold))
            throw std::overflow_error("Estimated detection threshold overflow.");
        const double threshold = std::max(config.minimumThresholdCodes, noiseThreshold);
        return {std::move(digital), baseline, threshold, clipped};
    }

    static void ValidateWindow(const LidarReturnWindow& window)
    {
        if (!std::isfinite(window.startTimeSeconds) || !std::isfinite(window.endTimeSeconds) ||
            window.endTimeSeconds <= window.startTimeSeconds)
            throw std::invalid_argument("Receiver window must be finite with positive duration.");
    }

    LidarReceiverPipelineConfig config;
    LinearPhotodetector detector;
    FirstOrderLidarResponse response;
    IdealTransimpedanceAmplifier amplifier;
    LidarVoltageNoise noise;
    IdealUniformAdc adc;
    LidarDigitalMultiEchoEstimator multiEchoEstimator;
};
