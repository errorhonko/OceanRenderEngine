#pragma once

#include "FirstOrderLidarResponse.h"
#include "IdealTransimpedanceAmplifier.h"
#include "IdealUniformAdc.h"
#include "LidarAnalogWaveform.h"
#include "LidarDigitalBaselineEstimator.h"
#include "LidarDigitalRangeEstimator.h"
#include "LidarDigitalWaveform.h"
#include "LidarRangeCalibration.h"
#include "LidarVoltageNoise.h"
#include "LidarVoltageWaveform.h"
#include "LidarWaveform.h"
#include "LinearPhotodetector.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>

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
              config.adcMaximumVoltageV)
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
        if (!std::isfinite(emissionTimeSeconds) || emissionTimeSeconds < 0.0)
            throw std::invalid_argument("Emission time must be finite and nonnegative.");

        const auto current = LidarAnalogWaveform::FromOpticalWaveform(opticalWaveform, detector);
        const auto received = response.Apply(current);
        const auto voltage = LidarVoltageWaveform::FromCurrentWaveform(received, amplifier);
        const auto noisyVoltage = noise.Apply(voltage, noiseSeed);
        const auto digital = LidarDigitalWaveform::FromVoltageWaveform(noisyVoltage, adc);

        LidarReceiverPipelineResult result;
        result.baseline = LidarDigitalBaselineEstimator::Estimate(digital, config.noiseWindow);
        result.hasAdcOutOfRangeSamples = digital.HasOutOfRangeSamples();
        const double noiseThreshold = config.thresholdMultiplier * result.baseline.sigmaCodes;
        if (!std::isfinite(noiseThreshold))
            throw std::overflow_error("Estimated detection threshold overflow.");
        result.thresholdCodes = std::max(config.minimumThresholdCodes, noiseThreshold);

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

    const LidarReceiverPipelineConfig& Config() const { return config; }

private:
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
};
