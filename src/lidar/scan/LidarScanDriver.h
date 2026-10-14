#pragma once

#include "core/sampling/IndependentSampler.h"
#include "core/sampling/Stratified2DSampler.h"
#include "lidar/simulation/LidarIntegrator.h"
#include "lidar/output/LidarPointCloudBuilder.h"
#include "lidar/emitter/pulse/LidarPulseProfile.h"
#include "lidar/receiver/LidarReceiverPipeline.h"
#include "lidar/scan/LidarScanSequence.h"
#include "lidar/simulation/LidarWaveform.h"
#include "lidar/scan/LidarPoseTrajectory.h"
#include <functional>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <unordered_set>
#include <vector>

enum class LidarRaySamplingMode
{
    Independent,
    Stratified2D
};

enum class LidarScanEchoMode { Single, Multi };

struct LidarScanDriverConfig
{
    LidarScanSequenceConfig scan;
    // 仅供旧的四参数矩形快捷构造使用。通用构造由 plan / pose 提供扫描信息。
    float wavelengthNm = 532.0f;
    float pulseEnergyJ = 1.0e-9f;
    float divergenceHalfAngleRadians = 0.0f;
    std::size_t samplesPerPulse = 64;
    float receiverFovHalfAngleRadians = 0.02f;
    float receiverApertureAreaSquareMeters = 0.01f;
    float receiverOpticalEfficiency = 0.8f;

    // 这里的起始时刻与两个接收窗口均为相对发射时刻的偏移，单位 s。
    // 底层 LidarWaveform / Pipeline 仍使用绝对仿真时间，由驱动平移。
    LidarWaveformConfig waveform{0.0, 0.25e-9, 1024};
    LidarReceiverPipelineConfig reception;
    LidarPointRangeSource rangeSource = LidarPointRangeSource::Raw;
    bool rejectClippedReturns = true;
    std::uint64_t raySeed = 42;
    std::uint64_t noiseSeed = 43;

    // 可选空间光束，空指针使用旧的 divergenceHalfAngleRadians 配置。
    // 非空时要求旧半角为 0；配置快照共享持有只读模型，随机状态由 Sampler 管理。
    std::shared_ptr<const LidarBeamProfile> beamProfile;

    // 仅控制单脉冲光斑内部的 MC 采样，不改变扫描方向计划或光束物理分布。
    // 默认保留旧独立随机模式；分层模式必须完整使用 xStrata * yStrata 个格子。
    LidarRaySamplingMode raySamplingMode = LidarRaySamplingMode::Independent;
    std::size_t xStrata = 8;
    std::size_t yStrata = 8; // 这两个字段仅在 Stratified2D 模式下验证和使用。
    LidarScanEchoMode echoMode = LidarScanEchoMode::Single; // 保留旧默认行为。
    // 非 All 策略要求 Multi，避免把旧全局测距结果误当作第一/最后回波。
    LidarReturnSelection returnSelection = LidarReturnSelection::All;
};

struct LidarScanPulseResult
{
    LidarPlannedPulse scan;
    LidarReceiverPipelineResult reception;
    std::size_t emittedRayCount = 0;
    std::size_t surfaceHitCount = 0;
    std::size_t returnSampleCount = 0;
    double receivedEnergyJ = 0.0;
    double recordedEnergyJ = 0.0;
    std::uint32_t raySeed = 0;
    std::uint64_t noiseSeed = 0;
    // 漏检、无校正结果或拒绝削顶时为空，记录本身仍保留。
    std::optional<LidarPointSample> point;
    LidarScanPoseConfig sensorPose; // 此脉冲实际使用的世界位姿
    // 多回波模式使用此记录；旧 reception / point 留空，不偷偷选择第一/最强回波。
    std::optional<LidarReceiverMultiEchoResult> multiReception;
    std::vector<LidarPointSample> points; // 两种模式统一的有效点集合。
};

struct LidarScanFrameResult
{
    std::vector<LidarScanPulseResult> pulses; // 发射顺序，包含未生成点的脉冲
    std::vector<LidarPointSample> points;   // 仅有效测量点，不是 MC 命中点
};

// 场景不自动更新；位姿可固定或按发射时刻查询轨迹，共址收发。
// world / scattering / profile 由调用方拥有，必须比本驱动活得更久。
// 单脉冲传播期间冻结收发位置，不做估计位姿去畸变或串扰；多回波只做峰候选分离。
// 每个脉冲独立接收、响应从零开始；波形与 MC 返回数组只临时保留。
class LidarScanDriver
{
public:
    LidarScanDriver(const Hittable& world,
        const LidarScatteringModel& scattering,
        const LidarPulseProfile& profile,
        const LidarScanDriverConfig& config)
        : LidarScanDriver(world, scattering, profile, LidarScanSequence(config.scan),
            LidarScanPoseConfig{config.scan.originWorld,config.scan.forwardWorld,config.scan.upHintWorld},config)
    {}

    // 通用入口：只在构造时读取 plan，快照到本帧；不保存计划引用。
    // config.scan 在此入口不使用。安装位姿与扫描方向/时序独立。
    LidarScanDriver(const Hittable& world,
        const LidarScatteringModel& scattering,
        const LidarPulseProfile& profile,
        const LidarScanPlan& plan,
        const LidarScanPoseConfig& poseConfig,
        const LidarScanDriverConfig& config)
        : LidarScanDriver(world,scattering,profile,plan,
            std::function<LidarScanPoseConfig(double)>([poseConfig](double){return poseConfig;}),config)
    {}

    LidarScanDriver(const Hittable& world,const LidarScatteringModel& scattering,
        const LidarPulseProfile& profile,const LidarScanPlan& plan,
        const LidarPoseTrajectory& trajectory,const LidarScanDriverConfig& config)
        : LidarScanDriver(world,scattering,profile,plan,
            std::function<LidarScanPoseConfig(double)>([&trajectory](double t){return trajectory.PoseAt(t);}),config)
    {}

private:
    LidarScanDriver(const Hittable& world,const LidarScatteringModel& scattering,
        const LidarPulseProfile& profile,const LidarScanPlan& plan,
        const std::function<LidarScanPoseConfig(double)>& poseAt,const LidarScanDriverConfig& config)
        : world(world), scattering(scattering), profile(profile), config(config),
          builder(config.rangeSource, config.rejectClippedReturns, config.returnSelection)
    {
        const std::size_t count = plan.Size();
        if (count == 0)
            throw std::invalid_argument("Scan plan must contain at least one firing.");
        std::unordered_set<std::uint64_t> ids;
        plannedPulses.reserve(count);
        for (std::size_t i=0; i<count; ++i)
        {
            auto event = plan.Firing(i);
            if (!ids.insert(event.pulseId).second)
                throw std::invalid_argument("Scan plan pulse ids must be unique.");
            if (i>0 && event.emissionTimeSeconds < plannedPulses.back().pulse.EmissionTimeSeconds())
                throw std::invalid_argument("Scan plan times must be nondecreasing.");
            const auto poseConfig=poseAt(event.emissionTimeSeconds);
            auto pulse = LidarScanPose(poseConfig).ToWorld(event);
            firingPoses.push_back(poseConfig);
            plannedPulses.push_back({i,std::move(event),std::move(pulse)});
        }
        if (config.samplesPerPulse == 0)
            throw std::invalid_argument("Scan samples per pulse must be positive.");
        // 与实际发射复用同一工厂，在追踪前拒绝无效模式、网格或不完整批次。
        const auto validatedSampler = MakeSampler(0);
        if (config.echoMode != LidarScanEchoMode::Single && config.echoMode != LidarScanEchoMode::Multi)
            throw std::invalid_argument("Unknown scan echo mode.");
        if (config.echoMode == LidarScanEchoMode::Single && config.returnSelection != LidarReturnSelection::All)
            throw std::invalid_argument("Return selection requires multi echo detection mode.");
        if (config.beamProfile && config.divergenceHalfAngleRadians != 0.0f)
            throw std::invalid_argument("Specify either a custom beam profile or legacy divergence, not both.");
        if (config.wavelengthNm != config.reception.wavelengthNm)
            throw std::invalid_argument("Emitter and receiver wavelengths must match.");
        if (config.rangeSource == LidarPointRangeSource::Corrected && !config.reception.calibration)
            throw std::invalid_argument("Corrected scan points require an explicit calibration.");

        // 复用现有类验证光学与电子参数，不复制各模块的算法。
        const auto& first = plannedPulses.front();
        const auto& p = first.pulse;
        const LaserEmitter emitter = MakeEmitter(p);
        const LidarReceiver receiver(p.OriginWorld(), p.CenterDirectionWorld(),
            config.receiverFovHalfAngleRadians, config.receiverApertureAreaSquareMeters,
            config.receiverOpticalEfficiency);
        const LidarWaveform waveform(config.waveform);
        const LidarReceiverPipeline pipeline(config.reception);
        const double end = config.waveform.startTimeSeconds +
            double(config.waveform.binCount) * config.waveform.binWidthSeconds;
        if (!std::isfinite(end))
            throw std::invalid_argument("Relative scan record end must be finite.");
        ValidateContainedWindow(config.reception.noiseWindow, end);
        ValidateContainedWindow(config.reception.searchWindow, end);
        ValidateTiming(p.EmissionTimeSeconds());
        ValidateTiming(plannedPulses.back().pulse.EmissionTimeSeconds());
    }

public:
    std::size_t Size() const { return plannedPulses.size(); }
    const LidarScanDriverConfig& Config() const { return config; }

    // 同一个编号独立重放不依赖之前扫描消耗了多少随机样本。
    LidarScanPulseResult SimulatePulse(std::size_t index) const
    {
        const auto& sample = plannedPulses.at(index);
        const auto& p = sample.pulse;
        const auto rayKey = Mix(config.raySeed ^ Mix(p.PulseId()) ^ 0x524159ULL);
        const auto raySeed = std::uint32_t(rayKey) ^ std::uint32_t(rayKey >> 32);
        const auto noiseSeed = Mix(config.noiseSeed ^ Mix(p.PulseId()) ^ 0x4e4f495345ULL);
        // 每个脉冲重新构造一批样本，避免耗尽、跨脉冲复用或重放依赖执行顺序。
        auto sampler = MakeSampler(raySeed);
        const LaserEmitter emitter = MakeEmitter(p);
        const LidarReceiver receiver(p.OriginWorld(), p.CenterDirectionWorld(),
            config.receiverFovHalfAngleRadians, config.receiverApertureAreaSquareMeters,
            config.receiverOpticalEfficiency);
        const LidarIntegrator integrator(world, emitter, receiver, config.samplesPerPulse, scattering);
        const auto pulse = integrator.SimulatePulse(*sampler, p.EmissionTimeSeconds());

        auto waveformConfig = config.waveform;
        waveformConfig.startTimeSeconds += p.EmissionTimeSeconds();
        LidarWaveform waveform(waveformConfig);
        // 积分器已完成 1/N；这里仅按脉冲轮廓合成，不再次除以 N。
        const double recorded = waveform.AccumulatePulse(pulse, profile);
        auto receptionConfig = config.reception;
        receptionConfig.noiseWindow = Shift(config.reception.noiseWindow, p.EmissionTimeSeconds());
        receptionConfig.searchWindow = Shift(config.reception.searchWindow, p.EmissionTimeSeconds());
        const LidarReceiverPipeline pipeline(receptionConfig);
        LidarScanPulseResult result{sample, {}, pulse.emittedRayCount, pulse.surfaceHitCount,
            pulse.returns.size(), pulse.TotalReceivedEnergyJ(), recorded, raySeed, noiseSeed,
            std::nullopt, firingPoses.at(index), std::nullopt, {}};
        if (config.echoMode == LidarScanEchoMode::Single)
        {
            result.reception = pipeline.Process(waveform, p.EmissionTimeSeconds(), noiseSeed);
            result.point = builder.BuildPoint({p, result.reception});
            if (result.point) result.points.push_back(*result.point);
        }
        else
        {
            result.multiReception = pipeline.ProcessMultiEcho(waveform, p.EmissionTimeSeconds(), noiseSeed);
            result.points = builder.BuildPoints({p, *result.multiReception});
        }
        for (auto& point : result.points)
        {
            point.channelId = sample.firing.channelId;
            point.positionSensor = LidarScanPose(firingPoses.at(index)).ToSensor(point.positionWorld);
        }
        if (result.point) result.point = result.points.front();
        return result;
    }

    LidarScanFrameResult SimulateFrame() const
    {
        LidarScanFrameResult result;
        result.pulses.reserve(Size());
        result.points.reserve(Size());
        for (std::size_t i = 0; i < Size(); ++i)
        {
            auto pulse = SimulatePulse(i);
            result.points.insert(result.points.end(), pulse.points.begin(), pulse.points.end());
            result.pulses.push_back(std::move(pulse));
        }
        return result;
    }

private:
    // 构造验证和每个脉冲复用同一采样器选择，不修改光束分布或能量权重。
    std::unique_ptr<Sampler> MakeSampler(std::uint32_t seed) const
    {
        switch (config.raySamplingMode)
        {
        case LidarRaySamplingMode::Independent:
            return std::make_unique<IndependentSampler>(seed);
        case LidarRaySamplingMode::Stratified2D:
        {
            auto sampler = std::make_unique<Stratified2DSampler>(config.xStrata, config.yStrata, seed);
            if (sampler->SamplesPerBatch() != config.samplesPerPulse)
                throw std::invalid_argument("Stratified scan samples per pulse must equal xStrata * yStrata.");
            return sampler;
        }
        default:
            throw std::invalid_argument("Unknown LiDAR ray sampling mode.");
        }
    }

    // 参数验证和每次实际发射复用同一构造选择，避免两条路径使用不同光束。
    LaserEmitter MakeEmitter(const LidarPulseMetadata& pulse) const
    {
        if (config.beamProfile)
            return LaserEmitter(pulse.OriginWorld(), pulse.CenterDirectionWorld(),
                config.wavelengthNm, config.pulseEnergyJ, config.beamProfile);
        return LaserEmitter(pulse.OriginWorld(), pulse.CenterDirectionWorld(),
            config.wavelengthNm, config.pulseEnergyJ, config.divergenceHalfAngleRadians);
    }

    // SplitMix64 混合；这是派生可复现 seed，不是密码学随机数。
    static std::uint64_t Mix(std::uint64_t value)
    {
        value += 0x9e3779b97f4a7c15ULL;
        value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
        value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
        return value ^ (value >> 31);
    }

    static LidarReturnWindow Shift(const LidarReturnWindow& window, double time)
    {
        const LidarReturnWindow shifted{time + window.startTimeSeconds, time + window.endTimeSeconds};
        if (!std::isfinite(shifted.startTimeSeconds) || !std::isfinite(shifted.endTimeSeconds) ||
            shifted.endTimeSeconds <= shifted.startTimeSeconds)
            throw std::invalid_argument("Shifted scan window is not representable.");
        return shifted;
    }

    void ValidateContainedWindow(const LidarReturnWindow& window, double recordEnd) const
    {
        if (window.startTimeSeconds < config.waveform.startTimeSeconds ||
            window.endTimeSeconds > recordEnd)
            throw std::invalid_argument("Scan reception windows must lie inside the record.");
    }

    void ValidateTiming(double time) const
    {
        const double start = time + config.waveform.startTimeSeconds;
        const double end = start + double(config.waveform.binCount) * config.waveform.binWidthSeconds;
        if (!std::isfinite(start) || !std::isfinite(end) || end <= start)
            throw std::invalid_argument("Scan waveform time range is not representable.");
        const double magnitude = std::max(std::abs(start), std::abs(end));
        const double spacing = std::nextafter(magnitude, std::numeric_limits<double>::infinity()) - magnitude;
        if (config.waveform.binWidthSeconds < spacing)
            throw std::invalid_argument("Scan waveform bins cannot be distinguished at this time.");
        const auto noise = Shift(config.reception.noiseWindow, time);
        const auto search = Shift(config.reception.searchWindow, time);
        if (noise.startTimeSeconds < search.endTimeSeconds && search.startTimeSeconds < noise.endTimeSeconds)
            throw std::invalid_argument("Shifted scan reception windows overlap.");
        std::size_t noiseBins = 0;
        for (std::size_t i = 0; i < config.waveform.binCount; ++i)
        {
            const double center = start + (double(i) + 0.5) * config.waveform.binWidthSeconds;
            if (center >= noise.startTimeSeconds && center < noise.endTimeSeconds)
                ++noiseBins;
        }
        if (noiseBins < 2)
            throw std::invalid_argument("Scan noise window needs at least two bin centers.");
    }

    const Hittable& world;
    const LidarScatteringModel& scattering;
    const LidarPulseProfile& profile;
    LidarScanDriverConfig config;
    std::vector<LidarPlannedPulse> plannedPulses;
    std::vector<LidarScanPoseConfig> firingPoses;
    LidarPointCloudBuilder builder;
};
