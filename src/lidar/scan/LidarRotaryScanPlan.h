#pragma once

#include "lidar/scan/LidarScanPlan.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_set>
#include <vector>

struct LidarRotaryChannel
{
    std::uint32_t channelId = 0;
    double elevationRadians = 0.0;
    double azimuthOffsetRadians = 0.0;
    // 相对每列起点的发射偏移，单位 s；默认 0 表示同时发射。
    double firingOffsetSeconds = 0.0;
    Vector3f originSensor{0,0,0};
};

struct LidarRotaryScanConfig
{
    std::vector<LidarRotaryChannel> channels;
    std::size_t azimuthSamplesPerRevolution = 1024;
    double rotationFrequencyHz = 10.0;
    double startAzimuthRadians = 0.0;
    double startTimeSeconds = 0.0;
    std::uint64_t firstPulseId = 0;
};

// 一圈的有限发射计划：局部 Y 为旋转轴，正方位从 +Z 转向 +X。
// 每列按通道发射偏移排序，同偏移保持配置顺序；不重复采样圈末端。
// 理想恒定转速、不模拟机械扰动、通道串扰或接收状态连续性。
class LidarRotaryScanPlan final : public LidarScanPlan
{
public:
    explicit LidarRotaryScanPlan(const LidarRotaryScanConfig& config)
        : config(config)
    {
        if (config.channels.empty())
            throw std::invalid_argument("Rotary scan needs at least one channel.");
        if (config.azimuthSamplesPerRevolution == 0 ||
            config.azimuthSamplesPerRevolution > std::numeric_limits<std::size_t>::max()/config.channels.size())
            throw std::invalid_argument("Rotary scan dimensions are invalid.");
        count = config.azimuthSamplesPerRevolution * config.channels.size();
        if (count-1 > std::numeric_limits<std::uint64_t>::max()-config.firstPulseId)
            throw std::invalid_argument("Rotary pulse ids overflow.");
        if (!std::isfinite(config.rotationFrequencyHz) || config.rotationFrequencyHz<=0 ||
            !std::isfinite(config.startTimeSeconds) || config.startTimeSeconds<0 ||
            !std::isfinite(config.startAzimuthRadians))
            throw std::invalid_argument("Rotary frequency, time or starting angle is invalid.");
        duration = 1.0/config.rotationFrequencyHz;
        columnPeriod = duration/double(config.azimuthSamplesPerRevolution);
        if (!std::isfinite(duration) || !std::isfinite(columnPeriod) || columnPeriod<=0)
            throw std::invalid_argument("Rotary time interval is not representable.");
        const double frameEnd = config.startTimeSeconds+duration;
        if (!std::isfinite(frameEnd) || frameEnd<=config.startTimeSeconds)
            throw std::invalid_argument("Rotary frame end is not representable.");
        startAngle = Wrap(config.startAzimuthRadians);
        std::unordered_set<std::uint32_t> ids;
        for (const auto& channel:config.channels)
        {
            if (!ids.insert(channel.channelId).second)
                throw std::invalid_argument("Rotary channel ids must be unique.");
            if (!std::isfinite(channel.elevationRadians) || std::abs(channel.elevationRadians)>Pi/2 ||
                !std::isfinite(channel.azimuthOffsetRadians) ||
                !std::isfinite(channel.firingOffsetSeconds) || channel.firingOffsetSeconds<0 ||
                channel.firingOffsetSeconds>=columnPeriod ||
                !std::isfinite(channel.originSensor.x) || !std::isfinite(channel.originSensor.y) ||
                !std::isfinite(channel.originSensor.z))
                throw std::invalid_argument("Rotary channel parameters are invalid.");
        }
        order.resize(config.channels.size());
        std::iota(order.begin(),order.end(),std::size_t(0));
        std::stable_sort(order.begin(),order.end(),[&](std::size_t a,std::size_t b) {
            return config.channels[a].firingOffsetSeconds<config.channels[b].firingOffsetSeconds;
        });
        const double lastOffset = config.channels[order.back()].firingOffsetSeconds;
        const double lastTime = std::fma(double(config.azimuthSamplesPerRevolution-1),columnPeriod,
            config.startTimeSeconds)+lastOffset;
        if (!std::isfinite(lastTime) || lastTime>=frameEnd)
            throw std::invalid_argument("Rotary final firing must precede the next revolution.");
        const double spacing=std::nextafter(frameEnd,std::numeric_limits<double>::infinity())-frameEnd;
        for (const auto& channel:config.channels)
            if (channel.firingOffsetSeconds>0 && channel.firingOffsetSeconds<spacing)
                throw std::invalid_argument("Nonzero rotary firing offset cannot be represented at this clock.");
        for (std::size_t i=1;i<order.size();++i)
        {
            const double difference=config.channels[order[i]].firingOffsetSeconds-
                config.channels[order[i-1]].firingOffsetSeconds;
            if (difference>0 && difference<spacing)
                throw std::invalid_argument("Distinct rotary firing offsets cannot be distinguished.");
        }
        if (config.azimuthSamplesPerRevolution>1)
        {
            const double firstOffset=config.channels[order.front()].firingOffsetSeconds;
            if (double(config.azimuthSamplesPerRevolution-1)==double(config.azimuthSamplesPerRevolution-2) ||
                columnPeriod-lastOffset+firstOffset<spacing)
                throw std::invalid_argument("Rotary column times cannot be distinguished.");
        }
    }

    std::size_t Size() const override {return count;}
    const LidarRotaryScanConfig& Config() const {return config;}
    double FrameDurationSeconds() const {return duration;}
    double ColumnPeriodSeconds() const {return columnPeriod;}

    LidarFiringEvent Firing(std::size_t index) const override
    {
        if (index>=count) throw std::out_of_range("Rotary firing index is out of range.");
        const std::size_t column=index/order.size();
        const auto& channel=config.channels[order[index%order.size()]];
        const double time=std::fma(double(column),columnPeriod,config.startTimeSeconds)+channel.firingOffsetSeconds;
        // 用相对列/通道偏移计算角度，绝不做 time-startTime 的大时刻相减。
        const double azimuth=Wrap(startAngle+TwoPi*double(column)/double(config.azimuthSamplesPerRevolution)+
            TwoPi*(channel.firingOffsetSeconds/columnPeriod)/double(config.azimuthSamplesPerRevolution)+
            Wrap(channel.azimuthOffsetRadians));
        const double cosElevation=std::cos(channel.elevationRadians);
        return {config.firstPulseId+std::uint64_t(index),channel.channelId,time,channel.originSensor,
            {float(cosElevation*std::sin(azimuth)),float(std::sin(channel.elevationRadians)),
             float(cosElevation*std::cos(azimuth))},std::nullopt};
    }

private:
    static constexpr double Pi=3.14159265358979323846;
    static constexpr double TwoPi=2*Pi;
    static double Wrap(double angle) {
        const double wrapped=std::fmod(angle,TwoPi);
        return wrapped<0 ? wrapped+TwoPi : wrapped;
    }
    LidarRotaryScanConfig config;
    std::vector<std::size_t> order;
    std::size_t count=0;
    double duration=0,columnPeriod=0,startAngle=0;
};
