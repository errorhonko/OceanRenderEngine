#pragma once
#include "lidar/output/LidarPointCloudFrame.h"
#include "lidar/scan/LidarScanDriver.h"
#include <algorithm>
#include <limits>
#include <unordered_set>

struct LidarMultiFrameConfig
{
    std::size_t frameCount = 1;
    double framePeriodSeconds = 0.1;
    std::uint64_t firstFrameId = 0;
};
struct LidarSequenceFrameResult
{
    LidarPointCloudFrame cloud;
    std::vector<LidarScanPulseResult> pulses; // 包含漏检及被拒绝的测量
};

// 重复一个有限扫描计划；帧间平移时间和 ID，不缓存所有帧。
// 位姿可固定或使用轨迹快照；不自动更新场景或模拟跨脉冲接收状态。
// world/scattering/profile 必须保持存活；原始计划只在构造时读取。
class LidarMultiFrameDriver
{
public:
    LidarMultiFrameDriver(const Hittable& world,const LidarScatteringModel& scattering,
        const LidarPulseProfile& profile,const LidarScanPlan& prototype,
        const LidarScanPoseConfig& pose,const LidarScanDriverConfig& reception,
        const LidarMultiFrameConfig& sequence)
        : world(world),scattering(scattering),profile(profile),pose(pose),reception(reception),sequence(sequence)
    {
        if(sequence.frameCount==0 || !std::isfinite(sequence.framePeriodSeconds) || sequence.framePeriodSeconds<=0)
            throw std::invalid_argument("Multi-frame count and period must be positive.");
        if(sequence.frameCount-1>std::numeric_limits<std::uint64_t>::max()-sequence.firstFrameId)
            throw std::invalid_argument("Frame ids overflow.");
        const std::size_t n=prototype.Size();
        if(n==0)throw std::invalid_argument("Prototype scan plan must be nonempty.");
        events.reserve(n);
        std::unordered_set<std::uint64_t> ids;
        std::uint64_t low=std::numeric_limits<std::uint64_t>::max(),high=0;
        for(std::size_t i=0;i<n;++i) {
            auto e=prototype.Firing(i);
            if(!std::isfinite(e.emissionTimeSeconds) || e.emissionTimeSeconds<0 ||
                (i && e.emissionTimeSeconds<events.back().emissionTimeSeconds) || !ids.insert(e.pulseId).second)
                throw std::invalid_argument("Prototype firing times or ids are invalid.");
            low=std::min(low,e.pulseId);high=std::max(high,e.pulseId);
            events.push_back(e);
        }
        firstTime=events.front().emissionTimeSeconds;
        const double span=events.back().emissionTimeSeconds-firstTime;
        if(span>=sequence.framePeriodSeconds)
            throw std::invalid_argument("Repeated firing windows must not overlap across frames.");
        const auto idSpan=high-low;
        if(sequence.frameCount>1 && idSpan==std::numeric_limits<std::uint64_t>::max())
            throw std::invalid_argument("Prototype id span cannot be repeated.");
        idStride=idSpan==std::numeric_limits<std::uint64_t>::max()?0:idSpan+1;
        if(sequence.frameCount>1) {
            const auto last=std::uint64_t(sequence.frameCount-1);
            if(idStride>std::numeric_limits<std::uint64_t>::max()/last ||
                high>std::numeric_limits<std::uint64_t>::max()-idStride*last)
                throw std::invalid_argument("Repeated pulse ids overflow.");
            if(double(sequence.frameCount-1)==double(sequence.frameCount-2))
                throw std::invalid_argument("Frame indices cannot be distinguished in double.");
        }
        const double end=std::fma(double(sequence.frameCount),sequence.framePeriodSeconds,firstTime);
        if(!std::isfinite(end))throw std::invalid_argument("Multi-frame final time overflows.");
        const double spacing=std::nextafter(end,std::numeric_limits<double>::infinity())-end;
        for(std::size_t i=1;i<events.size();++i) {
            const double gap=events[i].emissionTimeSeconds-events[i-1].emissionTimeSeconds;
            if(gap>0 && gap<spacing)
                throw std::invalid_argument("Distinct prototype firing times collapse in later frames.");
        }
        if(sequence.framePeriodSeconds-span<spacing)
            throw std::invalid_argument("Repeated frame times cannot be distinguished.");
        // 复用单帧参数/位姿/波形时间检查；不在构造中进行场景追踪。
        const auto first=MakePlan(0);
        const LidarScanDriver checkFirst(world,scattering,profile,first,pose,reception);
        const auto last=MakePlan(sequence.frameCount-1);
        const LidarScanDriver checkLast(world,scattering,profile,last,pose,reception);
    }

    std::size_t Size() const {return sequence.frameCount;}
    LidarMultiFrameDriver(const Hittable& world,const LidarScatteringModel& scattering,
        const LidarPulseProfile& profile,const LidarScanPlan& prototype,
        const LidarPoseTrajectory& motion,const LidarScanDriverConfig& reception,
        const LidarMultiFrameConfig& sequence)
        : LidarMultiFrameDriver(world,scattering,profile,prototype,
            InitialPose(prototype,motion),reception,sequence)
    {
        motion.PoseAt(std::fma(double(sequence.frameCount-1),sequence.framePeriodSeconds,events.back().emissionTimeSeconds));
        trajectory=motion; // 快照，源轨迹销毁或替换后仍可重放。
    }
    std::uint64_t PulseIdStride() const {return idStride;}
    LidarSequenceFrameResult SimulateFrame(std::size_t index) const
    {
        const auto plan=MakePlan(index);
        auto result=trajectory
            ? LidarScanDriver(world,scattering,profile,plan,*trajectory,reception).SimulateFrame()
            : LidarScanDriver(world,scattering,profile,plan,pose,reception).SimulateFrame();
        LidarPointCloudFrame cloud;
        cloud.frameId=sequence.firstFrameId+std::uint64_t(index);
        cloud.startTimeSeconds=std::fma(double(index),sequence.framePeriodSeconds,firstTime);
        cloud.endTimeSeconds=std::fma(double(index+1),sequence.framePeriodSeconds,firstTime);
        cloud.sensorPose=trajectory?trajectory->PoseAt(cloud.startTimeSeconds):pose;
        cloud.points=std::move(result.points);
        return {std::move(cloud),std::move(result.pulses)};
    }
private:
    static LidarScanPoseConfig InitialPose(const LidarScanPlan& plan,const LidarPoseTrajectory& motion) {
        if(plan.Size()==0)throw std::invalid_argument("Prototype scan plan must be nonempty.");
        return motion.PoseAt(plan.Firing(0).emissionTimeSeconds);
    }
    class FramePlan final : public LidarScanPlan {
    public:
        std::vector<LidarFiringEvent> events;
        std::size_t Size() const override {return events.size();}
        LidarFiringEvent Firing(std::size_t i) const override {return events.at(i);}
    };
    FramePlan MakePlan(std::size_t index) const {
        if(index>=sequence.frameCount)throw std::out_of_range("Frame index is out of range.");
        FramePlan p;p.events=events;
        const auto offset=std::uint64_t(index)*idStride;
        for(auto& e:p.events) {
            e.pulseId+=offset;
            e.emissionTimeSeconds=std::fma(double(index),sequence.framePeriodSeconds,e.emissionTimeSeconds);
        }
        return p;
    }
    const Hittable& world;
    const LidarScatteringModel& scattering;
    const LidarPulseProfile& profile;
    LidarScanPoseConfig pose;
    LidarScanDriverConfig reception;
    LidarMultiFrameConfig sequence;
    std::vector<LidarFiringEvent> events;
    std::uint64_t idStride=0;
    double firstTime=0;
    std::optional<LidarPoseTrajectory> trajectory;
};
