#pragma once
#include "lidar/scan/LidarScanPlan.h"
#include <algorithm>
#include <array>
#include <vector>

struct LidarPoseKeyframe
{
    double timeSeconds = 0.0;
    LidarScanPoseConfig pose;
};

// 传感器世界位姿的有限关键帧轨迹；线性平移 + 最短弧 quaternion SLERP。
// 不外推，不是动力学或 IMU 模型；车体轨迹与安装外参的组合尚未实现。
class LidarPoseTrajectory
{
public:
    explicit LidarPoseTrajectory(const std::vector<LidarPoseKeyframe>& frames) : frames(frames)
    {
        if(frames.empty())throw std::invalid_argument("Pose trajectory needs at least one keyframe.");
        for(std::size_t i=0;i<frames.size();++i) {
            if(!std::isfinite(frames[i].timeSeconds)||frames[i].timeSeconds<0||
                (i && frames[i].timeSeconds<=frames[i-1].timeSeconds))
                throw std::invalid_argument("Pose keyframe times must be finite, nonnegative and increasing.");
            rotations.push_back(Quaternion(frames[i].pose));
        }
    }
    double StartTimeSeconds() const {return frames.front().timeSeconds;}
    double EndTimeSeconds() const {return frames.back().timeSeconds;}
    LidarScanPoseConfig PoseAt(double time) const
    {
        if(!std::isfinite(time))throw std::invalid_argument("Pose query time must be finite.");
        if(time<StartTimeSeconds()||time>EndTimeSeconds())
            throw std::out_of_range("Pose query lies outside trajectory coverage.");
        const auto it=std::lower_bound(frames.begin(),frames.end(),time,
            [](const LidarPoseKeyframe& key,double t){return key.timeSeconds<t;});
        if(it->timeSeconds==time)return it->pose;
        const auto upper=std::size_t(it-frames.begin()),lower=upper-1;
        const double u=(time-frames[lower].timeSeconds)/(frames[upper].timeSeconds-frames[lower].timeSeconds);
        const auto q=Slerp(rotations[lower],rotations[upper],u);
        const auto& a=frames[lower].pose.originWorld;
        const auto& b=frames[upper].pose.originWorld;
        return {{float(std::lerp(double(a.x),double(b.x),u)),
                 float(std::lerp(double(a.y),double(b.y),u)),
                 float(std::lerp(double(a.z),double(b.z),u))},
                Rotate(q,{0,0,1}),Rotate(q,{0,1,0})};
    }

    // 与 PCD VIEWPOINT 共用姿态转换，四元数顺序 w,x,y,z。
    static std::array<double,4> Quaternion(const LidarScanPoseConfig& config)
    {
        const LidarScanPose pose(config);
        const auto a=pose.ToWorld({0,0,0,{0,0,0},{1,0,0},std::nullopt}).CenterDirectionWorld();
        const auto b=pose.ToWorld({0,0,0,{0,0,0},{0,1,0},std::nullopt}).CenterDirectionWorld();
        const auto c=pose.ToWorld({0,0,0,{0,0,0},{0,0,1},std::nullopt}).CenterDirectionWorld();
        const double m00=a.x,m01=b.x,m02=c.x,m10=a.y,m11=b.y,m12=c.y,m20=a.z,m21=b.z,m22=c.z;
        std::array<double,4> q;
        const double trace=m00+m11+m22;
        if(trace>0) {const double s=2*std::sqrt(trace+1);q={s/4,(m21-m12)/s,(m02-m20)/s,(m10-m01)/s};}
        else if(m00>m11&&m00>m22) {const double s=2*std::sqrt(1+m00-m11-m22);q={(m21-m12)/s,s/4,(m01+m10)/s,(m02+m20)/s};}
        else if(m11>m22) {const double s=2*std::sqrt(1+m11-m00-m22);q={(m02-m20)/s,(m01+m10)/s,s/4,(m12+m21)/s};}
        else {const double s=2*std::sqrt(1+m22-m00-m11);q={(m10-m01)/s,(m02+m20)/s,(m12+m21)/s,s/4};}
        return Normalize(q);
    }
private:
    static std::array<double,4> Normalize(std::array<double,4> q) {
        const double length=std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
        for(auto& value:q)value/=length;return q;
    }
    static std::array<double,4> Slerp(std::array<double,4> a,std::array<double,4> b,double u) {
        double dot=0;for(int i=0;i<4;++i)dot+=a[i]*b[i];
        if(dot<0){for(auto& v:b)v=-v;dot=-dot;}
        dot=std::clamp(dot,0.0,1.0);
        std::array<double,4> q;
        if(dot>.9995)for(int i=0;i<4;++i)q[i]=std::lerp(a[i],b[i],u);
        else {
            const double angle=std::acos(dot),denom=std::sin(angle);
            const double wa=std::sin((1-u)*angle)/denom,wb=std::sin(u*angle)/denom;
            for(int i=0;i<4;++i)q[i]=wa*a[i]+wb*b[i];
        }
        return Normalize(q);
    }
    static Vector3f Rotate(const std::array<double,4>& q,const Vector3f& v) {
        const double w=q[0],x=q[1],y=q[2],z=q[3];
        return {float((1-2*(y*y+z*z))*v.x+2*(x*y-w*z)*v.y+2*(x*z+w*y)*v.z),
                float(2*(x*y+w*z)*v.x+(1-2*(x*x+z*z))*v.y+2*(y*z-w*x)*v.z),
                float(2*(x*z-w*y)*v.x+2*(y*z+w*x)*v.y+(1-2*(x*x+y*y))*v.z)};
    }
    std::vector<LidarPoseKeyframe> frames;
    std::vector<std::array<double,4>> rotations;
};
