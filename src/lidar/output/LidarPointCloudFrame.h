#pragma once
#include "lidar/output/LidarPointSample.h"
#include "lidar/scan/LidarScanPlan.h"
#include <vector>

// 当前输出为世界坐标（Y 向上，单位 m），尚不是 ROS 车体坐标。
struct LidarPointCloudFrame
{
    std::uint64_t frameId = 0;
    double startTimeSeconds = 0.0; // 第一条计划发射的时刻
    double endTimeSeconds = 0.0;   // 下一帧起点，采用半开区间
    LidarScanPoseConfig sensorPose;
    std::vector<LidarPointSample> points;
};
