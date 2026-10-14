#pragma once

#include "lidar/simulation/LidarPulseMetadata.h"
#include "lidar/receiver/LidarReceiverPipeline.h"

struct LidarPulseMeasurement
{
    LidarPulseMetadata pulse;
    LidarReceiverPipelineResult reception;
};

struct LidarMultiEchoPulseMeasurement
{
    LidarPulseMetadata pulse;
    LidarReceiverMultiEchoResult reception;
};
