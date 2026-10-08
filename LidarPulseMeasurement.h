#pragma once

#include "LidarPulseMetadata.h"
#include "LidarReceiverPipeline.h"

struct LidarPulseMeasurement
{
    LidarPulseMetadata pulse;
    LidarReceiverPipelineResult reception;
};