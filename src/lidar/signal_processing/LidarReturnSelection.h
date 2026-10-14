#pragma once

// Output selection AFTER calibration availability and clipping rejection.
// First/Last refer to valid detected time order, not to an unobserved true target.
// Strongest compares peakAboveBaselineCodes, NOT reflectance or integrated energy;
// equal amplitudes choose the earlier return. All is the backwards-compatible default.
enum class LidarReturnSelection
{
    All,
    First,
    Strongest,
    Last
};
