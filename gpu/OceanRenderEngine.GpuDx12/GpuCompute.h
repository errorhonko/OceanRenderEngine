#pragma once
#include <cstdint>
#include <array>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace ocean::gpu {
struct DeviceInfo {
    std::wstring adapterName;
    std::uint64_t dedicatedVideoMemoryBytes = 0;
    bool softwareAdapter = false;
    unsigned raytracingTier = 0; // 0: unsupported, 10: DXR 1.0, 11: DXR 1.1
    bool debugLayerEnabled = false;
};

struct Float3 { float x, y, z; };
struct QueryRay {
    Float3 origin;
    float tMin = 0.0f;
    Float3 direction;
    float tMax = 1000.0f;
};
struct TriangleQueryResult {
    std::uint32_t hit = 0;
    std::uint32_t primitiveId = UINT32_MAX;
    float rayT = 0.0f;
    float baryU = 0.0f;
    float baryV = 0.0f;
    std::uint32_t occluded = 0;
};

struct GpuStatistics {
    std::uint64_t sceneBuildCount = 0;
    std::uint64_t rayDispatchCount = 0;
    std::uint64_t batchBufferGrowthCount = 0;
};

// Persistent scene, move-only. Resources keep their creating context alive.
// Only the creating GpuCompute can trace this scene; a moved-from scene is invalid.
class GpuTriangleScene {
public:
    ~GpuTriangleScene();
    GpuTriangleScene(GpuTriangleScene&&) noexcept;
    GpuTriangleScene& operator=(GpuTriangleScene&&) noexcept;
    GpuTriangleScene(const GpuTriangleScene&) = delete;
    GpuTriangleScene& operator=(const GpuTriangleScene&) = delete;
    bool Valid() const noexcept;
private:
    struct Impl;
    std::unique_ptr<Impl> impl;
    explicit GpuTriangleScene(std::unique_ptr<Impl> implementation);
    friend class GpuCompute;
};

// Synchronous checkpoint API. One instance owns one device/queue; not thread-safe.
// WARP is opt-in for infrastructure verification, never a hardware benchmark.
class GpuCompute {
public:
    explicit GpuCompute(bool useWarp = false);
    ~GpuCompute();
    GpuCompute(const GpuCompute&) = delete;
    GpuCompute& operator=(const GpuCompute&) = delete;
    const DeviceInfo& Info() const noexcept;
    GpuStatistics Statistics() const noexcept;
    std::vector<float> Transform(std::span<const float> input, float scale, float bias);
    // Upload/build once, then submit multiple ray batches against the same scene.
    GpuTriangleScene BuildTriangleScene(const std::array<Float3, 3>& vertices);
    std::vector<TriangleQueryResult> TraceRays(
        const GpuTriangleScene& scene, std::span<const QueryRay> rays);
    // Checkpoint 2: one opaque, two-sided triangle, identity instance.
    // P(t)=origin+t*direction; direction is NOT normalized by this API.
    // rayT is distance only for a unit direction. Query interval is [tMin,tMax].
    // Compatibility convenience wrapper: builds a temporary scene on every call.
    std::vector<TriangleQueryResult> TraceTriangle(
        const std::array<Float3, 3>& vertices, std::span<const QueryRay> rays);
private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
}
