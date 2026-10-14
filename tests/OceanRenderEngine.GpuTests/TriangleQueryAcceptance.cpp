#include "TriangleQueryAcceptance.h"
#include "GpuCompute.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace ocean::gpu;
struct Double3 { double x, y, z; };
Double3 D(Float3 v) { return {v.x, v.y, v.z}; }
Double3 Sub(Double3 a, Double3 b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
Double3 Cross(Double3 a, Double3 b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
double Dot(Double3 a, Double3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
struct ReferenceHit { bool hit = false; double t = 0, u = 0, v = 0; };

// Independent double-precision, two-sided Moller-Trumbore reference.
// Acceptance cases avoid exact edges/endpoints: hardware tie rules are not assumed.
ReferenceHit Intersect(const std::array<Float3, 3>& triangle, const QueryRay& ray) {
    const auto e1 = Sub(D(triangle[1]), D(triangle[0]));
    const auto e2 = Sub(D(triangle[2]), D(triangle[0]));
    const auto p = Cross(D(ray.direction), e2);
    const double det = Dot(e1, p);
    if (std::abs(det) < 1e-12) return {};
    const auto s = Sub(D(ray.origin), D(triangle[0]));
    const double u = Dot(s, p) / det;
    if (u < 0 || u > 1) return {};
    const auto q = Cross(s, e1);
    const double v = Dot(D(ray.direction), q) / det;
    if (v < 0 || u + v > 1) return {};
    const double t = Dot(e2, q) / det;
    if (t < ray.tMin || t > ray.tMax) return {};
    return {true, t, u, v};
}
void Require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void Near(float actual, double expected, const std::string& label) {
    Require(std::isfinite(actual) && std::abs(actual-expected) <= 2e-5 * std::max(1.0, std::abs(expected)),
        "DXR mismatch: " + label);
}
void Verify(const std::array<Float3, 3>& triangle, std::span<const QueryRay> rays,
    const std::vector<TriangleQueryResult>& results, const std::string& label) {
    Require(results.size() == rays.size(), "DXR output size mismatch: " + label);
    for (std::size_t i = 0; i < rays.size(); ++i) {
        const auto expected = Intersect(triangle, rays[i]);
        const auto& actual = results[i];
        const auto item = label + " ray=" + std::to_string(i);
        Require(actual.hit == (expected.hit ? 1u : 0u), "DXR hit mismatch: " + item);
        Require(actual.occluded == (expected.hit ? 1u : 0u), "DXR occlusion mismatch: " + item);
        if (expected.hit) {
            Require(actual.primitiveId == 0, "DXR primitive mismatch: " + item);
            Near(actual.rayT, expected.t, item + " t");
            Near(actual.baryU, expected.u, item + " u");
            Near(actual.baryV, expected.v, item + " v");
            // Validate the reported barycentrics against the actual point on the input ray.
            const double w = 1.0 - actual.baryU - actual.baryV;
            const double x = w*triangle[0].x + actual.baryU*triangle[1].x + actual.baryV*triangle[2].x;
            const double y = w*triangle[0].y + actual.baryU*triangle[1].y + actual.baryV*triangle[2].y;
            const double z = w*triangle[0].z + actual.baryU*triangle[1].z + actual.baryV*triangle[2].z;
            Near(static_cast<float>(x), rays[i].origin.x + double(actual.rayT)*rays[i].direction.x, item + " point.x");
            Near(static_cast<float>(y), rays[i].origin.y + double(actual.rayT)*rays[i].direction.y, item + " point.y");
            Near(static_cast<float>(z), rays[i].origin.z + double(actual.rayT)*rays[i].direction.z, item + " point.z");
        } else {
            Require(actual.primitiveId == UINT32_MAX && actual.rayT == 0 && actual.baryU == 0 && actual.baryV == 0,
                "DXR miss must have explicit sentinel fields: " + item);
        }
    }
}
template<class F> void Reject(F&& action, const char* label) {
    bool rejected = false;
    try { action(); } catch (const std::invalid_argument&) { rejected = true; }
    Require(rejected, std::string("DXR invalid input accepted: ") + label);
}
}

void RunTriangleQueryAcceptance(ocean::gpu::GpuCompute& gpu) {
    using namespace ocean::gpu;
    if (gpu.Info().softwareAdapter) {
        std::cout << "SKIP DXR triangle acceptance: WARP is compute-only verification here.\n";
        return;
    }
    Require(gpu.Info().raytracingTier >= 11, "Hardware DXR 1.1 is required for triangle acceptance");
    const std::array<Float3, 3> triangle{{{-1,-1,0}, {1,-1,0}, {0,1,0}}};
    const std::vector<QueryRay> cases{
        {{0,0,2}, 0, {0,0,-1}, 10},       // center: t=2, u=.25, v=.5
        {{0,0,-2}, 0, {0,0,1}, 10},       // reverse side, no face culling
        {{2,0,2}, 0, {0,0,-1}, 10},       // outside triangle
        {{0,0,2}, 0, {1,0,0}, 10},        // parallel
        {{0,0,2}, 0, {0,0,1}, 10},        // points away
        {{0,0,2}, 2.25f, {0,0,-1}, 10},   // excluded by tMin
        {{0,0,2}, 0, {0,0,-1}, 1.75f},    // receiver before blocker
        {{0,0,2}, 0, {0,0,-1}, 2.25f},    // receiver after blocker
        {{0,0,2}, 0, {0,0,-2}, 10},       // non-unit direction, t=1
        {{0,0,2}, 0, {0.1f,0,-1}, 10},    // oblique ray
        {{0,-0.99f,2}, 0, {0,0,-1}, 10},  // inside, close to edge
        {{0,0,0.001f}, 0.0001f, {0,0,1}, 10} // outward offset, no self-blocker
    };
    const auto namedResults = gpu.TraceTriangle(triangle, cases);
    Verify(triangle, cases, namedResults, "named cases");
    const bool expectedHits[]{true,true,false,false,false,false,false,true,true,true,true,false};
    for (std::size_t i = 0; i < cases.size(); ++i)
        Require(namedResults[i].hit == (expectedHits[i] ? 1u : 0u), "Analytic case mismatch");
    Near(namedResults[0].rayT, 2, "analytic center t");
    Near(namedResults[0].baryU, .25, "analytic center u");
    Near(namedResults[0].baryV, .5, "analytic center v");
    Near(namedResults[8].rayT, 1, "non-unit parameter t");
    std::cout << "PASS DXR 12 analytic cases (closest hit + finite-segment occlusion)\n";
    std::size_t compared = cases.size();
    for (std::size_t count : {0u, 1u, 63u, 64u, 65u, 257u}) {
        std::vector<QueryRay> batch(count);
        for (std::size_t i = 0; i < count; ++i) {
            batch[i] = cases[i % cases.size()];
            batch[i].origin.x += static_cast<float>(i % 11) * .001f;
        }
        Verify(triangle, batch, gpu.TraceTriangle(triangle, batch), "batch " + std::to_string(count));
        compared += count;
        std::cout << "PASS DXR batch=" << count << '\n';
    }
    auto shifted = triangle;
    for (auto& v : shifted) v.z = .75f;
    Verify(shifted, cases, gpu.TraceTriangle(shifted, cases), "rebuilt shifted triangle");
    auto reversed = triangle;
    std::swap(reversed[1], reversed[2]);
    Verify(reversed, cases, gpu.TraceTriangle(reversed, cases), "reversed winding");
    compared += cases.size() * 2;
    std::cout << "PASS DXR rebuild + reversed winding\n";
    auto invalidRay = cases[0];
    invalidRay.direction = {0,0,0};
    Reject([&] { gpu.TraceTriangle(triangle, std::span{&invalidRay, 1}); }, "zero direction");
    invalidRay = cases[0]; invalidRay.tMax = invalidRay.tMin;
    Reject([&] { gpu.TraceTriangle(triangle, std::span{&invalidRay, 1}); }, "empty interval");
    invalidRay = cases[0]; invalidRay.tMin = -1;
    Reject([&] { gpu.TraceTriangle(triangle, std::span{&invalidRay, 1}); }, "negative tMin");
    invalidRay = cases[0]; invalidRay.origin.x = std::numeric_limits<float>::quiet_NaN();
    Reject([&] { gpu.TraceTriangle(triangle, std::span{&invalidRay, 1}); }, "nonfinite ray");
    auto invalidTriangle = triangle;
    invalidTriangle[0].z = std::numeric_limits<float>::infinity();
    Reject([&] { gpu.TraceTriangle(invalidTriangle, cases); }, "nonfinite vertex");
    invalidTriangle = triangle; invalidTriangle[2] = invalidTriangle[1];
    Reject([&] { gpu.TraceTriangle(invalidTriangle, cases); }, "degenerate triangle");
    const float affineInput[]{-1,0,1};
    Require(gpu.Transform(affineInput, 2, 1) == std::vector<float>({-1,1,3}),
        "Compute pipeline reuse after ray queries failed");
    std::cout << "PASS DXR 6 invalid inputs + compute pipeline reuse\n"
              << "PASS DXR " << compared << " ray comparisons (two query modes per ray)\n";

    const auto beforeBuild = gpu.Statistics();
    auto scene = gpu.BuildTriangleScene(triangle);
    Require(scene.Valid(), "Persistent scene is invalid after build");
    Require(gpu.Statistics().sceneBuildCount == beforeBuild.sceneBuildCount + 1,
        "Persistent scene must be built exactly once");
    const auto built = gpu.Statistics();
    std::uint64_t dispatches = 0;
    std::size_t persistentComparisons = 0;
    std::uint64_t reservedGrowths = 0;
    for (std::size_t count : {513u, 65u, 1u, 0u, 513u}) {
        std::vector<QueryRay> batch(count);
        for (std::size_t i = 0; i < count; ++i) batch[i] = cases[i % cases.size()];
        Verify(triangle, batch, gpu.TraceRays(scene, batch), "persistent batch " + std::to_string(count));
        if (count != 0) ++dispatches;
        persistentComparisons += count;
        const auto stats = gpu.Statistics();
        Require(stats.sceneBuildCount == built.sceneBuildCount, "Tracing rebuilt a persistent scene");
        Require(stats.rayDispatchCount == built.rayDispatchCount + dispatches,
            "Empty batches must not dispatch GPU work");
        if (count == 513 && dispatches == 1) reservedGrowths = stats.batchBufferGrowthCount;
        else Require(stats.batchBufferGrowthCount == reservedGrowths, "Smaller/equal batches reallocated buffers");
    }
    auto moved = std::move(scene);
    Require(!scene.Valid() && moved.Valid(), "Scene move did not transfer ownership");
    Reject([&] { gpu.TraceRays(scene, cases); }, "moved-from scene");
    Verify(triangle, cases, gpu.TraceRays(moved, cases), "moved persistent scene");
    auto shiftedScene = gpu.BuildTriangleScene(shifted);
    Verify(shifted, cases, gpu.TraceRays(shiftedScene, cases), "second persistent scene");
    Verify(triangle, cases, gpu.TraceRays(moved, cases), "original scene after second build");
    Reject([&] { gpu.BuildTriangleScene(invalidTriangle); }, "invalid persistent build");
    invalidRay = cases[0]; invalidRay.direction = {0,0,0};
    Reject([&] { gpu.TraceRays(moved, std::span{&invalidRay, 1}); }, "invalid persistent ray");
    Verify(triangle, cases, gpu.TraceRays(moved, cases), "scene after rejected inputs");
    shiftedScene = std::move(moved);
    Require(!moved.Valid() && shiftedScene.Valid(), "Scene move assignment failed");
    Verify(triangle, cases, gpu.TraceRays(shiftedScene, cases), "move-assigned scene");
    {
        GpuCompute otherContext;
        Reject([&] { otherContext.TraceRays(shiftedScene, cases); }, "foreign device context");
    }
    // A scene owns its context through shared lifetime, never a dangling device pointer.
    std::optional<GpuTriangleScene> survivor;
    {
        GpuCompute shortLivedContext;
        survivor.emplace(shortLivedContext.BuildTriangleScene(triangle));
    }
    Require(survivor->Valid(), "Scene lost resources when its facade was destroyed");
    survivor.reset();
    persistentComparisons += cases.size() * 5;
    std::cout << "PASS persistent scene: " << persistentComparisons << " ray comparisons\n"
              << "PASS build once / repeated trace / batch buffer reuse / empty batch\n"
              << "PASS scene moves / interleaved scenes / rejected inputs / context ownership\n";
}
