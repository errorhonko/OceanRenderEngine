#include "GpuCompute.h"
#include "Dx12Internal.h"
#include "AffineShader.h"
#include "TriangleQueryShader.h"
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace ocean::gpu {
using namespace detail;
namespace {
constexpr std::size_t MaxBatchElements = 65535u * 64u;
bool Finite(Float3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}
void ValidateTriangle(const std::array<Float3, 3>& vertices) {
    for (auto v : vertices)
        if (!Finite(v)) throw std::invalid_argument("Triangle vertices must be finite.");
    const double e1[3]{double(vertices[1].x)-vertices[0].x,
        double(vertices[1].y)-vertices[0].y, double(vertices[1].z)-vertices[0].z};
    const double e2[3]{double(vertices[2].x)-vertices[0].x,
        double(vertices[2].y)-vertices[0].y, double(vertices[2].z)-vertices[0].z};
    const double nx = e1[1]*e2[2]-e1[2]*e2[1];
    const double ny = e1[2]*e2[0]-e1[0]*e2[2];
    const double nz = e1[0]*e2[1]-e1[1]*e2[0];
    if (nx == 0 && ny == 0 && nz == 0)
        throw std::invalid_argument("Triangle must be nondegenerate.");
}
void ValidateRays(std::span<const QueryRay> rays) {
    if (rays.size() > MaxBatchElements)
        throw std::length_error("Ray batch exceeds the one-dimensional dispatch limit.");
    for (const auto& ray : rays)
        if (!Finite(ray.origin) || !Finite(ray.direction) ||
            (ray.direction.x == 0 && ray.direction.y == 0 && ray.direction.z == 0) ||
            !std::isfinite(ray.tMin) || !std::isfinite(ray.tMax) ||
            ray.tMin < 0 || ray.tMax <= ray.tMin)
            throw std::invalid_argument("Ray requires finite data, nonzero direction, and 0 <= tMin < tMax.");
}
UINT DispatchGroups(std::size_t count) { return (static_cast<UINT>(count) + 63) / 64; }
UINT64 AlignedAsSize(UINT64 bytes) {
    if (bytes == 0) throw std::runtime_error("Acceleration structure prebuild returned zero size.");
    constexpr UINT64 alignment = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BYTE_ALIGNMENT;
    return (bytes + alignment - 1) & ~(alignment - 1);
}
Dx12Buffer AsBuffer(ID3D12Device* device, UINT64 bytes, D3D12_RESOURCE_STATES state) {
    return Dx12Buffer::Create(device, AlignedAsSize(bytes), D3D12_HEAP_TYPE_DEFAULT,
        state, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
}
static_assert(sizeof(Float3) == 12 && sizeof(QueryRay) == 32);
static_assert(offsetof(QueryRay, tMin) == 12 && offsetof(QueryRay, direction) == 16
    && offsetof(QueryRay, tMax) == 28);
static_assert(sizeof(TriangleQueryResult) == 24 && offsetof(TriangleQueryResult, rayT) == 8
    && offsetof(TriangleQueryResult, occluded) == 20);
}

struct GpuTriangleScene::Impl {
    // Declared first, so the context is released after its GPU resources.
    std::shared_ptr<Dx12Context> context;
    Dx12Buffer vertices, blas, tlas;
};
GpuTriangleScene::GpuTriangleScene(std::unique_ptr<Impl> implementation) : impl(std::move(implementation)) {}
GpuTriangleScene::~GpuTriangleScene() = default;
GpuTriangleScene::GpuTriangleScene(GpuTriangleScene&&) noexcept = default;
GpuTriangleScene& GpuTriangleScene::operator=(GpuTriangleScene&&) noexcept = default;
bool GpuTriangleScene::Valid() const noexcept { return impl != nullptr; }

struct GpuCompute::Impl {
    std::shared_ptr<Dx12Context> context;
    ComputePipeline affinePipeline, rayPipeline;
    TransferBatch affineBatch, rayBatch;

    explicit Impl(bool useWarp) : context(std::make_shared<Dx12Context>(useWarp)) {
        D3D12_ROOT_PARAMETER parameters[3]{};
        parameters[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
        parameters[0].Constants.Num32BitValues = 3;
        parameters[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
        parameters[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
        affinePipeline = ComputePipeline::Create(context->device.Get(), parameters, g_affineShader);
    }
    void InitializeRayPipeline() {
        if (rayPipeline.state) return;
        context->RequireRayQueries();
        D3D12_ROOT_PARAMETER parameters[4]{};
        parameters[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
        parameters[0].Constants.Num32BitValues = 1;
        parameters[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
        parameters[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
        parameters[2].Descriptor.ShaderRegister = 1;
        parameters[3].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
        rayPipeline = ComputePipeline::Create(context->device.Get(), parameters, g_triangleQueryShader);
    }
    std::unique_ptr<GpuTriangleScene::Impl> BuildTriangleScene(const std::array<Float3, 3>& vertices) {
        ValidateTriangle(vertices);
        InitializeRayPipeline();
        ComPtr<ID3D12Device5> rayDevice;
        Check(context->device.As(&rayDevice), "Query ID3D12Device5");
        auto scene = std::make_unique<GpuTriangleScene::Impl>();
        scene->context = context;
        const auto vertexData = std::as_bytes(std::span{vertices});
        auto vertexUpload = Dx12Buffer::Upload(context->device.Get(), vertexData);
        scene->vertices = Dx12Buffer::Create(context->device.Get(), vertexData.size_bytes(),
            D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_STATE_COPY_DEST);

        D3D12_RAYTRACING_GEOMETRY_DESC geometry{};
        geometry.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
        geometry.Flags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;
        geometry.Triangles.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT;
        geometry.Triangles.VertexCount = 3;
        geometry.Triangles.VertexBuffer.StartAddress = scene->vertices.Address();
        geometry.Triangles.VertexBuffer.StrideInBytes = sizeof(Float3);
        geometry.Triangles.IndexFormat = DXGI_FORMAT_UNKNOWN;
        D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS blasInputs{};
        blasInputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
        blasInputs.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;
        blasInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
        blasInputs.NumDescs = 1;
        blasInputs.pGeometryDescs = &geometry;
        D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO blasSizes{};
        rayDevice->GetRaytracingAccelerationStructurePrebuildInfo(&blasInputs, &blasSizes);
        scene->blas = AsBuffer(context->device.Get(), blasSizes.ResultDataMaxSizeInBytes,
            D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE);
        auto blasScratch = AsBuffer(context->device.Get(), blasSizes.ScratchDataSizeInBytes,
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        D3D12_RAYTRACING_INSTANCE_DESC instance{};
        instance.Transform[0][0] = instance.Transform[1][1] = instance.Transform[2][2] = 1;
        instance.InstanceMask = 0xff;
        instance.AccelerationStructure = scene->blas.Address();
        auto instances = Dx12Buffer::Upload(context->device.Get(), std::as_bytes(std::span{&instance, 1}));
        D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS tlasInputs{};
        tlasInputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
        tlasInputs.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;
        tlasInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
        tlasInputs.NumDescs = 1;
        tlasInputs.InstanceDescs = instances.Address();
        D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO tlasSizes{};
        rayDevice->GetRaytracingAccelerationStructurePrebuildInfo(&tlasInputs, &tlasSizes);
        scene->tlas = AsBuffer(context->device.Get(), tlasSizes.ResultDataMaxSizeInBytes,
            D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE);
        auto tlasScratch = AsBuffer(context->device.Get(), tlasSizes.ScratchDataSizeInBytes,
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        auto* commands = context->Begin();
        ComPtr<ID3D12GraphicsCommandList4> rayCommands;
        Check(commands->QueryInterface(IID_PPV_ARGS(&rayCommands)), "Query ID3D12GraphicsCommandList4");
        scene->vertices.CopyFrom(commands, vertexUpload, vertexData.size_bytes());
        scene->vertices.Transition(commands, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
        D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC build{};
        build.Inputs = blasInputs;
        build.DestAccelerationStructureData = scene->blas.Address();
        build.ScratchAccelerationStructureData = blasScratch.Address();
        rayCommands->BuildRaytracingAccelerationStructure(&build, 0, nullptr);
        scene->blas.UavBarrier(commands);
        build.Inputs = tlasInputs;
        build.DestAccelerationStructureData = scene->tlas.Address();
        build.ScratchAccelerationStructureData = tlasScratch.Address();
        rayCommands->BuildRaytracingAccelerationStructure(&build, 0, nullptr);
        scene->tlas.UavBarrier(commands);
        context->SubmitAndWait();
        ++context->statistics.sceneBuildCount;
        return scene;
    }
    std::vector<TriangleQueryResult> TraceRays(const GpuTriangleScene& scene,
        std::span<const QueryRay> rays) {
        if (!scene.impl || scene.impl->context.get() != context.get())
            throw std::invalid_argument("Scene is moved-from or belongs to another GPU context.");
        ValidateRays(rays);
        if (rays.empty()) return {};
        std::vector<TriangleQueryResult> results(rays.size());
        const auto rayBytes = std::as_bytes(rays);
        const auto resultBytes = std::as_writable_bytes(std::span{results});
        if (rayBatch.Reserve(context->device.Get(), rayBytes.size_bytes(), resultBytes.size_bytes()))
            ++context->statistics.batchBufferGrowthCount;
        auto* commands = context->Begin(rayPipeline.state.Get());
        rayBatch.UploadInput(commands, rayBytes);
        rayPipeline.Bind(commands);
        commands->SetComputeRoot32BitConstant(0, static_cast<UINT>(rays.size()), 0);
        commands->SetComputeRootShaderResourceView(1, scene.impl->tlas.Address());
        commands->SetComputeRootShaderResourceView(2, rayBatch.input.Address());
        commands->SetComputeRootUnorderedAccessView(3, rayBatch.output.Address());
        commands->Dispatch(DispatchGroups(rays.size()), 1, 1);
        rayBatch.CopyOutput(commands, resultBytes.size_bytes());
        context->SubmitAndWait();
        rayBatch.readback.Read(resultBytes);
        ++context->statistics.rayDispatchCount;
        return results;
    }
    std::vector<float> Transform(std::span<const float> values, float scale, float bias) {
        if (!std::isfinite(scale) || !std::isfinite(bias))
            throw std::invalid_argument("Scale and bias must be finite.");
        if (values.size() > MaxBatchElements)
            throw std::length_error("Input exceeds the one-dimensional dispatch limit.");
        for (float value : values)
            if (!std::isfinite(value)) throw std::invalid_argument("Input must be finite.");
        if (values.empty()) return {};
        std::vector<float> results(values.size());
        const auto inputBytes = std::as_bytes(values);
        const auto resultBytes = std::as_writable_bytes(std::span{results});
        if (affineBatch.Reserve(context->device.Get(), inputBytes.size_bytes(), resultBytes.size_bytes()))
            ++context->statistics.batchBufferGrowthCount;
        auto* commands = context->Begin(affinePipeline.state.Get());
        affineBatch.UploadInput(commands, inputBytes);
        affinePipeline.Bind(commands);
        struct Constants { UINT count; float scale; float bias; };
        static_assert(sizeof(Constants) == 12);
        const Constants constants{static_cast<UINT>(values.size()), scale, bias};
        commands->SetComputeRoot32BitConstants(0, 3, &constants, 0);
        commands->SetComputeRootShaderResourceView(1, affineBatch.input.Address());
        commands->SetComputeRootUnorderedAccessView(2, affineBatch.output.Address());
        commands->Dispatch(DispatchGroups(values.size()), 1, 1);
        affineBatch.CopyOutput(commands, resultBytes.size_bytes());
        context->SubmitAndWait();
        affineBatch.readback.Read(resultBytes);
        return results;
    }
};
GpuCompute::GpuCompute(bool useWarp) : impl(std::make_unique<Impl>(useWarp)) {}
GpuCompute::~GpuCompute() = default;
const DeviceInfo& GpuCompute::Info() const noexcept { return impl->context->info; }
GpuStatistics GpuCompute::Statistics() const noexcept { return impl->context->statistics; }
std::vector<float> GpuCompute::Transform(std::span<const float> input, float scale, float bias) {
    return impl->Transform(input, scale, bias);
}
GpuTriangleScene GpuCompute::BuildTriangleScene(const std::array<Float3, 3>& vertices) {
    return GpuTriangleScene(impl->BuildTriangleScene(vertices));
}
std::vector<TriangleQueryResult> GpuCompute::TraceRays(
    const GpuTriangleScene& scene, std::span<const QueryRay> rays) {
    return impl->TraceRays(scene, rays);
}
std::vector<TriangleQueryResult> GpuCompute::TraceTriangle(
    const std::array<Float3, 3>& vertices, std::span<const QueryRay> rays) {
    // Validate before allocating a scene, preserving the old invalid-input behavior.
    ValidateTriangle(vertices);
    ValidateRays(rays);
    if (rays.empty()) { impl->context->RequireRayQueries(); return {}; }
    auto scene = BuildTriangleScene(vertices);
    return TraceRays(scene, rays);
}
}
