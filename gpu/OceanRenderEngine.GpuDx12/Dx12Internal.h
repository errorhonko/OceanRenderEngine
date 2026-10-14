#pragma once
// Backend infrastructure only. Application code should include GpuCompute.h.
#include "GpuCompute.h"
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <d3d12.h>
#include <d3d12sdklayers.h>
#include <wrl/client.h>
#include <span>

namespace ocean::gpu::detail {
using Microsoft::WRL::ComPtr;
void Check(HRESULT hr, const char* operation);

class EventHandle {
public:
    EventHandle();
    ~EventHandle();
    EventHandle(const EventHandle&) = delete;
    EventHandle& operator=(const EventHandle&) = delete;
    HANDLE Get() const noexcept { return value; }
private:
    HANDLE value;
};

// A single synchronous queue. Each SubmitAndWait completes before any reset/release.
// Instances are not thread-safe; arbitrary asynchronous work is intentionally not exposed.
class Dx12Context {
public:
    explicit Dx12Context(bool useWarp);
    ID3D12GraphicsCommandList* Begin(ID3D12PipelineState* pipeline = nullptr);
    void SubmitAndWait();
    void RequireRayQueries() const;
    DeviceInfo info;
    ComPtr<ID3D12Device> device;
    GpuStatistics statistics;
private:
    void CheckDebugMessages();
    ComPtr<ID3D12CommandQueue> queue;
    ComPtr<ID3D12CommandAllocator> allocator;
    ComPtr<ID3D12GraphicsCommandList> commands;
    ComPtr<ID3D12Fence> fence;
    ComPtr<ID3D12InfoQueue> messages;
    EventHandle completed;
    UINT64 fenceValue = 0;
};

// Move-only committed buffer. Heap/state/size travel with its owning resource.
class Dx12Buffer {
public:
    Dx12Buffer() = default;
    Dx12Buffer(Dx12Buffer&&) noexcept = default;
    Dx12Buffer& operator=(Dx12Buffer&&) noexcept = default;
    Dx12Buffer(const Dx12Buffer&) = delete;
    Dx12Buffer& operator=(const Dx12Buffer&) = delete;
    static Dx12Buffer Create(ID3D12Device* device, UINT64 bytes,
        D3D12_HEAP_TYPE heap, D3D12_RESOURCE_STATES state,
        D3D12_RESOURCE_FLAGS flags = D3D12_RESOURCE_FLAG_NONE);
    static Dx12Buffer Upload(ID3D12Device* device, std::span<const std::byte> data);
    void Write(std::span<const std::byte> data);
    // Caller must have completed the queue before reading a READBACK heap.
    void Read(std::span<std::byte> destination) const;
    void Transition(ID3D12GraphicsCommandList* commands, D3D12_RESOURCE_STATES after);
    void UavBarrier(ID3D12GraphicsCommandList* commands) const;
    void CopyFrom(ID3D12GraphicsCommandList* commands, const Dx12Buffer& source, UINT64 bytes);
    ID3D12Resource* Get() const noexcept { return resource.Get(); }
    D3D12_GPU_VIRTUAL_ADDRESS Address() const noexcept { return resource->GetGPUVirtualAddress(); }
private:
    ComPtr<ID3D12Resource> resource;
    UINT64 byteSize = 0;
    D3D12_HEAP_TYPE heapType = D3D12_HEAP_TYPE_DEFAULT;
    D3D12_RESOURCE_STATES state = D3D12_RESOURCE_STATE_COMMON;
};

struct ComputePipeline {
    ComPtr<ID3D12RootSignature> root;
    ComPtr<ID3D12PipelineState> state;
    static ComputePipeline Create(ID3D12Device* device,
        std::span<const D3D12_ROOT_PARAMETER> parameters, std::span<const unsigned char> shader);
    void Bind(ID3D12GraphicsCommandList* commands) const;
};

// Reusable input/upload/output/readback storage; grow only when a batch needs more bytes.
struct TransferBatch {
    Dx12Buffer upload, input, output, readback;
    bool Reserve(ID3D12Device* device, UINT64 inputBytes, UINT64 outputBytes);
    void UploadInput(ID3D12GraphicsCommandList* commands, std::span<const std::byte> data);
    void CopyOutput(ID3D12GraphicsCommandList* commands, UINT64 bytes);
private:
    UINT64 inputCapacity = 0, outputCapacity = 0;
};
}
