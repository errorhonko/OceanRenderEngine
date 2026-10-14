#include "Dx12Internal.h"
#include <dxgi1_6.h>
#include <sstream>
#include <stdexcept>
namespace ocean::gpu::detail {
void Check(HRESULT hr, const char* operation) {
    if (FAILED(hr)) {
        std::ostringstream message;
        message << operation << " failed (HRESULT 0x" << std::hex
                << static_cast<unsigned long>(hr) << ")";
        throw std::runtime_error(message.str());
    }
}
EventHandle::EventHandle() : value(CreateEventW(nullptr, FALSE, FALSE, nullptr)) {
    if (!value) throw std::runtime_error("CreateEventW failed");
}
EventHandle::~EventHandle() { CloseHandle(value); }
Dx12Context::Dx12Context(bool useWarp) {
        UINT factoryFlags = 0;
#ifdef _DEBUG
        ComPtr<ID3D12Debug> debug;
        if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&debug)))) {
            debug->EnableDebugLayer();
            info.debugLayerEnabled = true;
            factoryFlags = DXGI_CREATE_FACTORY_DEBUG;
        }
#endif
        ComPtr<IDXGIFactory6> factory;
        Check(CreateDXGIFactory2(factoryFlags, IID_PPV_ARGS(&factory)), "CreateDXGIFactory2");
        ComPtr<IDXGIAdapter1> adapter;
        if (useWarp) {
            Check(factory->EnumWarpAdapter(IID_PPV_ARGS(&adapter)), "EnumWarpAdapter");
            Check(D3D12CreateDevice(adapter.Get(), D3D_FEATURE_LEVEL_12_0,
                IID_PPV_ARGS(&device)), "Create WARP device");
        } else {
            for (UINT index = 0; ; ++index) {
                ComPtr<IDXGIAdapter1> candidate;
                const HRESULT hr = factory->EnumAdapterByGpuPreference(index,
                    DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE, IID_PPV_ARGS(&candidate));
                if (hr == DXGI_ERROR_NOT_FOUND) break;
                Check(hr, "EnumAdapterByGpuPreference");
                DXGI_ADAPTER_DESC1 desc{};
                Check(candidate->GetDesc1(&desc), "GetDesc1");
                if (desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) continue;
                if (SUCCEEDED(D3D12CreateDevice(candidate.Get(), D3D_FEATURE_LEVEL_12_0,
                    IID_PPV_ARGS(&device)))) {
                    adapter = candidate;
                    break;
                }
            }
            if (!device) throw std::runtime_error(
                "No hardware D3D12 adapter available. Check the driver/session. "
                "Use --warp only for software infrastructure checks.");
        }
        DXGI_ADAPTER_DESC1 desc{};
        Check(adapter->GetDesc1(&desc), "GetDesc1");
        info.adapterName = desc.Description;
        info.dedicatedVideoMemoryBytes = desc.DedicatedVideoMemory;
        info.softwareAdapter = (desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) != 0;
        D3D12_FEATURE_DATA_D3D12_OPTIONS5 options{};
        if (SUCCEEDED(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS5,
            &options, sizeof(options)))) {
            if (options.RaytracingTier >= D3D12_RAYTRACING_TIER_1_1) info.raytracingTier = 11;
            else if (options.RaytracingTier >= D3D12_RAYTRACING_TIER_1_0) info.raytracingTier = 10;
        }
        D3D12_FEATURE_DATA_SHADER_MODEL shaderModel{D3D_SHADER_MODEL_6_0};
        Check(device->CheckFeatureSupport(D3D12_FEATURE_SHADER_MODEL,
            &shaderModel, sizeof(shaderModel)), "Check shader model");
        if (shaderModel.HighestShaderModel < D3D_SHADER_MODEL_6_0)
            throw std::runtime_error("Shader Model 6.0 is required by the compute checkpoint.");
        if (info.debugLayerEnabled) device.As(&messages);

        D3D12_COMMAND_QUEUE_DESC queueDesc{};
        queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
        Check(device->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(&queue)), "CreateCommandQueue");
        Check(device->CreateCommandAllocator(queueDesc.Type,
            IID_PPV_ARGS(&allocator)), "CreateCommandAllocator");
        Check(device->CreateCommandList(0, queueDesc.Type, allocator.Get(), nullptr,
            IID_PPV_ARGS(&commands)), "CreateCommandList");
        Check(commands->Close(), "Close initial command list");
        Check(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence)), "CreateFence");

}
void Dx12Context::SubmitAndWait() {
        Check(commands->Close(), "Close command list");
        ID3D12CommandList* lists[]{commands.Get()};
        queue->ExecuteCommandLists(1, lists);
        Check(queue->Signal(fence.Get(), ++fenceValue), "Signal fence");
        if (fence->GetCompletedValue() < fenceValue) {
            Check(fence->SetEventOnCompletion(fenceValue, completed.Get()), "SetEventOnCompletion");
            if (WaitForSingleObject(completed.Get(), INFINITE) != WAIT_OBJECT_0)
                throw std::runtime_error("GPU completion wait failed");
        }
        Check(device->GetDeviceRemovedReason(), "GPU device status");
        CheckDebugMessages();
    }

void Dx12Context::CheckDebugMessages() {
        if (!messages) return;
        const auto count = messages->GetNumStoredMessagesAllowedByRetrievalFilter();
        for (UINT64 i = 0; i < count; ++i) {
            SIZE_T size = 0;
            Check(messages->GetMessage(i, nullptr, &size), "Get debug message size");
            std::vector<unsigned char> storage(size);
            auto* message = reinterpret_cast<D3D12_MESSAGE*>(storage.data());
            Check(messages->GetMessage(i, message, &size), "Get debug message");
            if (message->Severity == D3D12_MESSAGE_SEVERITY_ERROR ||
                message->Severity == D3D12_MESSAGE_SEVERITY_CORRUPTION)
                throw std::runtime_error(std::string("D3D12 validation: ") + message->pDescription);
        }
        messages->ClearStoredMessages();
    }

ID3D12GraphicsCommandList* Dx12Context::Begin(ID3D12PipelineState* pipeline) {
    Check(allocator->Reset(), "Reset command allocator");
    Check(commands->Reset(allocator.Get(), pipeline), "Reset command list");
    return commands.Get();
}
void Dx12Context::RequireRayQueries() const {
    if (info.raytracingTier < 11)
        throw std::runtime_error("Ray queries require DXR 1.1; no software fallback is used.");
    D3D12_FEATURE_DATA_SHADER_MODEL shaderModel{D3D_SHADER_MODEL_6_5};
    Check(device->CheckFeatureSupport(D3D12_FEATURE_SHADER_MODEL,
        &shaderModel, sizeof(shaderModel)), "Check RayQuery shader model");
    if (shaderModel.HighestShaderModel < D3D_SHADER_MODEL_6_5)
        throw std::runtime_error("Inline ray queries require Shader Model 6.5.");
}
}