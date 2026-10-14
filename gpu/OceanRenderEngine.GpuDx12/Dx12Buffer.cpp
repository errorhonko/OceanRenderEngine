#include "Dx12Internal.h"
#include <cstring>
#include <stdexcept>

namespace ocean::gpu::detail {
Dx12Buffer Dx12Buffer::Create(ID3D12Device* device, UINT64 bytes,
    D3D12_HEAP_TYPE heapType, D3D12_RESOURCE_STATES state, D3D12_RESOURCE_FLAGS flags) {
    if (bytes == 0) throw std::invalid_argument("Cannot allocate a zero-size GPU buffer.");
    D3D12_HEAP_PROPERTIES heap{};
    heap.Type = heapType;
    heap.CreationNodeMask = heap.VisibleNodeMask = 1;
    D3D12_RESOURCE_DESC desc{};
    desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    desc.Width = bytes;
    desc.Height = 1;
    desc.DepthOrArraySize = desc.MipLevels = 1;
    desc.SampleDesc.Count = 1;
    desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    desc.Flags = flags;
    Dx12Buffer result;
    Check(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE,
        &desc, state, nullptr, IID_PPV_ARGS(&result.resource)), "CreateCommittedResource");
    result.byteSize = bytes;
    result.heapType = heapType;
    result.state = state;
    return result;
}
Dx12Buffer Dx12Buffer::Upload(ID3D12Device* device, std::span<const std::byte> data) {
    auto result = Create(device, data.size_bytes(), D3D12_HEAP_TYPE_UPLOAD,
        D3D12_RESOURCE_STATE_GENERIC_READ);
    result.Write(data);
    return result;
}
void Dx12Buffer::Write(std::span<const std::byte> data) {
    if (!resource || heapType != D3D12_HEAP_TYPE_UPLOAD || data.size_bytes() > byteSize)
        throw std::invalid_argument("Upload write requires an owning UPLOAD buffer of sufficient size.");
    void* mapped = nullptr;
    const D3D12_RANGE noRead{0, 0};
    Check(resource->Map(0, &noRead, &mapped), "Map upload buffer");
    if (!data.empty()) std::memcpy(mapped, data.data(), data.size_bytes());
    const D3D12_RANGE written{0, data.size_bytes()};
    resource->Unmap(0, &written);
}
void Dx12Buffer::Read(std::span<std::byte> destination) const {
    if (!resource || heapType != D3D12_HEAP_TYPE_READBACK || destination.size_bytes() > byteSize)
        throw std::invalid_argument("Readback requires an owning READBACK buffer of sufficient size.");
    void* mapped = nullptr;
    const D3D12_RANGE readRange{0, destination.size_bytes()}, noWrite{0, 0};
    Check(resource->Map(0, &readRange, &mapped), "Map readback buffer");
    if (!destination.empty()) std::memcpy(destination.data(), mapped, destination.size_bytes());
    resource->Unmap(0, &noWrite);
}
void Dx12Buffer::Transition(ID3D12GraphicsCommandList* commands, D3D12_RESOURCE_STATES after) {
    if (state == after) return;
    if (!resource || heapType != D3D12_HEAP_TYPE_DEFAULT)
        throw std::invalid_argument("Only owning DEFAULT buffers can change resource state.");
    D3D12_RESOURCE_BARRIER barrier{};
    barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    barrier.Transition.pResource = resource.Get();
    barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    barrier.Transition.StateBefore = state;
    barrier.Transition.StateAfter = after;
    commands->ResourceBarrier(1, &barrier);
    state = after;
}
void Dx12Buffer::UavBarrier(ID3D12GraphicsCommandList* commands) const {
    if (!resource) throw std::invalid_argument("UAV barrier requires an owning buffer.");
    D3D12_RESOURCE_BARRIER barrier{};
    barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
    barrier.UAV.pResource = resource.Get();
    commands->ResourceBarrier(1, &barrier);
}
void Dx12Buffer::CopyFrom(ID3D12GraphicsCommandList* commands,
    const Dx12Buffer& source, UINT64 bytes) {
    if (!resource || !source.resource || bytes == 0 || bytes > byteSize || bytes > source.byteSize ||
        state != D3D12_RESOURCE_STATE_COPY_DEST ||
        !(source.state & D3D12_RESOURCE_STATE_COPY_SOURCE))
        throw std::invalid_argument("Buffer copy size/state is invalid.");
    commands->CopyBufferRegion(resource.Get(), 0, source.resource.Get(), 0, bytes);
}
ComputePipeline ComputePipeline::Create(ID3D12Device* device,
    std::span<const D3D12_ROOT_PARAMETER> parameters, std::span<const unsigned char> shader) {
    D3D12_ROOT_SIGNATURE_DESC desc{};
    desc.NumParameters = static_cast<UINT>(parameters.size());
    desc.pParameters = parameters.data();
    ComPtr<ID3DBlob> serialized, errors;
    Check(D3D12SerializeRootSignature(&desc, D3D_ROOT_SIGNATURE_VERSION_1,
        &serialized, &errors), "Serialize compute root signature");
    ComputePipeline result;
    Check(device->CreateRootSignature(0, serialized->GetBufferPointer(), serialized->GetBufferSize(),
        IID_PPV_ARGS(&result.root)), "Create compute root signature");
    D3D12_COMPUTE_PIPELINE_STATE_DESC pipelineDesc{};
    pipelineDesc.pRootSignature = result.root.Get();
    pipelineDesc.CS = {shader.data(), shader.size_bytes()};
    Check(device->CreateComputePipelineState(&pipelineDesc, IID_PPV_ARGS(&result.state)),
        "Create compute pipeline");
    return result;
}
void ComputePipeline::Bind(ID3D12GraphicsCommandList* commands) const {
    commands->SetPipelineState(state.Get());
    commands->SetComputeRootSignature(root.Get());
}
bool TransferBatch::Reserve(ID3D12Device* device, UINT64 inputBytes, UINT64 outputBytes) {
    if (inputBytes == 0 || outputBytes == 0)
        throw std::invalid_argument("Transfer batch requires nonempty buffers.");
    bool grew = false;
    if (inputBytes > inputCapacity) {
        auto newUpload = Dx12Buffer::Create(device, inputBytes, D3D12_HEAP_TYPE_UPLOAD,
            D3D12_RESOURCE_STATE_GENERIC_READ);
        auto newInput = Dx12Buffer::Create(device, inputBytes, D3D12_HEAP_TYPE_DEFAULT,
            D3D12_RESOURCE_STATE_COPY_DEST);
        upload = std::move(newUpload);
        input = std::move(newInput);
        inputCapacity = inputBytes;
        grew = true;
    }
    if (outputBytes > outputCapacity) {
        auto newOutput = Dx12Buffer::Create(device, outputBytes, D3D12_HEAP_TYPE_DEFAULT,
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
        auto newReadback = Dx12Buffer::Create(device, outputBytes, D3D12_HEAP_TYPE_READBACK,
            D3D12_RESOURCE_STATE_COPY_DEST);
        output = std::move(newOutput);
        readback = std::move(newReadback);
        outputCapacity = outputBytes;
        grew = true;
    }
    return grew;
}
void TransferBatch::UploadInput(ID3D12GraphicsCommandList* commands, std::span<const std::byte> data) {
    upload.Write(data);
    input.Transition(commands, D3D12_RESOURCE_STATE_COPY_DEST);
    input.CopyFrom(commands, upload, data.size_bytes());
    input.Transition(commands, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    output.Transition(commands, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
}
void TransferBatch::CopyOutput(ID3D12GraphicsCommandList* commands, UINT64 bytes) {
    output.Transition(commands, D3D12_RESOURCE_STATE_COPY_SOURCE);
    readback.CopyFrom(commands, output, bytes);
}
}
