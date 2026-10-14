cbuffer Parameters : register(b0)
{
    uint elementCount;
    float scale;
    float bias;
};
StructuredBuffer<float> inputValues : register(t0);
RWStructuredBuffer<float> outputValues : register(u0);

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID)
{
    // Partial final groups must not read/write outside the logical array.
    if (id.x < elementCount)
        outputValues[id.x] = inputValues[id.x] * scale + bias;
}
