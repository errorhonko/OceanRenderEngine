cbuffer Parameters : register(b0) { uint rayCount; };
RaytracingAccelerationStructure scene : register(t0);
struct QueryRay {
    float3 origin;
    float tMin;
    float3 direction;
    float tMax;
};
struct QueryResult {
    uint hit;
    uint primitiveId;
    float rayT;
    float baryU;
    float baryV;
    uint occluded;
};
StructuredBuffer<QueryRay> rays : register(t1);
RWStructuredBuffer<QueryResult> results : register(u0);

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID)
{
    if (id.x >= rayCount) return;
    QueryRay input = rays[id.x];
    RayDesc ray;
    ray.Origin = input.origin;
    ray.Direction = input.direction;
    ray.TMin = input.tMin;
    ray.TMax = input.tMax;
    QueryResult result = (QueryResult)0;
    result.primitiveId = 0xffffffff;

    // Opaque built-in triangles are automatically committed; no candidate callbacks.
    // No face culling: geometric front-side policy belongs to the LiDAR model later.
    RayQuery<RAY_FLAG_FORCE_OPAQUE> closest;
    closest.TraceRayInline(scene, 0, 0xff, ray);
    while (closest.Proceed()) { }
    if (closest.CommittedStatus() == COMMITTED_TRIANGLE_HIT) {
        result.hit = 1;
        result.primitiveId = closest.CommittedPrimitiveIndex();
        result.rayT = closest.CommittedRayT();
        float2 bary = closest.CommittedTriangleBarycentrics();
        result.baryU = bary.x;
        result.baryV = bary.y;
    }

    // Independent boolean query: any blocker in the finite segment is sufficient.
    RayQuery<RAY_FLAG_FORCE_OPAQUE | RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH> shadow;
    shadow.TraceRayInline(scene, 0, 0xff, ray);
    while (shadow.Proceed()) { }
    result.occluded = shadow.CommittedStatus() == COMMITTED_TRIANGLE_HIT ? 1 : 0;
    results[id.x] = result;
}
