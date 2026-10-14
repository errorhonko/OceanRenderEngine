# 可复用 DX12 GPU 基础层

本模块位于原 OceanRenderEngine 解决方案内：OceanRenderEngine.GpuDx12 是静态库，OceanRenderEngine.GpuTests 是对照检查程序。当前支持普通计算和一个不透明、双面、单位实例变换的持久三角形场景。

借鉴 PBRT v4 第 15 章的 CPU/GPU 分工和批量任务思想。当前使用单队列同步接口，没有实现完整 wavefront 积分器。

## 直接调用

应用只需包含 GpuCompute.h、引用静态库，并链接 d3d12.lib、dxgi.lib。GPU 测试项目已经配置这些依赖，原 CPU 渲染器尚未添加调用。

```cpp
#include "GpuCompute.h"
#include <array>
#include <vector>

using namespace ocean::gpu;
GpuCompute gpu; // 一个设备与执行上下文，建议长期保留
const std::array<Float3, 3> vertices{{{-1,-1,0}, {1,-1,0}, {0,1,0}}};
auto scene = gpu.BuildTriangleScene(vertices); // 上传、构建一次，完成后返回

std::vector<QueryRay> rays{
    {{0,0,2}, 0.0001f, {0,0,-1}, 10.0f}
};
auto first = gpu.TraceRays(scene, rays);
rays[0].tMax = 1.5f;
auto second = gpu.TraceRays(scene, rays); // 复用同一 BLAS/TLAS，无重新构建

const auto statistics = gpu.Statistics();
// sceneBuildCount=1，rayDispatchCount=2；计数不是性能计时
const std::array<float, 3> values{-1, 0, 1};
auto transformed = gpu.Transform(values, 2.0f, 1.0f);
```

| 公共接口 | 职责 |
| --- | --- |
| GpuCompute(bool useWarp=false) | 创建硬件设备、队列和计算管线；显式 useWarp 才使用软件 |
| Info() | 适配器、显存、DXR 等级、调试层状态 |
| BuildTriangleScene(vertices) | 检查输入、上传顶点、构建 BLAS/TLAS，返回持久场景 |
| TraceRays(scene, rays) | 批量最近交点与有限区间遮挡查询，完成读回后返回 |
| Transform(values, scale, bias) | 调用仿射计算内核，作为普通 compute 示例 |
| Statistics() | 成功场景构建数、非空射线 dispatch 数、批量缓冲区扩容事件数 |
| TraceTriangle(vertices, rays) | 保留旧测试便利入口，每次创建临时场景；日常重复追踪用新接口 |
| GpuTriangleScene::Valid() | 判断是否持有场景；移动后的源对象无效 |

scene 是 move-only 的 RAII 对象，可以 std::move。场景内部持有创建上下文的共享生命周期，避免 facade 先析构导致设备悬空；TraceRays 只接受当前 GpuCompute 创建的场景，跨上下文传入会抛 invalid_argument。

## 基础设施层

应用通过 GpuCompute.h 调用；新增 DX12 内核时复用内部 Dx12Internal.h，不把 DirectX 类型带入 CPU 数学/物理模型。

| 内部封装 | 复用能力 |
| --- | --- |
| Dx12Context | 设备选择、能力检查、队列/分配器/命令列表、Begin、SubmitAndWait、fence、调试消息 |
| Dx12Buffer | move-only 资源所有权、Create、Upload、Write、Read、CopyFrom、Transition、UavBarrier |
| ComputePipeline | 根签名和计算 PSO 的统一创建与绑定 |
| TransferBatch | UPLOAD/DEFAULT 输入、UAV 输出、READBACK 缓冲区；按需扩容、跟踪状态、重复使用 |
| GpuTriangleScene | 持久顶点、BLAS/TLAS 与上下文所有权 |

设备与同步在 Dx12Context.cpp；缓冲区、传输和计算管线在 Dx12Buffer.cpp；业务组合和三角形场景在 GpuCompute.cpp。已有 shader 在构建时由 DXC 编译并嵌入静态库，不需要运行时寻找文件。

新增计算内核可以按 Transform 的模式：编译 HLSL → ComputePipeline::Create → TransferBatch::Reserve → Begin → UploadInput → Bind/设置参数 → Dispatch → CopyOutput → SubmitAndWait → Read。新资源和 scratch 必须保持到提交完成；低层工具供库内部使用，调用方不需要手动写 barrier/fence。

## 行为与边界

- 同步、单线程调用；每次提交等待完成后再读回/重用资源。不提供异步 ticket、多队列或并发提交。
- 持久场景不因追踪而重建；几何改变时显式构建新场景。暂无原位更新或 refit。
- 两类业务各有独立批量缓存，较小/相同批量复用；缓存只增长，释放 GpuCompute 时回收。
- 每次 build 使用临时上传与独立 scratch，等待完成后回收；持久场景保留顶点和 BLAS/TLAS。
- 接口最多 65535*64 个元素/射线；空批量不 dispatch；拒绝非有限输入、零方向、非法区间及退化三角形。
- rayT 对应 P(t)=origin+t*direction；只有单位方向时才直接是距离。API 不归一化、不自动偏移原点、不施加 LiDAR 正面条件。
- occluded 是“同一条输入射线在给定区间内是否遇到遮挡”，不是自动完成海面交点到接收器的连接；后续需要另外生成连接射线。
- shader 同时执行最近交点和遮挡两种查询；还没有按业务拆分不同查询模式。
- TraceRays 返回 primitiveId、rayT、重心坐标与标志；尚无海面法线、BRDF、回波能量或波形。
- 当前只支持单三角形、双面不透明几何和单位实例变换。未扩展为通用网格/场景图。
- 实际 driver/device 失败应重新创建上下文；未承诺 GPU 丢失后的自动恢复。非法参数会在录制命令前拒绝。

## 环境、运行与验证

Windows、显卡驱动、Visual Studio C++ 桌面开发与 Windows SDK（包含 x64 dxc.exe）。使用 v145 工具集；普通 compute 要求 SM6.0，RayQuery 要求 DXR 1.1/SM6.5，运行时检查。

打开原 OceanRenderEngine.slnx，选择 Debug/x64，生成并启动 OceanRenderEngine.GpuTests，Ctrl+F5。Release/x64 同样可以运行。默认报告实际硬件，不静默降级。显式 --warp 仅运行软件 compute 验证，并 SKIP 光追验收。

Debug 在 Graphics Tools 调试层可用时启用；ERROR/CORRUPTION 使提交检查失败。当前验收含普通 compute、单三角形 CPU double 对照，以及持久场景不重建、批量缓存复用、场景移动/交替/跨上下文拒绝与生命周期检查。详情见 VALIDATION.md。

尚未进行性能计时，不把资源复用验收当作加速倍数。下一阶段可以在这些基础设施上增加多三角形网格与冻结海面，再接 LiDAR。

参考：
- https://pbr-book.org/4ed/Wavefront_Rendering_on_GPUs
- https://microsoft.github.io/DirectX-Specs/d3d/Raytracing.html
- https://learn.microsoft.com/en-us/windows/win32/direct3d12/using-resource-barriers-to-synchronize-resource-states-in-direct3d-12
