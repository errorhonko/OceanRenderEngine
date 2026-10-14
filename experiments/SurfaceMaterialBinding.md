# 逐面表面材质与雷达散射绑定

2026-10-14。此阶段实现手动逐三角形绑定；没有实现 OBJ/MTL 自动材质转换，
没有替换 Toyblocks/城市场景的统一散射模型，没有提交或推送 Git。

## 分层与所有权

- `TriangleMesh`：仍只保存共享顶点和三角形索引，不包含材质或光谱。
- `SurfaceMaterial`：两个固定共享指针，分别指向现有 `Material` 和独立 `LidarScatteringModel`。
  只用于雷达的表面可以没有渲染材质；绑定为空模型对也是允许的显式状态。
- `SurfaceMaterialBindings`：每个三角形一个非空组合指针，顺序对应 `Triangles()`，
  不是 OBJ 材质编号。长度必须精确匹配；缺失组合指针拒绝。相同材质可重复使用同一组合。
- `MeshTriangle`：从组合取得两种模型指针的值快照，模型/曲线本体不复制。
- `HitRecord`：保留 `material`，新增 `lidarScattering`；`SetSurfaceMaterials` 同时设置或清空两者。
  内置 MeshTriangle、Sphere、Triangle 的成功求交都设置这两个字段，避免复用交点时残留绑定。
  新增自定义 Hittable 求交实现也应遵循此约定。

组合使用 const 成员防止通过可变 shared_ptr 别名换绑。渲染 `Material` 仍使用现有
非 const 接口，其内部缓存/可变状态及线程安全不由此组合解决。

## 手动使用

```cpp
#include "scene/SceneBuilder.h"
#include "reflection/lidar/LambertianLidarScattering.h"
#include "reflection/lidar/SurfaceLidarScattering.h"

auto low = std::make_shared<const SurfaceMaterial>(SurfaceMaterial{
    nullptr, std::make_shared<LambertianLidarScattering>(0.2)});
auto high = std::make_shared<const SurfaceMaterial>(SurfaceMaterial{
    nullptr, std::make_shared<LambertianLidarScattering>(0.6)});

// 示例 importedMesh 恰有两个三角形；真实网格应逐面填满。
SurfaceMaterialBindings perFace{low, high};
SceneBuilder builder;
builder.AddMeshWithSurfaceMaterials(importedMesh, perFace, pose);
auto scene = builder.Build();
SurfaceLidarScattering scattering; // 严格：命中缺少雷达模型的有效反射面会报错。
LidarIntegrator integrator(*scene, emitter, receiver, samplesPerPulse, scattering);
auto pulse = integrator.SimulatePulse(sampler);
```

`AddMeshWithSurfaceMaterials` 使用具名接口，避免与旧 `AddMesh(mesh, pose, nullptr)` 歧义。
先完整验证并准备临时对象，再加入 builder；失败不增加对象。
builder 复制绑定向量并按原有逻辑复制/放置几何；scene 与命中的共享指针持有模型，
不依赖原向量、导入器、源网格或 builder 的生命周期。
与以前一样，integrator 引用的 scene 和分派器必须比 integrator 活得久。

底层也支持 `CreateMeshTriangles(mesh, perFace)`、`CreateMeshHittables(mesh, perFace)`
和 `TriangleMeshAggregate(mesh, perFace, leafCapacity)`。
旧的统一渲染材质构造/工厂/scene builder 调用保持不变。

## 分派与错误策略

`SurfaceLidarScattering::Evaluate` 优先使用 `hit.lidarScattering`，只在缺失绑定时使用
构造函数显式传入的默认模型。默认没有 fallback，因此严格报错。
禁止直接把 SurfaceLidarScattering 绑定到面上或作为另一个分派器的 fallback，以避免递归。

已有黑色模型仍是有效绑定，不回退；频带外、非法波长、其他模型异常原样传播。
无效 BRDF 仍由 `EstimateSingleReturn` 检查。不修改回波能量公式、余弦、1/N、波形或测距。
最近的几何交点缺少模型时不能跳过它再找后面的有材质交点。

保留 `LidarIntegrator` 旧接口：如果继续传入固定 Lambert/海面模型，仍全局使用该模型；
只有传入 SurfaceLidarScattering 才启用逐面分派，这是明确的可选模式。

## 验证

新增 `SurfaceMaterialAcceptance.cpp` 并注册到 CPU 验收入口。
同一张四顶点网格的两个三角形共享顶点，法线均向上，测量距离均为 15 m，
模型反射率分别 0.2、0.6。测试求交得到不同渲染/雷达模型，解析回波能量比为 3。
另覆盖模型共享/生命周期、绑定向量和几何快照、错误输入、旧形状清空字段、
最近交点、显式 fallback、递归保护、光谱分派、波形能量守恒、1/N 和传播时间。
实验不依赖外部资产或网络，不能作为真实城市材质的物理精度验证。

CPU 新增 50 项检查通过；Release / Debug 全量各 2154 项通过，零失败、零跳过。
VS 项目与筛选器注册已检查。此处只报告 CPU 验收，没有构建或验证 GPU 分支。

## 后续自动转换进度

导入时为每份 OBJ 保留 `reader.GetMaterials()`，把每个源材质解析成共享组合，
再用 `ImportedMesh.materialIds` 生成已解析的 perFace 向量。
本地编号只在所属资产内有效；-1 的默认策略必须显式指定，不能混用不同 OBJ 的编号。
不要用 RGB Kd 推断近红外反射率，或仅凭名字把某份实测曲线当成完整角度 BRDF。

2026-10-14 已实现这一步，见 [OBJ/MTL 自动材质绑定](ObjMaterialBinding.md)。
保留默认严格策略与显式模型覆盖；城市预设增加可选材质加载入口，旧行为不变。

参考 PBRT v4 primitive / material / SurfaceInteraction 分工：
https://www.pbr-book.org/4ed/Primitives_and_Intersection_Acceleration/Primitive_Interface_and_Geometric_Primitives
本项目在现有 MeshTriangle 面元层适配，没有新增完整 GeometricPrimitive 层。
