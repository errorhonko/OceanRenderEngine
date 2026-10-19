# OBJ/MTL 自动表面材质绑定

2026-10-16 补充：默认行为仍为下文的 Lambert 自动绑定；现已增加显式可选的
[HELIOS 经验角度基线](HeliosEmpiricalBaseline.md)。该模式逐个初始化镜面比例，
复用原光谱/逐面绑定，另用投影响应入口避免重复余弦，不冒充纯 BRDF。
下文 2026-10-14 的测试数量和“未改动回波公式”描述是当时的历史记录。

2026-10-14。在手动逐面绑定上增加三个步骤：解析 tinyobj 材质表、用每个三角形的
本地编号生成已解析绑定、原子地组装多组网格。没有更改 Monte Carlo、BRDF 能量公式、
接收器/ADC/测距器，也没有自动推送或修改外部资产。

## 实现位置

- `ObjMaterialBinding.h`：`ConvertTable` 和 `BindFaces`，以及策略/来源/诊断。
- `ObjSceneAsset.h`：拥有导入几何、原始解码 MTL 表、模型表、逐面绑定和解析警告。
- `SceneBuilder::AddMeshesWithSurfaceMaterials`：完整准备各组，再提交；失败不留下半个资产。
- `HeliosUrbanStaticScene::Load(directory, materials)`：可选自动绑定模式。
  原 `Load(directory)` 保留纯几何行为，原 Toyblocks 固定反射率实验也保持不变。

VS 新头文件位于 `Scenes/Import`。继续使用 `SurfaceLidarScattering` 才启用交点材质分派；
向 integrator 直接传固定模型仍是全场景统一模型模式。

## 转换优先级与单位

1. 调用方按当前资产的材质名提供完整 `SurfaceMaterial` override。
   它替代整份自动转换，不读取被覆盖的反射率、光谱或渲染参数。
2. `helios_reflectance`：HELIOS 的百分数 [0,100]，除以 100 得到无量纲 rho，
   创建原 `LambertianLidarScattering`。存在时优先于 `helios_spectra`；报告被忽略的光谱。
3. `helios_spectra`：从显式指定的谱目录读取命名 ASTER/ECOSTRESS `.txt`，
   创建 `SpectralLambertianLidarScattering`，返回 rho(lambda)/pi。
4. 两者均缺失：只有调用方显式指定 `missingLidarFallback` 才允许继续，并报告 fallback。
   否则报错，不静默采用 HELIOS 默认 50%，不根据 RGB Kd 猜近红外反射率。

`helios_reflectance 20` 对应 rho=0.2；`helios_reflectance 0.2` 对应 rho=0.002，
不是 20%。小于 1 的正百分数另加单位提示，但不自动改数值。
空值、非有限值、越界和多余数值拒绝；支持首尾空白、科学计数、单个前导 + 和行尾 # 注释。
有声明却出错的光谱不会退回默认模型。波长范围仍由光谱曲线严格约束。

谱标识仅接受 ASCII 字母、数字、下划线、连字符，可选小写 `.txt` 后缀。
不接受路径、目录跳转、绝对路径、Windows 设备名。规范化后的文件必须直接位于
配置的谱目录内，不能用符号链接跳到目录外。相同规范文件在一次材质表转换内
共享同一个散射模型，不逐面读取或复制光谱；不同资产表不使用裸编号作为全局缓存键。

## 渲染材质与不支持的特性

默认 `createDiffuseRenderMaterials=true`：把已解码的 RGB Kd 映射到现有 `DiffuseMaterial`。
这是纯漫反射预览基线，不是完整 MTL/PBR/Phong 复现；Ks/Ns/PBR 参数、纹理、
法线/位移图及发光等未实现特性会保留在源表并报告简化，不自动创建面积光。
RGB Kd 的三通道不能代替激光波长处的反射率。
纯雷达使用可设置该选项为 false，此时不创建渲染材质。

d/Tr 解码透明度、map_d 或透射型 illum 默认拒绝自动转换，避免把玻璃/镂空叶片
静默当作实体反射面。只有显式 override 或 `allowOpaqueApproximation=true` 才放行；
后者报告不透明近似。此时几何仍是完全不透明最近表面，不包含透明穿透。

本层接收 tinyobj 已解码表，并验证所使用的数值；不是第二个完整 MTL 语法分析器。
原表包含分类/地面标记等扩展字段，但尚未把它们写到点云语义标签中。

## 最小用法

```cpp
#include "scene/import/ObjSceneAsset.h"
#include "reflection/lidar/SurfaceLidarScattering.h"

ObjMaterialBindingOptions materials;
materials.spectraDirectory = spectraDirectory;
materials.createDiffuseRenderMaterials = false; // 纯雷达，可选。

// 可选：缺少反射率定义时采用明确的实验假设，而不是实测数据。
materials.missingLidarFallback = std::make_shared<LambertianLidarScattering>(0.5);
// 另一个独立策略：OBJ 面没有 usemtl，即编号 -1。
materials.unassignedMaterial = std::make_shared<const SurfaceMaterial>(
    SurfaceMaterial{nullptr, materials.missingLidarFallback});

auto asset = ObjSceneAsset::Read(objPath, geometryOptions, materials);
SceneBuilder builder;
asset.AddToScene(builder, pose);
auto scene = builder.Build();
SurfaceLidarScattering scattering;
LidarIntegrator integrator(*scene, emitter, receiver, samplesPerPulse, scattering);
auto pulse = integrator.SimulatePulse(sampler);
```

不指定两种默认策略就是严格模式。材质编号 -1 使用 `unassignedMaterial`，
不把它偷换成表第 0 项；小于 -1 或超出表大小的编号拒绝。
即使另一个资产有同样的编号，也只能在所属源表里查找。

`asset.MaterialBindings().info` 含来源类型、规范谱文件路径、简化/缺失诊断。
`asset.SourceMaterials()` 保留完整 tinyobj 材质元数据，`ParserWarning()` 保留解析警告。
文件只在导入时读取，scene 和交点指针持有模型；scene 可比 asset/reader 活得更久。
integrator 引用的 scene 和分派器自身仍必须比 integrator 活得久。

## 城市预设

```cpp
auto urban = HeliosUrbanStaticScene::Load(urbanDirectory, materials);
SurfaceLidarScattering scattering;
// 后续扫描驱动传 *urban.scene 和 scattering。
```

城市逐资产材质表独立保存，旋转、缩放、放置和 onGround 操作保留逐面材质编号。
资料中未给反射率的建筑/车辆/植被，需要明确的默认值或 override；这些不能标为实测结果。

## 验证与真实数据注意事项

新增 `ObjMaterialBindingAcceptance.cpp` 和自造 `fixtures/obj_materials`。
覆盖百分数、优先级、缓存、非法输入与谱路径、透明度策略、原始表/来源保存、
局部 ID、缺失策略、对象所有权、批量失败原子性、坐标单位、导入到回波/波形链路。
自造等距离两面在 532 nm 反射率 0.2/0.6，能量比为 3；光谱面在 1550 nm 反射率
变为 0.3，回波能量降到 532 nm 的一半。

新增 74 项检查全部通过；CPU Release / Debug 全量各 2228 项通过，零失败、零跳过。
此次没有构建或验证 GPU 分支。自造 OBJ 夹具已加精确的 Git 忽略例外，避免被 *.obj 漏掉。

本地另用之前下载的固定版 HELIOS 资产与 10 份光谱验证（不作为新测试夹具再发布）：

- Toyblocks groundplane：未定义雷达反射率，本次显式 fallback rho=0.5。
- cube：wood 光谱，rho(532)=0.44718。
- sphere：原 MTL 为 helios_reflectance 0.2，优先于 conifer，rho=0.002。
- cylinder：原 MTL 为 helios_reflectance 0.5，优先于 shingle_red，rho=0.005。
- 城市：仍为 53 个对象、2,445,341 个三角形；表中 1 个光谱条目（地面沥青），
  76 个条目使用本次明确指定的 rho=0.5 默认模型，1 个资产没有 MTL 表。
- 9 个向下求交探针均取得雷达模型；地面探针 rho(532)=0.170136。
  这是导入/绑定验证，不是完整城市测距扫描、真实材质精度验证或性能基准。

## 参考与刻意简化

- [HELIOS++ 官方材料/强度文档](https://heliospp.readthedocs.io/en/alpha-dev/intensity_fwf.html)：
  mtllib/usemtl、反射率百分数与光谱命名。
- [固定版 SpectralLibrary.cpp](https://github.com/3dgeo-heidelberg/helios/blob/2b3585aa766213ae9a46c4790ad4f3594ffa213d/src/assetloading/SpectralLibrary.cpp)：
  显式反射率优先于光谱。没有照搬缺失时自动使用 50 的策略。
- [PBRT v4 面元/交点分工](https://www.pbr-book.org/4ed/Primitives_and_Intersection_Acceleration/Primitive_Interface_and_Geometric_Primitives)：
  模型在面元层绑定，命中交点带回材质。本实现沿用既有 MeshTriangle，不重写几何层。

自动雷达角度模型仍为 Lambert，而不是 HELIOS 的 Phong 强度函数；两者不能直接比较强度数值。
