# PBRT 主导的粗糙导体雷达反射：第一步

2026-10-14。[总方案](ReflectionMaterialRoadmap.md) 阶段 1。
本文记录固定波长基线及公共层合并。当前文件已按模块整理到 `src/`；
后续新增的同类光谱模式、查询接口及验收见 [SpectralConductorLidar.md](SpectralConductorLidar.md)。

## 实现与公式

- `FresnelUtils.h::FrConductor`：float/double 标量非偏振导体 Fresnel。
- `MicrofacetReflection.h::EvaluateMicrofacetConductorReflection`：局部 +Z 的纯 BRDF。
- `RoughConductorLidarScattering.h`：固定目标波长、世界方向/法线/切线适配，GGX/Smith。
- RGB `ConductorBxDF::f()` 和雷达标量接口共用 `EvaluateMicrofacetConductorReflection`。
  公共内核通过 Fresnel 求值回调适配 Spectrum/double，半程向量和 D/G 只计算一次。
- 旧 `FrComplex` 保留接口，但变为 `FrConductor` 的薄适配；复数 Fresnel 数学只有一份。
  旧参数仍为相对 n/k，雷达参数为绝对 n/k 与外部介质折射率；不把 RGB 当激光波长。
- `ConductorBxDF` 的 Sample_f/Pdf、镜面 Delta 分支和双半球约定保留。
  负半球方向在适配层同时反向后进入局部 +Z 内核，利用已有 GGX 的对称性。
  FrComplex 现在继承公共实现的非法参数检查和匹配介质掠射端点防护。

对实数外部介质折射率 etaI 和导体复折射率 n+i*k，先得到相对复折射率 (n+i*k)/etaI，
计算两种偏振的复振幅 Fresnel，取平方模平均 F。n/k 是绝对光学常数，不是反射率。
法线入射独立解析式：F0=((n-etaI)^2+k^2)/((n+etaI)^2+k^2)。

m=normalize(wi+wo)，f_r=F D(m) G(wi,wo)/(4 cosThetaI cosThetaO)。
两个方向均从表面向外；f_r 单位 sr^-1，不带额外的接收余弦、1/pi 或采样权重。
已有 EstimateSingleReturn 与 LidarIntegrator 分别负责接收几何/能量和 Monte Carlo 1/N。

n≥0、k≥0，且不能同时为零；etaI>0。匹配无吸收介质零反射、其余精确掠射端点 F=1；
有限输入 cosine 按 PBRT 约定钳到 [0,1]。非法/数值溢出明确报错，不传播 NaN。
BRDF 本身允许超过 1，不钳成反射率；能量约束针对半球积分。

## 使用

```cpp
#include "reflection/lidar/RoughConductorLidarScattering.h"

// n905/k905 由调用方选定覆盖 905 nm 的光学常数来源。
auto metal = std::make_shared<RoughConductorLidarScattering>(
    etaIncident905, n905, k905, alphaX, alphaY, 905.0f);

ObjMaterialBindingOptions options;
options.overrides["bare_metal"] = std::make_shared<const SurfaceMaterial>(
    SurfaceMaterial{nullptr, metal});
// 使用既有 ObjSceneAsset 导入、SurfaceLidarScattering 分派。
```

构造最后一个参数是此组 n/k 对应的唯一目标波长（nm）。Evaluate 的查询波长必须相同，
其他正波长抛 out_of_range；不会把该值默认为全波段恒定材料。
不包含默认铜/铝数据，也不读取 PBRT 截止 885.6 nm 的内置金属表当 905 nm 数据。

AlphaX/AlphaY 直接指定 GGX 分布参数。FromRoughness 使用现有 PBRT sqrt 映射，
roughness∈[1e-6,1]，alpha至少1e-3。未增加 GGX 的精确 RMS 坡度映射。
没有 shading normal 时用 geometric normal；纠正 ns/ng 半球，并投影 dpdu 到切平面。
退化/缺失 dpdu 自动构造 Frame；几何背面为零，非法几何法线拒绝。

## 已验证范围

`RoughConductorLidarAcceptance.cpp` 使用合成 n=.3、k=3，不能称为真实铜或 905 nm 实测参数。
所有数据均为公式验证参数，不依赖新增网络/下载或外部 BRDF 库。
导体首版 61 项加上公共层合并回归 11 项，共 72 项检查全部通过；
CPU Release / Debug 全量各 2300 项通过，零失败、零跳过。
此导体模型只验证 CPU；GPU 基础 compute/DXR 验收不覆盖该 BRDF。源码随版本检查点提交。

- 正常入射、外部介质指数、相对光学常数缩放、k=0 介质极限/TIR、匹配零反射及掠射。
- 标量与 RGB 适配结果对照；独立 Cartesian GGX/复振幅测试公式检查多角度、多通道结果。
- 双半球、互易性、RGB 通道独立、粗糙采样的 f/PDF 一致性和镜面 Delta 分支回归。
- 计数分布验证 RGB 求值只调用一次 D/G，未按通道重复计算几何项。
- D/G/F/分母解析、互易性、局部/世界旋转与方向缩放不变性。
- 确定性 mu/phi 中点半球积分：128×256，法线/60°入射；法线加密到256×512检查收敛。
  检查能量≤1（给数值积分0.001容限），并允许单次微表面散射造成的能量缺失。
  这里只在 ng=ns 同一平坦法线下验证，不是任意着色法线扰动的能量证明。
- 光谱/几何参数错误、目标波长不匹配、Delta 请求、退化方向/法线/切线处理。
- MTL 显式覆盖到真实网格交点，再到解析回波、1/N、传播时间和波形能量。

## 刻意没有实现

光学常数的光谱容器/数据库导入、镜面 Delta、导体薄膜、涂层、多次微表面散射补偿、
偏振、接收孔径积分、BRDF 采样/PDF 新接口、完整多跳或透射。
新模型不自动替换已有城市材料，不根据名字/Ks/Ns 猜 n/k/alpha；用户显式指定 override。
下一小步按总方案阶段 2，建立独立 SpectralOpticalConstants，而非复用 rho∈[0,1] 容器。

## 主参考

https://www.pbr-book.org/4ed/Reflection_Models/Conductor_BRDF
https://www.pbr-book.org/4ed/Reflection_Models/Specular_Reflection_and_Transmission
https://www.pbr-book.org/4ed/Reflection_Models/Roughness_Using_Microfacet_Theory
