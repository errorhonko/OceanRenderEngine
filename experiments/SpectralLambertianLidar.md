# 光谱 Lambert 雷达散射接入

2026-10-13。新增 `SpectralLambertianLidarScattering.h`，放在 Visual Studio
`头文件/LiDAR/Scattering` 筛选器。原固定反射率 `LambertianLidarScattering`
及其调用方保持不变。

## 接口与物理含义

这是 `LidarScatteringModel` 的实现，拥有一份只读访问的 `SpectralReflectanceCurve`。
构造时复制左值或移动右值；文件只在加载时读取，不在每次交点求值时读取。
借鉴 PBRT 的 Lambert 反射率 / pi 关系与已有分段线性光谱查询：

`f_r(lambda) = curve.Evaluate(lambda) / pi`，单位 sr^-1。

测量曲线自身并不能恢复完整角度 BRDF。这里把 rho(lambda) 解释为 Lambert 模型的
漫反射半球反射率，是明确的角度散射基线假设，不是对真实材料角度响应的复现。
不转换成 RGB，不做 CIE 归一化，不添加 Fresnel 或额外余弦。
与固定 Lambert 一样，几何正反面检查交由 `EstimateSingleReturn` 完成。
非法波长和超出测量频带保留曲线异常，不默认取端点、零或固定值。

## 使用与生命周期

```cpp
#include "reflection/lidar/SpectralLambertianLidarScattering.h"

auto curve = SpectralReflectanceCurve::FromAsterFile("data/spectra/asphalt.txt");
SpectralLambertianLidarScattering scattering(std::move(curve));
LidarIntegrator integrator(world, emitter, receiver, samplesPerPulse, scattering);
auto pulse = integrator.SimulatePulse(sampler);
```

示例文件路径需要调用方提供，不会自动下载。散射模型必须比 integrator 活得更久，
因为既有 integrator 保存的是模型引用；不要传入临时散射模型。
固定波长场景仍可使用原 `LambertianLidarScattering(rho)`。

调用顺序：`SimulatePulse -> EstimateSingleReturn -> Evaluate(wavelengthNm)`
`-> curve.Evaluate -> rho/pi -> 回波能量 -> LidarWaveform.AccumulatePulse`。
原能量估计和 Monte Carlo 1/N 不变，光谱变化只影响 BRDF，不改变几何传播时间。

## 验证范围

新增 `SpectralLambertianLidarAcceptance.cpp` 并注册到 CPU 验收入口。
使用自造光谱夹具，不依赖 HELIOS 下载、网络或真实样本再发布。
覆盖端点、插值、单位、来源元数据、非法/越界波长、模型多态、拥有关系、
与固定 Lambert 一致性，以及球顶正常入射解析回波能量、两波长能量比、
1/N、到达时刻、零反射率和波形能量守恒。
新增 31 项检查全部通过；CPU 全量 Release / Debug 各 2104 项通过，零失败、零跳过。
这不是重新扫描城市的结果；端到端测试使用已知几何与自造光谱，验证光谱到回波的连接。

这一步已连接到回波积分接口，但没有自动替换既有场景模型。
2026-10-14 已补齐手动逐三角形绑定与命中后模型分派，详见
[表面材质绑定](SurfaceMaterialBinding.md)。随后已实现 [OBJ/MTL 自动转换](ObjMaterialBinding.md)
及城市的可选材质加载入口；缺少测量的材料仍需要显式实验假设，并非全场景都有实测数据。
