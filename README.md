# OceanRenderEngine

一个使用 C++20 编写的 CPU 渲染与海面激光雷达仿真项目。渲染部分参考 *Physically Based Rendering, 4th Edition* 的光传输、采样和 BSDF 设计；海面部分从 Elfouhaily 方向波谱生成随时间变化的三角形网格；激光雷达部分正在实现脉冲发射和单次表面回波估算。

> **当前状态：开发中。** 各模块已有独立实现和确定性验收测试，但尚未组成可一键运行的“海面生成 → 激光发射 → 回波波形”程序。当前 `main.cpp` 仍是 Stanford Bunny 的栅格化示例，并非海面 LiDAR 演示。

## 已实现的模块

| 模块 | 当前能力 | 主要代码 |
| --- | --- | --- |
| CPU 渲染器 | 球体与三角形求交、几何/着色法线、BSDF 与材质、点光/平行光/环境光/漫反射面积光；`SimplePathIntegrator` 用于理解直接光与 BSDF 采样，`PathIntegrator` 加入 MIS 和俄罗斯轮盘赌 | [`SimplePathIntegrator.h`](SimplePathIntegrator.h)、[`PathIntegrator.h`](PathIntegrator.h)、[`BSDF.h`](BSDF.h) |
| 频谱海面 | Elfouhaily 长波/短波谱和方向扩展；可注入其他谱函数的频域随机海面；时间演化、二维逆 FFT、高度场及顶点法线 | [`ElfouhailySpectrum.h`](ElfouhailySpectrum.h)、[`OceanFrequencyField.h`](OceanFrequencyField.h)、[`OceanHeightField.h`](OceanHeightField.h) |
| 网格与加速 | 共享顶点的海面三角网格、通用三角网格面元、equal-counts BVH；网格变化后可 `Refit()` 或 `Rebuild()` | [`OceanSurfaceMesh.h`](OceanSurfaceMesh.h)、[`TriangleMeshAggregate.h`](TriangleMeshAggregate.h)、[`BVHAccel.h`](BVHAccel.h) |
| LiDAR 基线 | 理想单方向/均匀圆锥脉冲发射；接收视场、口径、回程遮挡、飞行时间；给定单波长 BRDF 数值时估算一次表面回波能量 | [`LaserEmitter.h`](LaserEmitter.h)、[`LidarReceiver.h`](LidarReceiver.h)、[`LidarReturnEstimator.h`](LidarReturnEstimator.h) |

目前的海面数据流如下，频谱输入使用函数接口，因此不必把随机海面生成器绑定到 Elfouhaily 模型：

```text
ElfouhailySpectrum / 其他二维波谱
    → OceanFrequencyField: H(k, t)
    → OceanFFT + OceanHeightField: h(x, z, t)
    → OceanSurfaceMesh: 共享顶点、法线、三角形索引
    → TriangleMeshAggregate + BVHAccel: 射线求交
```

LiDAR 的当前单次回波链路是 `LaserEmitter → 实际表面命中 → LidarReceiver 的视场/回程可见性 → EstimateSingleReturn → LidarReturnSample`。两条链路尚未由完整的 LiDAR 积分器统一驱动。

## 单次回波模型

当前估算器按发射方向采样一条射线，再连接其命中点与接收口径：

$$
\widehat E_{\mathrm{rec}}
=W\,f_r(\lambda,\omega_i,\omega_r)\,
\cos\theta_r\,\Omega_{\mathrm{rec}}\,\eta\,V,
\qquad
\Omega_{\mathrm{rec}}\approx
\frac{A_{\mathrm{rec}}\cos\theta_a}{R_r^2}.
$$

其中 `energyWeightJ` 是发射方向的采样权重 $W$，$f_r$ 是单波长 BRDF，$V$ 表示回程可见且位于接收视场内。多条射线估算同一次脉冲时，必须对**全部发射样本（包括未命中的样本）**取平均。到达时刻按发射时刻加 $(R_e+R_r)/c$ 计算。

此处的 `brdfAtWavelengthPerSr` 仍由调用方提供；验收测试使用 Lambert 漫反射作为可计算的基线。渲染器目前以 RGB `Spectrum` 工作，不能将它直接视为激光波长处的海水光谱 BRDF。理想镜面反射也不能简单代入一个有限的 BRDF 值。

## 构建与验证

项目当前使用 Windows 上的 Visual Studio C++ 工程：需要支持 `.slnx` 的 Visual Studio、C++20 工具链以及 Windows SDK。工程文件当前指定 MSVC `v145` 工具集，建议选择 **x64 / Debug** 或 **x64 / Release**。

1. 打开 [`OceanRenderEngine.slnx`](OceanRenderEngine.slnx)。
2. 生成解决方案；将 `OceanRenderEngine.Tests` 设为启动项目并运行。
3. 测试是控制台验收程序，全部通过时会输出 `All acceptance tests passed.`。

在 Visual Studio Developer PowerShell 中也可以单独构建并运行测试工程：

```powershell
msbuild .\tests\OceanRenderEngine.Tests\OceanRenderEngine.Tests.vcxproj /t:Build /p:Configuration=Debug /p:Platform=x64
.\tests\OceanRenderEngine.Tests\x64\Debug\OceanRenderEngine.Tests.exe
```

[`tests/OceanRenderEngine.Tests`](tests/OceanRenderEngine.Tests) 包含渲染、频谱/FFT、高度场/网格、BVH 与 LiDAR 的确定性验收用例。随机海面的测试通过显式 seed 保持可复现。

截至 2026-09-17，本机 x64 Debug 与 Release 验收程序均通过 **247 项检查**。

根目录的 [`main.cpp`](main.cpp) 目前只演示栅格化：它会尝试读取运行目录下的 `models/stanford-bunny.obj`，并写出 `bunny_render.ppm`。**该 OBJ 模型未包含在仓库中**；若想运行此示例，需要自行提供模型并从仓库根目录启动。海面与 LiDAR 的现阶段结果请以验收测试为准。

## 下一阶段

- 实现激光波长处的海面方向散射模型，并明确处理镜面/近镜面回波。
- 将海面网格、BVH、发射器和接收器接入完整 LiDAR 积分器；对多条发射射线求平均，并按到达时间形成回波波形。
- 在物理模型稳定后加入大气衰减、探测器响应及定量对照实验。

## 参考资料

- [*Physically Based Rendering: From Theory to Implementation*, 4th Edition](https://pbr-book.org/4ed/contents) 与 [pbrt-v4 源码](https://github.com/mmp/pbrt-v4)：渲染架构、BSDF、采样和光传输的主要参考。本项目是针对海面与 LiDAR 目标的轻量实现，并非完整 pbrt 移植。
- T. Elfouhaily, B. Chapron, K. Katsaros, D. Vandemark, [“A unified directional spectrum for long and short wind-driven waves”](https://doi.org/10.1029/97JC00467), *Journal of Geophysical Research: Oceans*, 1997：海面波谱模型参考。

许可证文本见 [`LICENSE.txt`](LICENSE.txt)。
