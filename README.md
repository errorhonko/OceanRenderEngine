# OceanRenderEngine

使用 C++20 编写的 CPU 渲染、频谱海面与激光雷达仿真项目。渲染和反射模型以 PBRT v4 为主要参考，目标是把“几何与光学回波 → 接收电信号 → ADC → 测距 → 点云”串成可验证的链路。

> 开发中：已有确定性验收和静态场景扫描示例，但不是经过真实仪器标定的商用仿真器，也尚未接入 SLAM 系统。默认入口仍是 Bunny 栅格化示例；完整 LiDAR 链路请运行验收或导入实验。

最新阶段检查点（2026-10-19）：已重跑 **Urban 同位姿 RGB＋360° LiDAR**，
11,520次发射生成9,613个数字波形测距点；CPU x64 Debug/Release 各3,103项通过。
见[阶段总结](experiments/VersionCheckpoint20261019.md)及
[可复现实验、参数与聚合结果](experiments/UrbanSensorCheckpoint/README.md)。
外部资产、场景预览和点云保留本地，不因公开代码而假定拥有其再发布授权。

## 当前能力

| 模块 | 已实现内容 | 入口 |
| --- | --- | --- |
| CPU 光传输 | 图元求交、共享网格、BVH 构建/重建/Refit、几何与着色法线；直接光、BSDF 采样、MIS、俄罗斯轮盘赌 | [积分器](src/render/integrators/)、[几何](src/geometry/) |
| 频谱海面 | Elfouhaily 方向谱、固定 seed 随机频域海面、时间演化、逆 FFT、高度及频域坡度、海面网格和白冠基线 | [海面模块](src/ocean/) |
| 光束与扫描 | Delta、均匀锥、远场高斯方向模型；独立随机/二维分层采样；通用扫描计划、旋转多线、平台轨迹与连续多帧 | [发射器](src/lidar/emitter/)、[扫描](src/lidar/scan/) |
| 光学回波 | 单次表面命中、接收视场/口径、回程可见性、物理飞行时间、多射线脉冲能量平均及波形积累 | [回波仿真](src/lidar/simulation/) |
| 接收与测距 | 线性光电转换、一阶响应、跨阻放大、电压噪声、ADC；基线估计、峰值/重心测距、多回波、门限/峰间隔及输出选择 | [接收器](src/lidar/receiver/)、[信号处理](src/lidar/signal_processing/) |
| 点云输出 | 带脉冲/扫描/时间信息的测量结果、世界/传感器坐标点、ASCII PCD 输出 | [点云模块](src/lidar/output/) |
| 场景与材质 | OBJ/MTL、坐标转换、组装和逐面绑定；光谱 Lambert、介质/Beckmann 海面、光谱 n/k 粗糙导体、MC 涂层；可选 HELIOS 经验响应及 Urban 真实地形采样贴地 | [场景](src/scene/)、[反射材质](src/reflection/) |
| GPU 基础层 | DX12 仿射 compute、持久单三角形 BLAS/TLAS、DXR 最近命中/有限线段遮挡与缓冲区复用 | [GPU 说明](gpu/README.md) |

导体的 RGB BxDF 与 LiDAR 适配共用 BRDF 内核及 Fresnel 实现，不把 RGB 通道当激光波长，也不重复维护两套反射公式。已加入光谱光学常数、共同插值层和数据/角度响应配置工厂；涂层提供有限阶 MC 求值、随机游走、俄罗斯轮盘及近似外部 MIS 密度，不声称完整无限层精确 PDF。详见[反射材质方案](experiments/ReflectionMaterialRoadmap.md)。

## 数据链路

```text
海浪谱 → 随机频域 H(k,t) → 逆 FFT 高度/坡度 → 共享海面网格 → BVH

扫描计划 + 平台轨迹 → 脉冲与光束采样 → 场景命中 + 单波长 BRDF
  → 连接接收器、可见性、回波能量及到达时间
  → 光学波形 → 光电/模拟电压响应 → 噪声 → ADC
  → 基线/门限/多回波提取 → 测距与校正 → 点云 → PCD
```

一张脉冲波形由光束内的多条采样射线合成，不是每条 Monte Carlo 射线独立输出一个点。距离由接收信号处理估计；几何真值只用于评估，不作为测距答案。

单次回波能量基线为：

$$
\widehat E_{\mathrm{rec}}
=W\,f_r(\lambda,\omega_i,\omega_r)
\cos\theta_r\,\Omega_{\mathrm{rec}}\,\eta\,V,
\qquad
\Omega_{\mathrm{rec}}\approx
\frac{A_{\mathrm{rec}}\cos\theta_a}{R_r^2}.
$$

这里 \(W\) 是发射方向采样的能量权重；所有发射样本（包括未命中）参与脉冲平均。传播时刻为发射时刻加 \((R_e+R_r)/c\)。接收立体角采用小口径近似，有限 BRDF 不等价于理想镜面 Delta。

## 构建与运行

需要 Windows、支持 `.slnx` 的 Visual Studio、MSVC **v145**、C++20 和 **Windows SDK 10.0.26100.0**。六个 CPU/GPU/实验项目均固定此 SDK；主工程和测试工程使用 UTF-8 编码编译。

当前建议将仓库克隆到纯英文路径：发布前对照发现，中文项目目录下的测试资源定位仍有异常，本版本尚未修复该路径编码问题。

在 Visual Studio 打开 [OceanRenderEngine.slnx](OceanRenderEngine.slnx)，选择 **x64 / Debug** 或 **x64 / Release**。CPU 验收项目为 OceanRenderEngine.Tests。

在 Visual Studio Developer PowerShell 中，从仓库根目录运行：

```powershell
msbuild .\OceanRenderEngine.slnx /t:Rebuild /m:2 /p:Configuration=Debug /p:Platform=x64
.\x64\Debug\OceanRenderEngine.Tests.exe
.\tests\OceanRenderEngine.GpuTests\bin\x64\Debug\OceanRenderEngine.GpuTests.exe
```

Release 使用相应配置和输出目录。单独构建某个项目时输出位置可能不同，请以构建日志的实际路径为准，避免运行旧程序。

GPU 普通计算需要 SM6.0；硬件 RayQuery 验收需要 DXR 1.1/SM6.5。默认使用硬件；显式 `--warp` 只验证软件 compute，并跳过 DXR，不能当作 GPU 性能证据。CPU 仿真不依赖 GPU 计算接口。

## 验证与示例

2026-10-19 当前 CPU 阶段验证：

- x64 Debug / Release 正常构建并运行 CPU 全量验收，各 **3103 PASS / 0 FAIL / 0 SKIP**；计数依赖本机已获取外部资源。
- Urban 独立扫描与视觉实验通过，并核对 PCD、脉冲分组、共享位姿、投影和线性图像；这不是完整 SLAM、真实仪器标定或性能基准。
- 本轮未重新验证 GPU、Win32、完整海面数值研究和整套解决方案；此前结果保留为下列历史记录，不扩写为当前保证。

2026-10-14 历史本机验证：

- x64 Debug、Release 整套解决方案重建通过，**正常启用清单生成**，不需要关闭 mt.exe。
- CPU 两个配置各 **2300 PASS / 0 FAIL / 0 SKIP**，包含本机已获取的外部场景。
- AMD Radeon RX 7900 XTX 上两个配置均通过硬件 compute/DXR 验收。
- 两个海面数值实验工程 Debug/Release 编译通过，未重新运行完整实验。

完整范围和日志说明见 [SDK 验证记录](experiments/Sdk26100Verification.md)。Win32 运行及完整 GPU 渲染器未验证；存在既有编译警告，并非所有警告均已清理。

另以不含模型缓存的暂存源码副本验证：英文路径下 Release 正常构建，**2235 PASS / 0 FAIL / 2 SKIP**；跳过项为 Toyblocks 和 Urban 外部场景。

测试中的合成 OBJ/MTL/光谱 fixtures 已提交。HELIOS++ 下载资源因独立资产许可未完全确认，不随仓库发布；缺失时相关外部场景验收会报告 SKIP，因此新克隆的检查总数可能少于本机结果。可通过固定版本获取脚本恢复本地数据：

```powershell
.\models\helios_toyblocks\FetchToyblocks.ps1
.\models\helios_urban_static\FetchUrbanStatic.ps1
```

脚本校验资源哈希，不覆盖内容不一致的本地文件。详见 [Toyblocks 资源说明](models/helios_toyblocks/README.md) 和 [Urban 资源说明](models/helios_urban_static/README.md)。

[扫描 Toyblocks](experiments/ToyblocksImport.md) 和 [静态 Urban 导入](experiments/UrbanStaticImport.md) 提供完整导入/扫描示例；对应 Run.ps1 在 VS x64 开发终端执行，导出 PCD 和评估结果到独立输出目录。它们不是对 HELIOS XML survey 的完整复现。

最新的[Urban RGB＋LiDAR 示例](experiments/UrbanSensorCheckpoint/README.md)使用显式 TerrainContact 布局和共享传感器位姿。脚本需要 PowerShell 7、numpy 和 Pillow；保留原始线性RGB、数字测距点和完整诊断，不修改默认主程序。

[src/app/main.cpp](src/app/main.cpp) 仍是 Bunny 栅格化入口，需要自行提供运行目录下的 `models/stanford-bunny.obj`；它不是默认 LiDAR 演示，仓库不包含该模型。

## 目录与约定

源码位于 `src/`，VS 筛选器对应实体目录；头文件使用模块限定路径。世界坐标默认右手系、Y-up、米；海面是 X/Z 高度场；着色局部坐标以 +Z 为法线。OBJ 源单位和坐标轴由调用方显式声明，不自动猜测。

目录职责及迁移记录见[项目结构](experiments/ProjectStructure.md)。测试、数值实验、第三方依赖和外部模型获取脚本分别放在 `tests/`、`experiments/`、`third_party/` 和 `models/`。

## 当前边界与下一阶段

- 当前光学主链路为一次表面回波；多次反射、水下体散射和大气介质仍是扩展方向。
- 海面短波使用有限积分频带、统计微表面及坡度映射近似；白冠为基线模型，不等于完整破碎波流体仿真。
- 粗糙导体支持显式单波长 n/k 或已加载的光谱 n/k；已有一份来源明确的铝数据，尚不是通用金属材质库。MC 涂层已提供有限阶基线及随机游走；解析近似后端、通用实测 BRDF 导入和真实材质标定尚未完成。
- 接收器是可控基线模型；光子计数接口为占位，尚未完成实际仪器标定和完整偏振模型。
- GPU 仅为单三角形/单位实例的同步基础模块，尚未接入整个海面、材质或 LiDAR 仿真；没有性能加速结论。
- 点云可用于后续 SLAM 对接，但本仓库尚未实现或验证 KISS-ICP、FAST-LIO、LIO-SAM。

最新版本汇总见[2026-10-19检查点](experiments/VersionCheckpoint20261019.md)，此前目录/SDK整理见[2026-10-14检查点](experiments/VersionCheckpoint20261014.md)。后续优先学习点云配准、激光里程计与SLAM，不继续无边界扩张仿真器功能。

## 参考与许可

- [PBRT v4](https://pbr-book.org/4ed/contents) / [官方源码](https://github.com/mmp/pbrt-v4)：光传输、采样、BSDF 和微表面主要参考，本项目不是完整 PBRT 移植。
- [Elfouhaily et al., 1997](https://doi.org/10.1029/97JC00467)：统一方向海浪谱。
- [HELIOS++](https://github.com/3dgeo-heidelberg/helios)：光束、扫描与外部场景的工程参考；外部资产与代码许可须分别核对。
- tinyobjloader 及 stb 保留各自上游许可；项目许可证见 [LICENSE.txt](LICENSE.txt)。
