# 光学常数光谱容器

2026-10-14。本文记录阶段 2 第一小步的数据容器和确定性查询。
后续已完成 [RoughConductorLidarScattering 光谱接入](SpectralConductorLidar.md)，
后续已加入[独立三列读取器及文献铝数据](OpticalConstantsTableReader.md)，容器本身不负责文件解析。

## 文件和接口

- `src/core/spectrum/SpectralOpticalConstants.h`：独立、header-only 的容器。
- `tests/OceanRenderEngine.Tests/SpectralOpticalConstantsAcceptance.cpp`：验收测试。
- 主项目在 `src/core/spectrum` 筛选器中注册头文件；测试已加入统一验收入口。

```cpp
// 人工测试数据，不代表某种真实金属。
SpectralOpticalConstants table(
    {{400.0, 0.2, 2.0}, {600.0, 0.6, 4.0}, {1600.0, 1.6, 8.0}},
    {"synthetic", "example", "not measured data"});

OpticalConstants nk = table.Evaluate(905.0);
// nk.n = 0.905, nk.k = 5.22
```

`OpticalConstantsSample` 成对保存同一个波长对应的 n/k；`Evaluate()` 返回
`OpticalConstants`，不返回 RGB、反射率或 BRDF。只读的 `Samples()` 和 `Metadata()`
用于诊断；`SampleCount()`、`Contains()`、最小/最大波长用于检查有效频带。

## 单位与模型约定

- 波长是以 nm 表示的真空波长；没有隐式的 µm→nm 换算。
- n/k 是无量纲的绝对光学常数，采用本项目被动介质 `eta = n + i*k` 的约定。
- n/k 均可超过 1；不能复用强制反射率处于 [0,1] 的 `SpectralReflectanceCurve`。
- n/k 有限且非负，不能同时为零；允许非吸收介质极限 k=0，以及 n=0、k>0。
- 入射介质折射率不是表的一部分。后续由现有 `FrConductor()` 转换成相对复折射率。
- 来源、样品条件是随表拥有的描述信息，不自动推断温度、材料类别或测量真实性。

## 查询算法

公共数值层已整理到 [SpectrumInterpolation.md](SpectrumInterpolation.md)：本容器与
反射率容器共用区间定位，n/k 共用同一个区间权重；业务数据校验和越界规则不变。

1. 构造时要求至少两行，波长有限、为正并严格递增；不自动排序或合并重复行。
2. 查询先检查有限正波长，再要求其处于两端都包含的表范围。
3. 二分查找首个波长不小于查询值的样本。恰好命中节点时直接返回原值。
4. 对相邻样本计算 `t = (lambda - lambda0) / (lambda1 - lambda0)`，
   然后分别用 `std::lerp(n0, n1, t)` 和 `std::lerp(k0, k1, t)` 插值。
5. 数值下溢若把有效端点插成 (0,0)，明确报错，不交给 Fresnel 后再产生除零。

查询复杂度 O(log N)，无需采样或随机数。分段线性插值是有限表格的基线近似，
不保证稀疏数据能捕获吸收峰，也不是满足色散关系的拟合模型。

非法数据/查询抛出 `std::invalid_argument`，频带外查询抛出 `std::out_of_range`，
插值的联合零下溢抛出 `std::underflow_error`。移走数据后若表不再有效，查询及
端点访问抛出 `std::logic_error`；`Contains()` 安全返回 false。

## 验收范围

测试使用人工数据，覆盖节点、非均匀间隔插值、905/1550 nm、闭区间及越界、
非法数值、重复/乱序、n/k 超过 1、k=0、纯虚折射率、极端有限值与下溢、
所有权/复制/移动，以及调用既有 Fresnel 的法线入射解析值和介质极限。
这些检查不等于已验证真实金属散射、完整 BRDF、LiDAR 光谱适配或外部数据解析。

## 验证结果

本步验证：x64 Debug/Release CPU 验收各 2354 项通过，零失败、零跳过；
其中光学常数容器新增 54 项。Windows SDK 为 10.0.26100.0，保留默认 manifest。
运行的是本次独立测试项目编译日志中的 `tests/OceanRenderEngine.Tests/x64/<configuration>/`
可执行文件；未运行 GPU 验收。本步仅修改 CPU 独立容器、项目注册和测试入口。

## 下一步

现有固定波长导体构造方式已保留，同时增加了光谱数据输入方式；查询波长时取 n/k
再调用已合并的 BRDF 内核。后续文献铝数据及来源、单位、许可、频带已单独记录；
粗糙度仍为假设参数，下一功能阶段是涂层漫反射，禁止对未知波段静默外推。
