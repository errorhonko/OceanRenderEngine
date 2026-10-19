# 涂层界面条件 PDF 与采样一致性

2026-10-15。MC 细化第一步，只补接口与一致性验证，未接入 MIS，
未实现整体涂层的 Sample_f/Pdf，也未实现解析近似后端。

## 接口与测度

`src/reflection/layered/CoatedDiffuseReflection.h` 的 DielectricInterface、
DiffuseInterface 新增：

```cpp
double Pdf(const Vector3f& wo, const Vector3f& wi,
           TransportMode mode, BxDFReflectionType flags) const;
bool IsDelta() const noexcept;
```

- 复用 DielectricBxDF / DiffuseBxDF 的既有 Pdf，不维护第二份 VNDF/Jacobian 公式。
- flags 必须恰好是 Reflection 或 Transmission；All、Unset、未知值明确报错。
  Sample_f 同样检查，避免不小心进入旧全事件混合分支。
- 方向为局部坐标、背离界面的有限单位向量；容许 float 单位长度平方误差 1e-3。
  不在 Pdf 内隐式重归一化。宏观切向方向的连续密度为零，切向入射不采样。
- 连续 Pdf 为 dOmega(wi) 测度，条件是“要求采哪个事件”，不是“已采样成功”。
  有效方向上的积分可小于 1；其余概率对应 null/rejected 事件，不重试、不重新归一化。
- Delta 事件的连续 Pdf=0；其 Sample_f 返回的 pdf 是离散概率质量，不可二者相等比较。
  匹配折射率即使 alpha 非零，也是直行 Delta 透射。
- 底层 rho 只缩放 f，不缩放余弦采样 PDF；rho=0 也可返回方向样本和非零 PDF，
  但散射值为零、贡献为零。
- Radiance/Importance 改变透射 f 的折射率因子，不改变这套条件方向采样密度。

## 本轮发现并修正的条件采样错误

旧 DielectricBxDF 粗糙分支先用宏观法线 Fresnel 决定是否允许继续，
即使 flags=Transmission，也会在宏观 TIR 时整批提前返回 null。
但粗糙界面部分倾斜微表面仍可透射，与非零方向 PDF 不一致。

回归案例：eta=1.5，alphaX=.6、alphaY=.85，
wo=normalize(.8,.1,-.6)。旧条件透射零成功；修正后完整 131072 个试验中，
成功比例 .381652832，独立方向 PDF 积分 .381161226，差在统计/积分容限内。
这些数值是方向采样概率，不是透射能量。

修正仅在显式指定单事件时把预采样分支概率设为 1，先采 VNDF，
再由实际微表面 Fresnel / Refract 判断 TIR，f 保留真实 Fresnel。
同时把 Refract 失败检查移到使用其方向之前。
初始阶段未重构的 All 宏观分支概率 / 微表面 PDF 问题，现已通过独立 uc/u 修复，
见[All 与 MIS 更新](LayeredMISAndMixedSampling.md)。
本接口明确禁止 All，不能宣称整个 DielectricBxDF 的全部采样模式已验证。

## 测试与参数

`tests/OceanRenderEngine.Tests/LayeredInterfacePdfAcceptance.cpp`：初始 107 项；局部混合精度更新后 123 项。

1. 参数/事件掩码、上下半球、切向边界、Delta 与连续测度分离。
2. Lambert 上下半球及黑底：4 个 mu 箱 × 8 个 phi 箱的解析概率。
3. 正入射、各向同性 GGX 反射：使用独立解析累计分布，
   有效概率为 1/(1+alpha^2)，其余为 null 事件。
4. 各向异性外部反射/透射、内部反射、宏观 TIR 下透射、掠射反射：
   独立 dMu*dPhi 方向积分，128×256 加密为 256×512；
   32 个概率箱积分 L1 差小于 .004 后，再与样本频数比较。
5. 每个主要分布使用 131072 次试验、seed 42；模式配对使用 8192 次、seed 43。
   所有频数除以原始试验次数，不是成功次数。
6. 分箱误差使用 6 sigma 加每箱 .00015 的方向积分容限；解析分布不加积分容限。
   采样返回 PDF 与查询 PDF 最大相对差阈值 .005；本轮实际最大约 1.4124e-5。
   不是实测材质或全部参数范围的误差保证。

### 初始结果（数值链更新前）

CPU x64 Debug / Release 均成功编译并运行本次构建实际输出的 exe：
各 **2769 PASS / 0 FAIL / 0 SKIP**，包含新增 **107** 项条件 PDF 检查。
SDK 10.0.26100.0，默认 manifest 未关闭，未运行 GPU 验收。

| 分布 | 成功比例 | 连续 PDF 积分 |
| --- | ---: | ---: |
| Lambert | 1 | 1 |
| 正入射 GGX 反射，alpha=.6 | .735534668 | .735294102（解析） |
| 内部宏观 TIR 下的粗糙透射 | .381652832 | .381161226 |
| 掠射反射 | .933181763 | .934989582 |

以上比较没有把 PDF 强行归一化成 1，所有箱均通过统计容限。

## 初始数值限制及后续修复

额外记录了 Sample_f.f 与 f(wo,wi) 的重算差异。
旧 float 折射/Fresnel 在微表面透射接近临界角时具有病态相对敏感性：
上述内部透射试验中，12 个样本重建的透射微表面余弦 abs(dot(wi,wm))<.02，
其中最坏样本的 f 为 .000187341066 与 .000185335142 sr^-1，
相对差约 1.07%，绝对差约 2.006e-6 sr^-1。

这 12 个样本仍全部参加采样 PDF、方向直方图、总概率与 null 率检查；
没有因 f 的相对误差将其丢弃。临界区域的 f 最大相对差单独记录，
不将它作为 PDF 失败条件；常规区域仍检查 f 重算，最大相对差约 .00037644。
测试并未证明极临界角 f 可达同样相对精度。

2026-10-16 已完成[独立固定样本误差对照](CriticalRefractionStudy/REPORT.md)：
复现 1.07% 差异，定位到单位长度残差、eta 倒数、点积、Snell 及重建误差共同作用。
仅换 Fresnel 公式不能消除；从固定微表面开始用 double 重算折射/重建显著改善。
诊断当时尚未修改正式数值内核；后续已落实下述改动。误差加权能量贡献仍未研究。
局部混合精度方案已验证，性能基准仍待补充；不通过钳制 f/PDF 掩盖差异。

## 后续实现（已完成，见最新记录）

保留当前单出口提议作为对照，增加底层方向采样连接与 MIS。
两种提议必须先统一到同一内部方向立体角测度；Delta 分支特殊处理。
用多 seed 的均值、方差与时间比较，不把加入 MIS 直接等同于提升精度或效率。

参考：[PBRT LayeredBxDF](https://github.com/mmp/pbrt-v4/blob/master/src/pbrt/bxdfs.h)、
[PBRT 层状材质章节](https://www.pbr-book.org/4ed/Light_Transport_II_Volume_Rendering/Scattering_from_Layered_Materials)。

## 2026-10-16：局部混合精度更新

折射/Fresnel/半向量重建与 Jacobian 的关键中间量改为 double，
对外方向、Spectrum 和 PDF 接口不变，NDF/VNDF/G 仍为 float。
共享 Fresnel/Snell 分支同时计算功率 R/T，避免 float 1-R 消减。
细节及原始实验路径见 [数值链报告](CriticalRefractionStudy/REPORT.md#局部混合精度落地与回归2026-10-16)。

现在全部接受样本（含临界区域）均检查 f 与 PDF，
阈值均为 5e-5，而不是初始的 .005。
内部条件透射最大 f 相对差由 .0107073 降至 5.92501e-6，
最大 PDF 相对差为 3.06598e-6，接受数仍为 50024/131072。
主要条件分布中最大 f 相对差约 8.52124e-6，最大 PDF 相对差约 3.06598e-6。
新增旧最坏样本及独立 Decimal 参考等 16 项检查；
CPU Debug/Release 各 2785 PASS / 0 FAIL / 0 SKIP，专项 123 项。
模式配对的折射率平方参考也改为 double 倒数，避免测试沿用旧舍入。

这是数值一致性改进，不是增加 MC 样本数、强行归一化概率或更换物理模型。
随后已修复原生 BxDF 的 All，并加入可选 MIS 与多 seed 方差/耗时对照，
见[最新实现记录](LayeredMISAndMixedSampling.md)。内部涂层接口仍保持条件事件契约。
