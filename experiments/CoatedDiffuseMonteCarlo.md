# 涂层漫反射：默认蒙特卡洛求值后端

2026-10-15。按调用方选择实现顺序：先完成 MC，解析近似留作可选后端，
不是先用简单 Fresnel 混合公式冒充层间传输。未自动修改场景已有材质。

## 分层与复用

- `src/reflection/layered/LayeredReflection.h`：标量双界面估计器、后端选项。
- `CoatedDiffuseReflection.h`：现有 DielectricBxDF / DiffuseBxDF 的适配，
  GGX/Smith 上界面加不透明 Lambert 底层；局部坐标 +Z 朝涂层外。
- `src/reflection/lidar/CoatedDiffuseLidarScattering.h`：世界坐标适配，
  接受底层常数 rho 或拥有的 SpectralReflectanceCurve；纯 BRDF，单位 sr^-1。
- 沿用 SurfaceMaterial、OBJ override、SurfaceLidarScattering 和现有回波链路；
  不从 MTL 自动猜测涂层参数，不把既有场景默认 Lambert 全部替换。

只从既有介质 BxDF 取无色系数、从单位 DiffuseBxDF 取 Lambert 基函数/采样。
波长 rho 保持 double 标量，不将 RGB 颜色反解成 905/1550 nm 反射率。
折射率和 alpha 本版为常数；可变波长部分只有底层 rho。
底层 rho 不是整个涂层表面的总反射率，不能把整体实测曲线直接当底层输入而不校准。

## 后端开关与参数

```cpp
LayeredReflectionOptions options;
options.method = LayeredEvaluationMethod::MonteCarlo; // 默认
options.samplesPerEvaluation = 64;                   // 每次 BRDF 求值
options.maxInternalReflections = 32;                 // 0 = 仅 TRT
options.seed = 42;
options.russianRoulette = true;

auto scattering = std::make_shared<CoatedDiffuseLidarScattering>(
    1.0, 1.5, .2f, .3f, // eta 外/内、alphaX/Y，示例参数不是实测
    .5,                 // 底层 rho；也可传入 SpectralReflectanceCurve
    0.0,                // 光学厚度 tau，不是米
    options);
```

`AnalyticApproximation` 枚举已预留，当前选择它会抛出 logic_error，绝不静默回退。
新后端必须先实现并验证，才能允许选择它。
samplesPerEvaluation 合法范围 [1,1048576]；内部反射次数 [0,4096]。
alpha 至少 1e-3，Delta 材质需独立采样/接收孔径处理，不能直接输出巨大有限 BRDF。

## 算法与估计量

外部 wo/wi 都背离表面。无体散射层的所有贡献为：上界面直接反射，
以及透入涂层、一次或多次底层散射、最终透出的路径。

1. 上界面 `top.f(wo,wi,Radiance)` 只计算一次，不除内部采样数。
2. 每个样本从 wo 强制采样上界面透射（Radiance），得到朝下方向 a。
   独立从 wi 反向采样上界面透射（Importance），得到朝下方向 b。
3. 吞吐权重分别为 `f * abs(cosTheta) / pdf`，pdf 是指定反射/透射分支的
   立体角密度；透射的 Radiance/Importance 模式保留既有 eta^2 约定。
4. 当前基底连接贡献 `beta * bottom.f(-w,-b) * exitWeight`。
   entry/exitWeight 各包含完整向下/向上传播的 `exp(-tau/abs(w.z))`。
5. 底层采样反射向上，乘底层 f*cos/pdf 和向上传播透过率；
   上界面强制采样内部反射向下，乘其实际 Fresnel 权重和向下透过率。
   继续连接到同一反向出口，累加下一阶内部反射。
6. 达到反射次数上限后停止；从第 4 次循环后，对 beta<.25 的续行路径
   用生存概率 beta 做 RR，并除以生存概率补偿。
7. 所有样本之和除以原始 N，包括被拒绝/零贡献样本，再加直接反射。

强制分支复用既有采样代码，避开它的 2D 全事件混合路径；没有复制 Fresnel/VNDF。
初始版只使用反向出口连接一种提议。后续已加入[可选双策略 MIS](LayeredMISAndMixedSampling.md)，
默认保留单策略；MIS 不是在所有涂层参数下都降低方差。
这是对 PBRT LayeredBxDF 思想的独立简化，不是其完整移植或数值逐项等价版本。

## 误差与明确限制

- 理想随机采样下，估计目标是选定局部界面模型的有限反射阶级数；
  有限 N 有方差，有限反射上限对无限层间传输有截断偏差。
  RR 本身有概率补偿，但不消除深度上限偏差。
- `pathsAtReflectionLimit` 只计到达上限的路径，不能当作遗漏能量上界；
  即使后续反射本来为零，也可能记录到达上限。
- 固定 seed 和规范排序后的局部 wo/wi 决定样本；每条内部路径独立派生序列，
  无共享可变 RNG。交换方向得到相同有限估计是构造保证，不是互易性独立证明。
- 同一方向重复查询重放相同样本。外层增加准直光束射线数，不会自动降低
  这个固定 BRDF 查询的内部估计误差；需单独增加 samplesPerEvaluation 或比较多个 seed。
- 底层常数 64 样本只是起点，不宣称所有角度/高反射率均已收敛。
- 本版无体散射、无侧向位移/层内飞行时间、无偏振、无相干薄膜干涉。
  tau 是无量纲光学厚度；tau=0 为无吸收，不是 PBRT 默认光学厚度的照搬。
- 上界面仍是既有单次微表面模型，未补回其微表面多次散射损失。
  公共接口采用 float 数学/采样，权重与总和用 double，不是全 double 的新介质内核。
- 仅增加通用标量求值和 LiDAR 适配，此初始阶段尚未实现采样；后续已加入[无色模型随机游走适配](LayeredRandomWalkSampling.md)。
- 与既有 LiDAR 类相同，几何法线做外部半球判断、着色法线做局部计算；
  当前能量检查只覆盖二者相同的情况，不宣称解决任意扰动着色法线能量问题。

## 验收方案

`tests/OceanRenderEngine.Tests/CoatedDiffuseLidarAcceptance.cpp`：

- 默认 MC、未实现解析后端明确失败、参数/方向/频带边界、确定性与 seed 切换。
- 匹配折射率退化为 rho/pi；纯吸收层等于两个穿越方向的 Beer-Lambert 因子。
- 黑底只剩旧介质 BRDF；改变内部样本数不缩放直接反射。
- 拒绝样本不重试；半方位透射的合成模型验证平均分母仍是原始 N。
- 光滑上界面只用于通用估计器的独立解析对照（不开放生产 Delta BRDF）：
  `rho/pi * (1-Fi)*(1-Fo) / [eta^2*(1-rho*FavgInternal)]`，
  FavgInternal 用独立余弦加权 Fresnel 数值积分；MC 65536 样本，32 次内部反射，
  无 RR 相对容限 1%，开启 RR 1.5%。该公式不适用于一般粗糙涂层。
- 粗糙上界面零次内部反射，用确定性 BTDF 立体角积分独立核对 f*cos/pdf，容限 2%。
- 同法线条件的方向-半球积分仅作能量上界 smoke test，不是完整材质标定。
- 光谱复制/世界坐标退化/OBJ 逐面绑定/单脉冲与 16 条射线能量归一化/传播时间/波形守恒。

### 已执行结果

CPU x64 Debug / Release 验收项目均成功编译，并运行构建日志中实际输出的 exe：
各 **2662 PASS / 0 FAIL / 0 SKIP**，其中本文件新增涂层检查 112 项。
SDK 固定 10.0.26100.0，保留默认 manifest。VS 项目与筛选器 XML 有效，无重复注册项。
本轮未运行 GPU 验收，也未声称验证完整 GPU 或可见光层状 BSDF。

- 光滑涂层：MC 0.092757，开启 RR 0.0928662，独立解析控制 0.0928859；
  不开启 RR 的相对差约 0.14%。
- 粗糙涂层首阶 TRT：MC 0.0634532，独立 BTDF 积分 0.0634575，相对差约 0.007%。
  正入射各向同性时方位角无关，积分使用 4096/8192 个 mu 中点验证收敛；
  先前低 mu 分辨率未收敛，因此加密参考积分，而不是放宽验收容限。
- 同法线、rho=.5、alpha=.35 的方向反射能量 smoke test 为 0.270667；
  不将此单点结果当作任意参数的能量证明。

独立匹配折射率控制还发现并修正了现有公共函数的精度问题：
Refract 在 eta==1 时直接返回 -wi；FrDielectric 在两侧 eta 相等时直接返回 0。
否则极掠射时 sqrt 差消可能造成虚假全反射。加入精确掠射回归；
上述全量 CPU 测试是在这两项公共修正之后通过的。

## 后续解析近似讨论

MC 细化进度：已补两个界面的条件 Pdf / IsDelta 及
[107 项采样一致性检查](LayeredConditionalPdf.md)，CPU Debug/Release 各 2769 项通过。
修正粗糙条件透射被宏观 TIR 误拒绝的问题；临界角 float 散射值精度另行记录。
可选 MIS 现已接入，见[实现与实验记录](LayeredMISAndMixedSampling.md)；
随后已加入[涂层随机游走 Sample_f / MIS Pdf](LayeredRandomWalkSampling.md)，
明确使用比例型路径权重；无色普通材质可显式接入路径追踪，非任意彩色/多层模型。

先明确目标：加速当前粗糙层模型，而不是把其样本数设为零。
光滑界面的上述级数可以作为最简单的解析控制；粗糙界面还要对方向依赖透射、
内部反射和遮蔽积分，通常不能仅用两个外部 Fresnel 因子替代。
可考虑参考 roughplastic 的积分/预计算近似，与本 MC 基线在统一参数和角度上
比较误差与成本，再实现 AnalyticApproximation 后端。先不把近似公式接入默认结果。

## 主参考

- [PBRT v4：Scattering from Layered Materials](https://www.pbr-book.org/4ed/Light_Transport_II_Volume_Rendering/Scattering_from_Layered_Materials)
- [PBRT LayeredBxDF 源码](https://github.com/mmp/pbrt-v4/blob/master/src/pbrt/bxdfs.h)
- [Mitsuba roughplastic 源码，后续解析近似参考](https://github.com/mitsuba-renderer/mitsuba3/blob/master/src/bsdfs/roughplastic.cpp)
