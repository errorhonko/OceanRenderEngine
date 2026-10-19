# 涂层随机游走 Sample_f 与外部 MIS Pdf

2026-10-16。参考 PBRT v4 LayeredBxDF，沿现有无体散射、透明粗糙上界面 +
不透明 Lambert 底层模型扩展。现有 LiDAR Evaluate 链不替换，不自动改写 OBJ 材质。

## 文件与职责

- src/reflection/layered/LayeredReflectionSampling.h：
  通用标量层内随机游走、比例型路径样本、独立外部 MIS PDF 近似。
- CoatedDiffuseReflection.h：新增 Sample_f / Pdf，底层仍由每次传入的标量反射率构建。
  原 Evaluate / Estimate 和条件事件接口不变；另加界面 SampleAll / PdfAll。
- src/reflection/bxdfs/CoatedDiffuseBxDF.h：
  明确“中性、无色”的可见光 Spectrum 适配，不把近红外曲线当 RGB。
- src/reflection/materials/CoatedDiffuseMaterial.h：
  普通 Material / BSDF 的显式接入。不自动启用，不推断 MTL 是否为车漆。
- LayeredRandomWalkAcceptance.cpp：解析控制、RR、出射方向和真实 PathIntegrator 验收。

## 1. Sample_f：抽的是内部路径，不是余弦外部提议

公开标量入口：

~~~cpp
Sample_f(wo, uc, u, baseReflectance, mode, flags);
Pdf(wo, wi, baseReflectance, mode, flags);
~~~

从上界面 All 采样开始，反射则直接返回，透射则进入：
底层反射 → 上界面 All → 出射或继续向下。
每次真正离开整个层时才返回最终外部 wi。

外部底层不透明，因此 All / Reflection 都允许全部隐藏 R/T 路径；
Transmission / Unset 返回零。背面仍不支持，未伪装成双面模型。
所有采样坐标必须有限且属于 [0,1)；拒绝样本不重试。

### 比例型 f / pdf 记账

把返回成对变量记为 A / P：

- 入口透射：A=f_entry*abs(cos_entry)，P=p_entry。
- 其他中间界面：A*=f_j*abs(cos_j)，P*=p_j。
- 每次穿层：A*=exp(-opticalThickness/abs(cos_direction))。
- 最终出口：A*=f_exit，P*=p_exit，**不在此乘最后的外部余弦**。
- 入口直接反射：A=f_top，P=p_top。

外部积分器仍使用 A*abs(wi.z)/P，最后余弦只乘一次。
A 不是 Evaluate(wo,wi)；P 也不是把所有内部路径积分后的最终方向密度。
所有样本，包括入口直接反射，都标记 pdfIsProportional=true，
因为同一个 wi 仍可能由其他内部路径产生。

成对变量在过大/过小时共同缩放，保持 A/P 不变，避免长路径连乘下溢。
RGB float 适配还会共同缩放后转换；极微小吸收贡献的 float 下溢按零路径终止，
不把这一数值边界冒充物理 RR。

### RR 与深度约定

保持现有 f 求值器的约定：
maxInternalReflections=M 是“上界面反射回底层”的最大次数，
M=0 仍允许直接 R 和首阶 TRT，不是零反射，也不是 PBRT 的逐边界 depth 计数。

在一次允许继续的上界面反射之后，若 r>=3 且 beta=A/P<.25：
取存活率 s=beta，失败返回空；存活时 P*=s。
不再另外将 A 除以 s，以免重复补偿。硬性 M 上限仍会截断高阶项，
不能声称使用 RR 后无限阶误差自动消失。

采样流来自 options.seed、wo、uc/u，并与 f 及 Pdf 的流分域。
useMIS 和 f/PDF 样本预算不会改变随机游走本身。
两种 TransportMode 在无 RR 时保留相同方向；RR 在层内使用的权重可能因模式不同，
因此不承诺 RR 开启后两个模式逐路径完全相同。

## 2. Pdf：供外部 MIS 的独立近似，不替代 P

跟随 PBRT 的分工，保留直接反射和首阶 TRT 估计，用正的均匀尾项覆盖未求出的阶数：

q_MIS(wo,wi) = .9*(q_direct + estimate(q_TRT)) + .1/(2*pi)。

只对外部正半球有值；本模型不透射到底面，故尾项采用均匀半球，
而非通用双面 PBRT 的均匀球面。此近似不承诺积分等于 1，
尤其不能要求它等于 Sample_f 返回的路径乘积 P。

### 对参考实现的刻意适配

本实现用当前已修复的 All 事件概率描述直接项和 TRT 目标：

q_TRT = integral p_top_all(wo,t) *
        p_bottom(-t,v) * p_top_all(-v,wi) dOmega(t) dOmega(v)。

用条件入口透射提议时，补 p_top_all / p_top_conditional 的入口事件比例。
内层两提议分别为反向出口透射和底层反射，MIS 都比较同一个内部 v 的密度；
不能把外部 wi 的密度直接当内部 v 的密度。

光滑上界面使用 Snell 映射的逆立体角 Jacobian：
dOmega_inside / dOmega_outside = cos_outside / (eta^2*cos_inside)，
并保留入口、出口的真实透射事件概率。这一项有解析验收。
底层无反射时不计 TRT；即使如此，公开代理仍保留已说明的均匀尾项。

Pdf 不按光学厚度、RR 和所有高阶项精确重放最终方向的边缘密度。
它只用于 MIS 权重，不能通过“将 Evaluate 除以这个 Pdf”替代路径样本权重。
默认 pdfSamples=32，独立于 samplesPerEvaluation=64；后者仍用于 f 求值。
Pdf 使用不同的固定查询种子，不与 f 的规范方向查询流重用。

## 3. 路径积分器接入

原 PathIntegrator 已用比例型 f/pdf 更新吞吐量，并在 pdfIsProportional 时
调用 BSDF.Pdf 查询外部 MIS 密度，因此不改动这项既有职责。

新增 BxDF.NeedsComponentSample 虚查询，BSDF 转发：
涂层对外虽只有 Reflection，内部却仍须用独立 uc 选择入口 R/T，
不能仅凭外部 type 没有 Transmission 就把 uc 固定成 .5。

Diffuse / Conductor 等单叶行为和既有介电材质不变。
新普通材质是显式无色标量模型，不含任意 RGB 底色、光谱折射率、
体散射、多层金属底层或相干薄膜干涉。LiDAR 仍用独立光谱适配器。

## 4. 使用示意

~~~cpp
LayeredReflectionOptions options;
options.maxInternalReflections = 32;
options.russianRoulette = true;
options.pdfSamples = 32;

CoatedDiffuseReflection coating(1.0, 1.5, .2f, .3f, 0.0, options);
const float uc = sampler.Get1D();
const Point2f u = sampler.Get2D();
auto path = coating.Sample_f(wo, uc, u, .5);
if (path)
    beta *= path->f * std::abs(path->wi.z) / path->pdf;

// 仅用于已知外部方向的 MIS 权重，不能替换上面的分母：
double qForMIS = coating.Pdf(wo, wi, .5);
~~~

普通路径追踪的显式材质：
CoatedDiffuseMaterial(etaIncident,etaCoating,alphaX,alphaY,
                      baseReflectance,opticalThickness,options)。
alpha 使用已存在的直接 alpha 参数约定，不额外套粗糙度转换。

## 5. 验收设计与结果

- 匹配折射率：采样方向为真实余弦分布，逐样本 throughput=.5。
- 匹配且吸收：独立一维角积分 .2827001，MC .2824853，SE=.0001540。
- 光滑 eta=1.5、rho=.5：用内部余弦 Fresnel 积分求有限阶几何级数，
  测 M=0/4/16；均含 R 的 Delta 事件，拒绝仍除以原始 N。
- M=16：MC .3157518，独立参考 .3160708，SE=.0006410。
- 独立 RR 离散核 262144 条：参考 .42857136，
  无 RR .42797038，RR .42829330，后者有 1948 次存活补偿。
  该核仅为算法控制，不当作实测或互易介电材质。
- 50 次内部反射的极小成对因子：共同缩放不改变 throughput，
  M=49 则明确截断，区分 RR 与硬上限。
- 粗糙 eta=1.5、alpha=.4/.6、rho=.5、M=0：
  原 f 的独立方向积分 .2108700 / .2110731，
  随机游走 .2103486，SE=.0006937。
- 去除代理尾混合后，首阶 TRT 真实方向概率项积分 .4537971，
  实际采样接受率 .4525146；8 个 mu 箱全通过统计/积分容限，
  两档方向积分 L1 差 .00077047。没有把完整 MIS 代理冒充精确 PDF。
- 同粗糙界面 M=4、RR 开启：随机游走 .2442889，
  独立 f 积分 .2434717，SE=.0006440，有 15 次真实 RR 存活。
- 实际 PathIntegrator + 均匀环境：匹配 Lambert 均值 .4008544，
  参考 .4，SE=.0014482；粗糙 M=0 为 .2148237，
  独立积分 .2110731，SE=.0029648，均通过 6 标准误差及积分容限。
- 无状态重复、反射掩码、输入边界、Delta/连续测度、内部模式因子、
  独立组件样本、灰度 Spectrum 适配和旧 rho 接口均有专项检查。

最终 CPU x64 Debug / Release 各 **2921 PASS / 0 FAIL / 0 SKIP**，
新增随机游走专项 **74** 项。两种构建的上述数值结果一致，
原条件 PDF 123 项与混合采样/MIS 62 项也全部保留。
实际执行的是构建日志输出的
tests/OceanRenderEngine.Tests/x64/{Debug,Release}/OceanRenderEngine.Tests.exe。

日志和关键源码快照归档：
out/LayeredRandomWalkAcceptance/run-20261016-215901-e9845b/。
SDK 10.0.26100.0、默认 manifest；项目及筛选器 XML 有效、无重复项。
既存 BRDFUtils 浮点常量截断警告未扩大清理。

临界角独立 Decimal 对照重跑：
out/CriticalRefractionStudy/run-20261016-215442-9c41ed/，
全接受样本的 f 最大配对相对差仍为 5.925011e-6，
PDF 为 3.065983e-6；没有因新采样接口破坏上一轮数值链。
本轮不验证 GPU、不自动绑定场景材质、不提交或推送 Git。

## 主参考

- [PBRT v4 LayeredBxDF Sample_f/PDF 源码](https://github.com/mmp/pbrt-v4/blob/master/src/pbrt/bxdfs.h)
- [PBRT v4 分层材质章节](https://www.pbr-book.org/4ed/Light_Transport_II_Volume_Rendering/Scattering_from_Layered_Materials)

这里复用独立事件采样、随机游走、比例型 f/pdf 和 RR 的思想；
没有宣称与完整 PBRT 的体散射、双面、多层或 PDF 近似逐项相同。
