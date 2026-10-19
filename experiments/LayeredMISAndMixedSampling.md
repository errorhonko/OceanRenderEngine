# All 混合事件采样与涂层出口 MIS

2026-10-16。修复粗糙介电 All 的真实采样密度，并增加可选双策略涂层求值。
这是 PBRT v4 事件采样与 LayeredBxDF 连接思想的适配，不是完整移植。

## 1. 粗糙介电 All

旧流程先用宏观法线 Fresnel 选择事件，再采微表面，却在 PDF 中使用微表面 Fresnel。
此外宏观 TIR 会彻底禁止实际仍可能发生的倾斜微表面透射。

现在按顺序：

1. 用独立二维 u 从 VNDF 采样微表面 m。
2. 用实际 dot(wo,m) 计算共享的 double Fresnel / Snell。
3. 按 flags 屏蔽不允许事件，再用独立一维 uc 选择反射/透射。
4. 返回同一个事件概率乘 VNDF 密度与对应方向 Jacobian。
5. 几何无效/实际 TIR 的拒绝计入原试验 N，不重试，不按接受数归一化。

对 All，令 pR=R/(R+T)、pT=T/(R+T)：

- 反射 PDF：pR * pVNDF(m) / (4*abs(dot(wo,m)))。
- 透射 PDF：pT * pVNDF(m) * abs(dot(wi,m)) /
  (dot(wi,m)+dot(wo,m)/etaEffective)^2。
- f 保留真实 Fresnel 和 Radiance/Importance 折射率因子；不能再乘一次事件概率。

### 接口与调用迁移

BxDF/BSDF 新增虚分派/转发重载：

~~~cpp
Sample_f(wo, float uc, const Point2f& u, mode, flags);
~~~

应从 sampler 分别获取一维与二维样本，先保存到局部变量再调用，避免依赖
C++ 函数实参的求值顺序。SimplePathIntegrator 和 PathIntegrator 均已接入；
仅反射/仅透射的单事件材质可省略额外一维消耗。
薄介电新重载也明确使用 uc，不再把方向 u.x 当作事件维度。
原有单叶 Diffuse/Conductor 可继续忽略 uc，并保留二维接口。

旧二维入口仍支持强制 Reflection/Transmission，以及光滑/匹配界面。
**粗糙 All 的旧二维调用明确抛出 invalid_argument**，要求迁移到新重载；
不偷偷复用 u.x、不哈希生成假独立维度，不悄悄改变 PDF 约定。
通用涂层的 DielectricInterface 仍有意保持“只允许单事件”的内部契约。

rho 的旧接口只有二维样本：粗糙界面现在分别估计条件反射与条件透射积分后相加，
而不是伪造第三个随机维度；光滑入口的原离散采样语义保留。

## 2. 涂层 MIS：统一层内方向测度

在某个底层访问点，令 v 朝上、w 为到达底层的朝下传播方向。
当前查询的外部出射方向为 wi。去掉公共 entry/path throughput 后，连接积分是：

I = integral bottom.f(-w,v,Radiance) * abs(v.z) *
    Tr(thickness,v) * top.f(wi,-v,Importance) dOmega(v)。

这里使用与反向出口采样一致的 adjoint 求值；折射率因子已由接口处理，
不能再额外乘一遍 eta^2。两种提议密度都针对这个层内 v：

- qE(v) = top.Pdf(wi,-v,Importance,Transmission)。
- qB(v) = bottom.Pdf(-w,v,Radiance,Reflection)。

特别地，不能把 qB 与 top.Pdf(-v,wi,Radiance,Transmission) 直接相比：
后者是外部方向的密度，测度不同。

每个策略各取一个样本，用 power heuristic，指数 2：

wE = qE^2/(qE^2+qB^2)，wB = qB^2/(qE^2+qB^2)。

最后贡献是两个“integrand / 本策略 pdf * 本策略权重”的和，外层再除以
samplesPerEvaluation。**不要再把两策略和除以 2。**
ConnectionWeight 在 double 下缩放密度后计算平方，避免大/小 PDF 溢出下溢。

### 实现边界

- 反向出口样本失败，只让出口项为零；底层提议仍可贡献并延续内部路径。
- 底层样本复用于该连接和后续内部反射，不重复采样；不以该连接是否命中
  来决定是否允许路径继续。
- 最后允许的底层访问也包含两项贡献。maxInternalReflections=0 仍是完整首阶 TRT。
- Delta 上界面：只用出口离散提议，不把其概率质量与连续底层 PDF 比较。
- Delta 底层：只用底层离散提议连接连续出口。
- 双 Delta 层不能当作有限 BRDF 求值，明确报错。
- 保留有限反射阶截断、原有概率补偿 RR、吸收项和规范方向排序。
  MIS 不消除有限阶截断偏差，也不是“无限深度无偏”的证明。

## 3. 开关与默认值

~~~cpp
LayeredReflectionOptions options;
options.useMIS = true; // 启用双策略，传给 CoatedDiffuseReflection / LiDAR 适配器。
~~~

useMIS=false 保留原出口单策略。**默认仍为 false**，原因是本轮测量显示
当前 Lambert 底层的出口提议已经很好，power MIS 在这些参数上增加方差及耗时。
不能因为名字叫 MIS 就默认宣称“所有涂层更快、更准”。

真实粗糙界面：eta=1.5、alphaX=.4、alphaY=.6、底层 rho=.5，seed 42 起。
零内部反射使用正上方查询、无吸收；
多跳查询使用 normalize(.5,.1,.8) / normalize(-.2,.3,.9)、光学厚度 .2。

| 对照 | seed 数 / 每次样本 / 反射上限 | 单策略均值 | MIS 均值 | MIS/单策略样本方差 |
| --- | --- | ---: | ---: | ---: |
| 半方位支持的合成出口 | 24 / 4096 / 0 | .0252806 | .0255485 | .6366 |
| 尖锐底层合成核 | 32 / 512 / 0 | .0968234 | .1021113 | .01894 |
| 粗糙界面首阶 TRT | 16 / 4096 / 0 | .0623498 | .0623869 | 16.04 |
| 粗糙界面多跳且吸收 | 16 / 2048 / 4 | .0415745 | .0415847 | 2.998 |

这些是独立 seed 的有限样本估计方差，不是单条射线方差，也没有按运行时间归一化。
Debug 这两组真实粗糙界面 MIS 耗时约为单策略的 1.5–1.6 倍；
秒数随构建/机器负载变化，不作为验收断言。均值差均通过 6 标准误差检查。
合成尖锐核只验证数值估计器，不作为新的物理材质或实测验证。

## 4. 验收

新增 MixedSamplingAndMisAcceptance.cpp，并注册到 VS 测试项目和 Tests 筛选器：

- All 外部、内部宏观 TIR、掠射：各 131072 次，seed 42/43/44。
- 独立全方向 PDF 积分：128x256 加密到 256x512，上下两个半球，
  2x4x8 概率箱；检查积分收敛、角度频数、空事件概率。
- 独立 f*abs(cos) 积分核对 MC throughput；包含拒绝事件，分母仍为 N。
- All 采样/查询 f 与 PDF 阈值 5e-5；实测最大分别约 9.95e-6、9.98e-6。
- 固定同一个微表面改变 uc：核对事件选择、混合/条件 PDF 比例与 f 不变。
- Radiance/Importance、光滑/全反射/Delta、动态 BxDF 与 BSDF 转发。
- 两个真实路径积分器：确认独立事件采样接入；单次反射权重含 1/pR，
  因此不能把 uc 强制选中反射的单次估计误当作未经概率补偿的 Fresnel R。
- 同密度双提议的解析 TRT 控制：防止重复计算或额外除 2。
- 出口拒绝但底层项仍有效、单策略零支持、Delta 质量/密度分离、
  最后访问点和多跳 RR，以及多 seed 均值/方差/耗时比较。
- 上轮 CriticalRefractionStudy 独立 Decimal 对照重跑，局部精度指标保持不变：
  全部接受透射样本 f 最大配对相对差 5.925011e-6，PDF 为 3.065983e-6。
  新输出：out/CriticalRefractionStudy/run-20261016-122526-b96e25/。

最终完整 CPU x64 Debug / Release 各 **2847 PASS / 0 FAIL / 0 SKIP**，
新增专项 **62** 项，原条件 PDF 专项 **123** 项也全部保留。
SDK 固定 10.0.26100.0，保留默认 manifest；VS 测试项目/筛选器 XML 有效，
新源文件各注册一次。仍有既存 BRDFUtils 浮点常量截断警告，未扩大清理范围。
未运行 GPU 验收。

使用构建日志确认的实际程序：
tests/OceanRenderEngine.Tests/x64/{Debug,Release}/OceanRenderEngine.Tests.exe。
构建与验收日志归档：
out/LayeredMISAcceptance/run-20261016-122854-9cd131/。

另加多跳 RR（seed 42–57、每次 2048 样本、上限 8）：
单策略均值 .04157390401，MIS .04158405975，方差比 2.99748，
同样通过均值一致性检查。Debug / Release 的上述数值指标一致。

本轮没有提交或推送 Git；解析近似后端仍未实现。
后续已加入[无色涂层随机游走 Sample_f / MIS Pdf](LayeredRandomWalkSampling.md)，
区分比例型路径采样因子与近似方向密度，不把本节界面条件 PDF 混为整体精确 PDF。

## 参考与刻意简化

- [PBRT DielectricBxDF](https://github.com/mmp/pbrt-v4/blob/master/src/pbrt/bxdfs.cpp)：
  独立 uc/u，先采微表面后按其 Fresnel 选事件。
- [PBRT LayeredBxDF](https://github.com/mmp/pbrt-v4/blob/master/src/pbrt/bxdfs.h)：
  双连接与 power MIS 的参考。此处只处理标量双界面、无体散射，
  明确把两提议放在相同的内部方向测度，不机械照抄其全部 BSDF/体散射逻辑。
