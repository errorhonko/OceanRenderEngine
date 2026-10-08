# 精确坡度到法线映射基线

`OceanSlopeNormalDistribution.h` 是独立于旧 `OceanNormalDistribution` 的实验性组件。它不调用具体海浪谱，不改变现有 LiDAR BRDF。输入是谱积分得到的世界 X/Z 短波坡度协方差，以及当前长波宏观法线。

## 假设与范围

短波世界坡度为零均值二维高斯，协方差正定；长短波独立，沿世界 Y 方向叠加高度。不包含长短波动力学调制、非高斯尾部、空间/时间相关性、泡沫或破碎。这里“精确”仅指该基线内部的坡度到法线映射。零方差或秩亏协方差是 delta 情形，本类明确拒绝，不添加人工粗糙度下限。

## 接口

```cpp
OceanSlopeNormalDistribution normals(unresolvedSlopes, resolvedNormal);

// 两个独立 U[0,1) 样本；不内置随机数发生器或种子。
Vector3f m = normals.SampleNormalWorld(u1, u2);
double pdf = normals.PdfNormalWorld(m);
double areaDensity = normals.AreaDensityWorld(m);

// 已有两个独立标准正态变量时，也可以直接调用：
Vector3f reference = normals.NormalFromStandardNormal(z1, z2);
```

采样法线使用 Vector3f，内部坡度和密度使用 double。查询接受非单位方向并在内部归一化；非法方向抛异常，世界下半球及地平线返回零。查询不会裁剪长波局部下半球。极端数值溢出显式报错；高斯尾部下溢为零是浮点限制。均匀样本允许 0，不允许 1；Box-Muller 使用 `log1p(-u1)` 处理零端点。

## 三种测度不能混用

令 n0 为长波单位法线，p=-n0.x/n0.y，q=-n0.z/n0.y。对世界单位法线 m，令 dp=-m.x/m.y-p，dq=-m.z/m.y-q，P(dp,dq) 为短波坡度高斯密度。

- 采样按世界水平投影面积等权，精确法线为 normalize(-p-dp,1,-q-dq)。
- `PdfNormalWorld` = P / m.y^3，单位 sr^-1，世界上半球积分为 1；不是可见法线 PDF。
- `AreaDensityWorld` = n0.y * P / m.y^4，表示每单位长波宏观面积、每单位立体角的微表面面积。它不是概率密度，其普通立体角积分不必为 1。

面积推导：微表面 dAm=dAh/m.y，宏观面 dA0=dAh/n0.y，所以面积密度是方向 PDF 再乘 n0.y/m.y。完整支持域满足向量投影积分 `integral D_A(m) m dOmega = n0`，以及带符号标量投影积分 `integral D_A(m) dot(m,n0) dOmega = 1`。

对于倾斜宏观面，一些样本可能满足 dot(m,n0)<=0。只在局部上半球积分 D_A*cos 不一定为 1，不能直接作为标准局部 NDF 替换旧模型。尚未提供遮蔽 G、反射方向 PDF、Fresnel 或几何/着色法线校正。

这采用 PBRT 微表面“每单位宏观面积的微表面面积分布”概念，但支持域不同，因此有意不提供模糊的 `D()` 别名：
https://pbr-book.org/4ed/Reflection_Models/Roughness_Using_Microfacet_Theory

## 验证方法

专项测试 `tests/OceanRenderEngine.Tests/OceanSlopeNormalDistributionAcceptance.cpp` 接入已有验收入口。

- 确定性样本和独立逆协方差公式；尺度不变性；平坦长波时退化为旧 Beckmann 面积分布。
- 独立世界球面角度积分检查 PDF 归一化、坡度均值/协方差、向量投影及带符号投影守恒。
- 宽坡度案例保留局部下半球，并验证裁剪会改变投影面积。
- 固定种子 20 万样本，对照球面 PDF 积分得到的方向锥概率。锥概率的估计量是指示函数均值；只有按对应分布采样、且不裁剪/重新归一化时，才是该模型方向概率的无偏估计。它不是回波估计，不含 Fresnel、可见性或反射余弦权重。
- Elfouhaily 谱积分接入检查，非法输入及掠射边界检查。

下一阶段先研究与该支持域匹配的遮蔽模型，再接 BRDF；不能直接复用旧 GGX 的 G。

## G1 候选的解析验证（2026-09-26）

目前实现只在 `tests/OceanRenderEngine.Tests/OceanSlopeMaskingAcceptance.cpp` 内，尚未添加正式遮蔽接口、G2 或修改 BRDF。采用除背面指示函数外不依赖微法线的单方向遮蔽闭合假设；面积守恒并不单独证明真实空间可见性。

记 w 为从表面向外的单位观察方向，定义：

```
mu = w.y - p*w.x - q*w.z
sigma^2 = w.x^2*Cxx + 2*w.x*w.z*Cxz + w.z^2*Czz
X = mu - w.x*dp - w.z*dq
```

X 为一维高斯。正向投影面积 A+(w)=integral D_A(m)*max(0,dot(w,m)) dOmega = n0.y*E[max(0,X)]。

对 sigma>0，令 a=mu/sigma：

```
E[max(0,X)] = sigma*phi(a) + mu*Phi(a)
g1(w) = mu / E[max(0,X)]                  // 仅 mu>0
G1(w,m) = indicator(dot(w,m)>0) * g1(w)   // m.y>0
```

phi/Phi 是标准正态密度和 CDF。宏观背面 mu<=0 时，本单面候选将 g1 定为0，不使用绝对值扩展。sigma=0 时，正部期望为 max(0,mu)。候选的可见法线密度 `D_A*max(dot(w,m),0)*G1/dot(w,n0)` 在宏观正面归一化；这不等于验证了实际海面的可见法线统计。

验证内容：

- 21 组均值/标准差组合，对照独立一维高斯积分（积分范围标准正态 +/-12，10万中点，断点处拆分）。
- 三组宏观面与协方差，包括平坦各向异性、倾斜正相关、倾斜负相关；每组9个观察方向，使用1024x512独立世界球面积分。
- 最大正向投影面积绝对差依次为 5.1407e-6、2.33008e-7、1.10736e-6；最大可见投影面积守恒绝对差依次为 5.1407e-6、2.23945e-7、9.74346e-7。这是当前离散积分与解析式的差，不是物理遮蔽误差。
- 世界正上方 w=(0,1,0)：sigma=0，mu=1，g1=1，即使微法线属于宏观下半球也不任意剔除。
- 平坦宏观面时，与 Gaussian/Beckmann Smith 解析式一致。
- 宏观掠射 mu->0+ 且 sigma>0 时，g1 ~ sqrt(2*pi)*mu/sigma ->0。对 p=1、q=.25，C=(.64,.25,.12)，采用 w=normalize(t+epsilon*n0)，t=normalize(1,1,0)，epsilon从1e-1到1e-8，g1从0.50873下降到6.36373e-8。
- 同一倾斜模型沿世界水平 w=(-1,0,0) 时 g1=0.961105：世界地平线不等同于宏观掠射。

新增26项检查，Debug/x64和Release/x64均420项验收通过。仍沿用本机编译参数 `/p:GenerateManifest=false`，未修改项目清单配置。

参考：Heitz 2014 的可见投影面积约束与 Smith 法线独立遮蔽假设；当前倾斜高度场的 mu/sigma 形式由本模型推导。下一步需要显式短波几何等外部参考验证，及收发联合可见性 G2；特别是冻结表面同路径往返不能默认将两个单向概率相乘。
https://jcgt.org/published/0003/02/03/
