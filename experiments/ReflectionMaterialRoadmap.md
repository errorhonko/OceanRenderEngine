# 反射材质实现与验证方案

2026-10-14。目标：在已完成的 OBJ/MTL 逐面绑定和单次 LiDAR 回波链路上，
逐步扩展物理反射模型；不是把所有表面自动判定成金属或把 RGB 当近红外。

## 当前进度

阶段 0 已完成：31 个既有头文件迁入独立筛选器，加上新模型共 32 个；
清理 6 个无文件引用的旧 ReflectionModel 展示节点，未移动实体文件或修改包含路径。
阶段 1 已完成：[粗糙导体及公共层合并](RoughConductorLidar.md)。导体共 72 项检查通过，
CPU Release/Debug 各 2300 项通过，零失败、零跳过；未验证 GPU 分支。
下一小步为阶段 2 的光学常数光谱容器，尚未下载或引入 RGL/Mitsuba 等新外部库。
导体公共层已合并：RGB ConductorBxDF 与雷达类共用 BRDF 内核，FrComplex 适配公共
FrConductor；旧采样/PDF/Delta 分支保留，不维护第二份复数 Fresnel 或导体 BRDF 公式。

## 参考分工

- **PBRT v4 主导模型实现**：Fresnel、导体/介质、微表面 D/G、层间传输、
  材质/BxDF/交点分工。使用官方书和 mmp/pbrt-v4，不机械照搬全光谱容器。
- **Mitsuba 3 交叉核对**：独立 roughconductor、roughdielectric、roughplastic 和 measured 实现。
  参数、坐标、BRDF/BRDF*cos/PDF/吞吐量约定必须对齐后再比较。
- **RGL 实测 BRDF 对照**：532/905 nm 均在其标注的 360–1000 nm 范围内；
  约 4 nm 谱间隔，含材料角度变化。1550 nm 不在此库频带内。
  处理后的某些角度来自外推，雷达回向散射验证需检查原始测量覆盖。
- **RefractiveIndex.INFO 提供 n(lambda)/k(lambda)**：逐条核对原论文、样品、温度和频带；
  光学常数不包含表面粗糙度或完整 BRDF。
- **ECOSTRESS 保留光谱基线**：rho(lambda) 不能唯一决定 BRDF，核对 Measurement 实验几何。
- **HELIOS++ 工程对照**：材质字段/扫描/波形，不把其 Phong 强度项直接作为本接口的纯 BRDF。

PBRT 当前内置 Cu/Ag/Au/Al 的 eta/k 表约 298.8–885.6 nm，不能作为 905/1550 nm 数据。
RGL 原始 loader 的 eval 返回 f_r*cos，不可原样转发给要求纯 BRDF 的 LidarScatteringModel。
库代码许可与测量数据许可分别检查；RGL 数据默认 CC0（有例外时从其说明），loader 为 BSD-3-Clause。
MERL 是 RGB 数据，不能当近红外曲线，且有研究/学术用途限制。

## 分类与明确范围

- 干燥粗糙哑光表面：Lambert 光谱基线，后续用实测角度曲线决定是否升级。
- 裸金属：导体 Fresnel + GGX/Smith 微表面模型，需要 n、k 和 alphaX/alphaY。
- 粗糙透明界面：介质反射/透射，折射与介质传播另属传输扩展。
- 塑料、清漆木材、车漆：涂层与底层模型；车身材质不能仅因底材是金属就按裸金属处理。
- 海面：保留已有谱驱动 Beckmann/Smith/Fresnel 和白冠组合。
- 反光膜、标志牌：专门的逆反射响应，不能默认普通镜面或 Lambert。

光谱总反射率不得同时完整赋给漫反射项并另加一份镜面项，避免能量重复计算。
实测曲线要区分方向-半球反射率、方向反射因子等测量量，不自动混用。

## 分阶段实施

### 阶段 0：组织与保持兼容

建立 VS `头文件/ReflectionMaterials` 及 Bindings、Materials、BxDFs、Fresnel、Microfacet、LiDAR 子筛选器。
仅修改展示归属，不移动实体文件，不改变 #include。普通材质与雷达模型继续共用 SurfaceMaterial 绑定。
OBJ 自动转换仍默认 Lambert；新模型通过显式 overrides 或手动绑定启用。

### 阶段 1：单目标波长的粗糙导体（当前小步）

1. FresnelUtils 中增加标量非偏振 FrConductor，依据 PBRT 复折射率 Fresnel。
2. MicrofacetReflection 中增加局部 +Z 的纯导体 BRDF 求值。
3. RoughConductorLidarScattering 把世界方向转换到表面局部坐标，复用已有 GGX D/G。
4. 显式指定目标波长、入射介质折射率、导体折射率实部和消光系数、alphaX/alphaY。
   此处 n/k 是该波长处的调用方输入，没有默认金属或内置近红外数据；其他波长拒绝。
5. 接 SurfaceMaterial/MTL override，并验证到回波、波形的现有链路。

公式：m = normalize(wi+wo)，f_r = F(wi·m) D(m) G(wi,wo)/(4 cosThetaI cosThetaO)，单位 sr^-1。
F 为无量纲反射比例；D 是投影面积归一化法线分布，G 是微表面遮蔽，外部宏观可见性另算。
不额外乘 receiver 余弦、rho、1/pi、PDF 或 1/N；这些各有既有职责。

此步仅有限粗糙度、反射、非偏振、局部无限导体界面，不是金属薄膜、吸收体传播或 Delta 镜面。
alpha 至少 1e-3；不提供 GGX 的“精确 RMS 坡度工厂”。FromRoughness 沿用当前 PBRT sqrt 映射。
微表面单次散射会遗漏内部多次散射能量，不能将积分强行归一化为 1。

### 阶段 2：光学常数的光谱容器与数据

新增独立的 SpectralOpticalConstants，存储 lambda/n/k，而非复用限定 rho∈[0,1] 的容器。
严格 nm 单位、递增样本、n/k 合法性、频带外报错、来源与样品条件。
先选择一个覆盖目标波长的金属数据集，核对 Fresnel 解析值；不把所有材料表一次性打包进仓库。
之后让导体/介质模型按波长查询，保持模型接口和逐面绑定稳定。

### 阶段 3：涂层漫反射

参照 PBRT CoatedDiffuse 与 Mitsuba roughplastic，先做明确简化的无体散射薄涂层，
考虑界面 Fresnel、透射衰减、底层漫反射和内部反射，避免直接无约束相加。
参数需区分底层 rho 与整体反射率。复杂车漆/珠光/多层金属涂层不在首版范围内。

### 阶段 4：实测 BRDF 接入与验证

封装 RGL C++ loader 为 MeasuredLidarScattering，正确处理角度坐标、谱插值、cos 约定和频带。
先选低复杂度样本对比 Lambert/GGX/涂层；再选车漆等复杂样本。
雷达 wi≈wo 的近回向散射区域要区分实测与外推，不能仅凭渲染外观判定准确。

### 阶段 5：雷达特有扩展

逆反射、接收孔径积分、Delta 反射方向判定、透明目标折射/多跳。
窄峰情况下，现有点方向 BRDF × 接收立体角近似需验证；不能用巨大有限 BRDF 冒充 Delta。
再研究参数拟合、材料分类、湿润状态、噪声下的强度与掉点，单独记录测量和仿真假设。

## 每阶段的验收门槛

- 解析法线入射 Fresnel，k=0 的介质极限、匹配介质零反射、掠射极限。
- 方向互易性、世界旋转/方向缩放不变性、法线半球及切线退化处理。
- 确定性半球积分，检验反射能量≤1；微表面单次散射允许小于 1。
- 固定几何/波长/seed 的回波解析值、1/N、到达时间与波形能量守恒。
- 非法参数、超频带、Delta 请求必须明确失败，不能用钳制掩盖物理错误。
- 与独立来源/参考实现对照，保留参数约定、积分收敛检查及误差阈值。

## 主参考

- https://www.pbr-book.org/4ed/Reflection_Models/Conductor_BRDF
- https://www.pbr-book.org/4ed/Reflection_Models/Specular_Reflection_and_Transmission
- https://www.pbr-book.org/4ed/Reflection_Models/Roughness_Using_Microfacet_Theory
- https://www.pbr-book.org/4ed/Light_Transport_II_Volume_Rendering/Scattering_from_Layered_Materials
- https://github.com/mmp/pbrt-v4
- https://mitsuba.readthedocs.io/en/latest/src/generated/plugins_bsdfs.html
- https://rgl.epfl.ch/pages/lab/material-database
- https://github.com/rgl-epfl/brdf-loader
- https://refractiveindex.info/about
- https://speclib.jpl.nasa.gov/
- https://heliospp.readthedocs.io/en/alpha-dev/intensity_fwf.html
