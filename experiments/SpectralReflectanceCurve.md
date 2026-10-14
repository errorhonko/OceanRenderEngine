# 独立光谱反射率曲线

2026-10-13。此阶段新增 `SpectralReflectanceCurve.h`，在 Visual Studio 的
`头文件/spectrum` 筛选器下。没有修改 `Spectrum` 的 RGB 别名、`SampledSpectrum`、
现有散射模型、回波能量公式或场景材质绑定。

## 对象与单位

这是拥有数据的值类型，不是 `Material` 或 `LidarScatteringModel` 的基类/派生类。
存储一组 `(wavelengthNm, reflectance)` double 样本；构造时复制或移动输入，
只暴露只读访问。无全局缓存、可见光限制、随机状态或外部引擎依赖。

- 波长：有限、正数，单位 nm。
- 反射率：无量纲 rho，范围 [0,1]，不是百分比、RGB、辐射强度或 BRDF。
- 至少两个样本；波长严格递增。重复/乱序/非法数据直接报错，不自动排序、合并或钳制。
- `Evaluate(lambdaNm)` 返回分段线性插值。
- `Contains(lambdaNm)` 查询是否在闭区间内，非法/范围外波长返回 false。
- `MinimumWavelengthNm()`、`MaximumWavelengthNm()`、`SampleCount()`、`Samples()` 提供只读诊断。

区间内：

`t = (lambda - lambda0) / (lambda1 - lambda0)`

`rho = lerp(rho0, rho1, t)`

内部用 `lower_bound` 找区间，查询 O(log N)。端点和原采样波长返回该样本原值。
超出测量频带抛 `std::out_of_range`；不会外推、取最近端点或静默返回 0。
调用方若需要其他行为，应在调用侧明确选策略，避免把无数据误认为吸收材料。

## 使用示例

```cpp
#include "core/spectrum/SpectralReflectanceCurve.h"

const SpectralReflectanceCurve curve({
    {400.0, 0.2},
    {600.0, 0.6},
    {1550.0, 0.8}
});
double rho500 = curve.Evaluate(500.0); // 0.4
double rho905 = curve.Evaluate(905.0); // 约 0.66421，不限制在可见光。
bool available = curve.Contains(2000.0); // false
```

```cpp
// 文件需由调用方提供；此相对路径只是用法示例，不是自动下载入口。
auto measured = SpectralReflectanceCurve::FromAsterFile("data/spectra/asphalt.txt");
double rho = measured.Evaluate(532.0);
const auto& provenance = measured.Metadata();
```

## ASTER / ECOSTRESS 读取

提供 `FromAsterFile(path)` 和 `FromAsterStream(input, sourceName)`。
依据显式 X/Y Units 转换，不像 HELIOS++ 原实现那样固定跳过 26 行。
已验证 HELIOS++ 固定版本所附 10 份真实光谱均可读取。

支持的声明：

- `Wavelength (micrometers/micrometres/um/µm/μm)`：乘 1000 转成 nm。
- `Wavelength (nanometers/nanometres/nm)`：已经是 nm。
- `Reflectance (percent/%)`：乘 0.01 转成 [0,1]。
- `Reflectance (fraction/dimensionless/unitless)`：已经是无量纲反射率。

声明大小写、ASCII 空白变化可接受；未知或缺失单位拒绝。不把 wavenumber、emissivity
当成 wavelength、reflectance，也不凭数值范围猜单位。

文件头长度可以变化；首个数值开头行启动数据表。之前忽略不认识的描述字段/续行；
之后每个非空、非注释行必须恰有两列数字。支持制表符/空格、科学计数、UTF-8 BOM、
CRLF、无末尾换行、整行 # 注释和数据行尾 # 注释；描述字段内的 # 原样保留。
数字解析使用 `from_chars`，不受系统小数逗号 locale 影响。

如果文件声明 `Number of X Values`、`First X Value`、`Last X Value`，则检查与实际表一致；
如果声明 First/Second Column，则要求 X/Y 顺序。重复的受支持头字段拒绝，避免歧义。
范围外反射率、NaN/Inf、缺列/额外列、表内非数值内容、数值或单位转换溢出均拒绝。
数据错误报告源标识与实际行号；文件无法打开、输入流错误明确报错。

`Metadata()` 保存样本名、样本编号、Measurement 行和源文件路径/调用方源标识。
这是基本来源记录，不是完整实验元数据存档：应同时保存原始文件、样本描述和使用许可。

## 验证

- CPU 新增专项 84 项：核心插值与域、输入合法性、单位、头部/表格解析、文件、元数据、拷贝/移动。
- 最终 CPU 全量回归 Release / Debug 各 2073 项通过，零失败、零跳过。
- 自包含头文件用独立小程序编译验证，无 tinyobj 或渲染器链接依赖。
- 普通验收只依赖项目内自造的 `fixtures/spectra/synthetic_reflectance.txt`，不依赖网络或真实外部数据。
- 额外本地验证读取 HELIOS++ commit `2b3585aa766213ae9a46c4790ad4f3594ffa213d` 中全部 10 份光谱，
  按固定 Git blob hash 核对原文件。真实样本只下载到工作区用于检查，不作为项目测试夹具再发布。
- 真实沥青：rho(532)=0.170136、rho(905)=0.2776305、rho(1550)=0.361647，与此前插值核对一致。

## 物理范围与下一阶段

曲线表示单个样本测量的 rho(lambda)，不包含角度散射、极化、透明传播、纹理或海面 Fresnel。
测量可能是特定入射角的方向-半球反射率或其他配置，不能仅凭曲线恢复完整 BRDF。
也不把它缩放/归一化为人眼响应；不能用 RGB Kd 猜近红外反射率。

后续已新增 `SpectralLambertianLidarScattering`，返回 `rho(lambda) / pi`（sr^-1），
并通过既有 `LidarScatteringModel` 接入回波估计与积分接口。详见
[光谱 Lambert 接入记录](SpectralLambertianLidar.md)。原固定反射率模型保持不变。
2026-10-14 已实现手动逐面绑定和雷达分派，详见 [表面材质绑定](SurfaceMaterialBinding.md)。
后续已实现 [OBJ 本地材质表/编号自动转换](ObjMaterialBinding.md)，以及城市可选加载入口。
没有光谱/反射率数据的材料仍使用调用方明确提供的模型，不能当作真实光谱测量。
本阶段没有提交或推送 Git。

## 参考

- [PBRT v4 PiecewiseLinearSpectrum](https://pbr-book.org/4ed/Radiometry,_Spectra,_and_Color/Representing_Spectral_Distributions)：
  借鉴 nm 样本对和分段线性查询；采用 double，并将其范围外返回零改为显式报错。
- [HELIOS++ 光谱读取源码](https://github.com/3dgeo-heidelberg/helios/blob/2b3585aa766213ae9a46c4790ad4f3594ffa213d/src/assetloading/SpectralLibrary.cpp)：
  参考 ASTER 输入与单位，不照搬固定头部行数或缺失数据静默替代。
- [ECOSTRESS 光谱库](https://speclib.jpl.nasa.gov/)：数据具有样本/测量条件，使用时保留来源与引用，
  发布真实样本或衍生数据前核实相应许可。
