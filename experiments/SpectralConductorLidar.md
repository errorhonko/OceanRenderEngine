# 光谱粗糙金属 LiDAR 材质

2026-10-14。阶段 2：在现有 `RoughConductorLidarScattering` 中接入
`SpectralOpticalConstants`，不新增第二个金属 BRDF 或重复实现 Fresnel/GGX/Smith。

## 接口与用法

```cpp
#include "reflection/lidar/RoughConductorLidarScattering.h"

// 人工例子，不代表铜、铝或任何实测金属；波长单位 nm。
SpectralOpticalConstants spectrum(
    {{500.0, 0.15, 2.0}, {905.0, 0.3, 3.0}, {1600.0, 0.6, 4.5}},
    {"synthetic conductor", "example", "not measured data"});
auto metal = std::make_shared<RoughConductorLidarScattering>(
    1.0, spectrum, 0.2f, 0.3f); // etaIncident, spectrum, alphaX, alphaY

ObjMaterialBindingOptions options;
options.overrides["bare_metal"] = std::make_shared<const SurfaceMaterial>(
    SurfaceMaterial{nullptr, metal});
// 之后继续使用 ObjSceneAsset、SurfaceLidarScattering 和原回波链路。
```

- 新构造函数按值接收光谱表：左值复制、右值移动，材质拥有数据，逐面绑定共享材质。
- 既有固定波长构造函数保持可用；其他波长仍明确报错，不隐式当作恒定全波段材料。
- 两个 FromRoughness 重载都复用同一 roughness→alpha 转换，alpha 不小于 1e-3。
- 内部 `std::variant` 保存固定参数或光谱表，避免互相矛盾的布尔标志和空闲参数。
- `IsSpectral()` 返回模式；`OpticalConstantsSpectrum()` 返回只读表指针，固定模式返回 nullptr。
- `OpticalConstantsAtWavelength(double)` 在两种模式下都可查询，返回 n/k。
- EtaConductor()/KappaConductor()/WavelengthNm() 仍可在固定模式使用；光谱模式抛出
  logic_error，不能返回某个未经调用方指定波长的 n/k。为此这三个 getter 不再 noexcept。

## 求值顺序与物理量

1. 检查查询波长，查询固定参数或光谱 n/k。频带外拒绝，包括几何背向请求。
2. 保留原方向归一化、几何半球检查、着色法线和 dpdu 切线坐标系。
3. 将世界方向转为局部方向，调用现有 EvaluateMicrofacetConductorReflection。
4. 返回纯 BRDF，单位 sr^-1；接收余弦、立体角、光学效率和 1/N 仍由原链路计算。

n/k 是绝对复折射率，公共 Fresnel 根据固定的正实数 etaIncident 转为相对值。
本步只让 n/k 随波长变化，不让 GGX alpha 或入射介质折射率随波长变化。
材质表查询只读，不缓存最后查询值，不读文件，不把 RGB 通道当激光波长。

## 验证

新增 RunSpectralConductorLidarAcceptanceTests，与原 72 项导体检查放在同一测试源文件。
新增 81 项检查通过，包括：

- 500/532/600/905/1000/1550/1600 nm 的法线入射解析值、固定模式对照、互易性和方向缩放。
- 独立 n/k 线性插值对照、入射介质、两种模式诊断接口、复制/移动与所有权。
- 波长/alpha/eta 非法输入、频带外查询、退化法线和非吸收介质极限。
- OBJ 逐面 override 到实际求交与回波；905/1550 nm 的解析能量、1/N、到达时间和波形守恒。

实际 x64 Debug/Release CPU 验收各 2488 项通过，零失败、零跳过；SDK 10.0.26100.0，
默认 manifest。按本次独立测试项目构建日志中产生的可执行文件运行；未运行 GPU 验收。

## 下一步与限制

后续已加入[独立光学常数读取器及 Rakić 1995 文献铝数据](OpticalConstantsTableReader.md)，
频带覆盖 905/1550 nm。粗糙度仍独立指定，没有真实表面标定；不对频带外进行外推。
OBJ/MTL 自动转换仍为 Lambert 基线；金属光谱由显式 override 绑定。
本模型不是车漆、逆反射膜、薄膜金属，也不包含偏振、透明折射或多跳传播。
