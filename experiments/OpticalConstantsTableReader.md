# 光学常数读取与首份文献铝数据

2026-10-14。阶段 2 的数据接入小步；复用已完成的容器、插值和金属 BRDF。

## 结构与读取

- `src/core/spectrum/OpticalConstantsTableReader.h`：独立的三列文本读取器。
- `data/materials/optical_constants/Al_Rakic1995_um.txt`：26 行连续波段子表。
- `third_party/refractiveindex-info/Al/Rakic.yml`：固定版本原始快照，含 CC0 说明。
- `tests/OceanRenderEngine.Tests/OpticalConstantsTableReaderAcceptance.cpp`：读取及数据链路验收。

```cpp
#include "core/spectrum/OpticalConstantsTableReader.h"
#include "reflection/lidar/RoughConductorLidarScattering.h"

const auto spectrum = OpticalConstantsTableReader::FromFile(
    "D:/sunmmer_camp/computerGraphics/OceanRenderEngine/data/materials/optical_constants/Al_Rakic1995_um.txt",
    OpticalConstantsWavelengthUnit::Micrometers,
    {"Al Rakic 1995", "https://doi.org/10.1364/AO.34.004755",
     "Intrinsic constants reconstructed from evaporated-Al film data; roughness unspecified"});
auto metal = std::make_shared<RoughConductorLidarScattering>(
    1.0, spectrum, 0.2f, 0.3f); // alpha 仍为独立的假设参数。
// 继续通过 SurfaceMaterial / OBJ override 绑定。
```

读取器要求显式指定 nm 或 um。FromStream 与 FromFile 都可用；不推测单位、不解析
任意标题行、不排序、不钳制；UTF-8 BOM、空白行和 # 注释可用。有效数据行必须恰好
三个有限数值，支持科学计数法。最终调用 SpectralOpticalConstants 的校验规则，
不在读取器中另写一套物理验证。文件 I/O 只发生在构造数据时，不在逐光线求值时发生。

数据默认 provenance 为源路径/流名；调用方给出的 DOI、名称与样品条件会被保留。
这不是 YAML、CSV、ASTER 或分离 n/k 两表的通用导入器，也不解析介电函数/能量列。

## 数据来源与范围

文献：A. D. Rakic, Applied Optics 34, 4755-4767 (1995),
https://doi.org/10.1364/AO.34.004755
其分析光学及电子能量损失资料，重建蒸镀铝膜的本征光学常数，不能称为任意实际
铝件的实测 BRDF。论文摘要已核对；没有声称独立复现其 Kramers-Kronig 重建算法。

数据库使用 CC0，固定 revision 37773dce8128ccabcd9d5108d9958d6041244661。
原始 UTF-8/LF SHA-256 为 e93b31fe821551836e5b000949c67c633b8a1ca1f2c5549f4db0f1c03b24844d。
工作表连续截取 413.28-2479.7 nm 的 26 行；已逐行确认数值 token 与源表一致，
没有预先重采样，频带外仍报错。µm→nm 仅在读取时转换一次，n/k 不缩放。

在当前分段线性查询下，空气入射的参考值为：

| 波长 nm | n | k | 法线 Fresnel F0 |
| --- | --- | --- | --- |
| 905 | 2.0522093282 | 8.2566105950 | 0.8940625690 |
| 1550 | 1.5785092141 | 15.6581093044 | 0.9749268959 |

这些查询值由相邻源节点插值，并非论文在两个指定波长的独立直接测量。
不包含表面粗糙度、氧化层、涂层、合金差异或未知温度状态。

## 验收与局限

新增 62 项检查全部通过：显式单位转换、BOM/注释、严格列数/数值、非法样本、
来源信息、文件缺失、固定表节点和频带、905/1550 nm 独立源节点插值、Fresnel
法线解析值、固定参数模型角度对照、OBJ 材质绑定到回波/1/N/时间/波形守恒。

CPU x64 Debug/Release 各 2550 项通过，零失败、零跳过；SDK 10.0.26100.0、默认
manifest，运行本次构建日志中的新可执行文件。未运行 GPU 验收。
首轮新增测试的 C++20 已弃用路径接口已改为 char8_t UTF-8 路径构造，不屏蔽诊断。

这是文献数据加载与现有公式链路验证，不是实际铝表面散射模型标定；试验 alpha
仍为假设，也没有相对实际接收波形进行拟合。阶段 2 基线就绪，下一功能阶段是
涂层漫反射模型。若要现实参数验证，应另补加工状态/粗糙度/角度测量与仪器响应。
