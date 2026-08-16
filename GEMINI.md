# OceanRenderEngine — 项目背景文件

## 开发者背景

| 项目 | 说明 |
|---|---|
| **当前学习** | 正在系统学习 **PBRT v4**（Physically Based Rendering: From Theory to Implementation），以其架构作为本项目材质系统的设计参考 |
| **最终目标** | 实现一个**海面激光雷达（LiDAR）回波仿真引擎**：模拟激光脉冲打到真实海面（含波浪起伏、泡沫、散射）后的回波能量分布，用于海洋探测领域的物理仿真 |
| **项目定位** | 秋招求职作品集核心项目，展示 C++ 图形学工程能力、物理渲染理论理解深度及从零构建渲染系统的能力 |

> **给 AI 助手的提示**：
> - **不要自动修改项目文件**。用户希望通过自己动手修改来提升能力。收到代码问题时，提供「诊断原因 + before/after 对比代码片段」，由用户决定是否采纳。
> - 用户具备一定的图形学基础，熟悉 PBRT 架构风格。在给出建议时，可直接使用渲染领域术语（BxDF、PDF、MIS、Russian Roulette 等）。
> - 代码建议应与项目已有的 PBRT 风格保持一致（`PascalCase` 函数名、局部坐标系 Z = 法线等）。
> - 优先引导用户朝**激光雷达物理建模**方向推进（波面法线分布 / 菲涅耳反射 / 散射截面），而非通用渲染特效。

---

## 项目概述

**OceanRenderEngine** 是一个从零手写的、基于 CPU 的软件渲染引擎，使用 **C++17** 实现，目标是实现一套完整的光线追踪（Ray Tracing）与光栅化（Rasterization）双渲染管线，并为未来的海洋场景物理渲染（特别是海面 BRDF/散射建模）奠定基础。

该项目是一个 **Visual Studio** 解决方案（`.slnx` / `.vcxproj`），目前为单一 C++ 项目，无任何第三方渲染框架依赖，所有数学、着色、几何逻辑均自行实现。

---

## 技术栈

| 类别 | 技术 |
|---|---|
| 语言 | C++17 |
| 构建系统 | Visual Studio (MSVC) |
| 数学库 | 自实现（`Vector3f`, `Vector4f`, `Matrix4f`, `Point2f`） |
| 图像输出 | PPM 格式（`P6` 二进制） |
| 模型加载 | 自实现 OBJ 解析器（`load_obj`） |
| 纹理读取 | `stb_image.h`（单头文件库） |
| 依赖 | 无外部依赖（仅 STL + stb_image） |

---

## 目录结构与核心模块

### 渲染管线

| 文件 | 职责 |
|---|---|
| `main.cpp` | 程序入口；设置相机/MVP矩阵、加载模型、驱动两条渲染管线 |
| `Rasterizer.h/.cpp` | **光栅化管线**：重心坐标三角形填充、Z-Buffer 深度测试、透视矫正颜色插值、PPM 图像写出 |
| `camera.h/.cpp` | 相机模型 |
| `Ray.h/.cpp` | 光线结构体（原点 + 方向） |

### 场景几何

| 文件 | 职责 |
|---|---|
| `Hittable.h` | 可求交抽象基类接口 |
| `HittableList.h/.cpp` | 场景对象容器，遍历求最近交点 |
| `HitRocord.h/.cpp` | 光线-几何交点信息（位置、法线、t 值） |
| `Sphere.h/.cpp` | 球体几何求交 |
| `Triangle.h/.cpp` | 三角形几何求交 |
| `load_obj.h/.cpp` | OBJ 模型解析器（支持顶点/法线/UV，若模型无法线则自动计算面法线并平滑） |

### 材质与着色（PBRT 风格）

项目的材质系统参考 **PBRT（Physically Based Rendering Techniques）** 架构设计：

| 文件 | 职责 |
|---|---|
| `BxDF.h/.cpp` | `BxDF` 抽象基类，定义 `f()`、`Sample_f()`、`rho()`、`Pdf()` 接口；内联实现菲涅耳公式（`FrDielectric`、`FrConductor`）和反射方向计算（`Reflect`）。`BxDFType` 标志位枚举（漫射/光泽/镜面/反射/透射）。 |
| `BSDF.h/.cpp` | `BSDF` 容器类，管理一个 `BxDF`；负责渲染空间与局部着色坐标系（`Frame`）之间的方向变换 |
| `BRDFUtils.h` | 着色坐标系工具函数命名空间（`CosTheta`, `SinTheta`, `TanTheta`, `CosPhi`, `SinPhi`等）；`TransportMode` 枚举；`Frame` 结构体（局部坐标系，支持 `FromXZ`/`FromZ` 构建） |
| `Lambertian.h/.cpp` | Lambertian 漫反射 BxDF |
| `SpecularReflection.h/.cpp` | 镜面反射 BxDF |
| `SpecularTransmission.h/.cpp` | 镜面透射 BxDF |
| `ScaledBxDF.h` | 带缩放权重的 BxDF 包装器 |
| `Fresnel.h/.cpp` | 菲涅耳效果抽象基类 |
| `FresnelConductor.h/.cpp` | 导体（金属）菲涅耳 |
| `FresnelDielectric.h/.cpp` | 电介质菲涅耳 |
| `FresnelNoOp.h/.cpp` | 无菲涅耳效果（总返回1.0）|
| `material.h/.cpp` | 材质抽象层 |

### 光谱与颜色

| 文件 | 职责 |
|---|---|
| `CoefficientSpectrum.h/.cpp` | 模板基类，存储 N 个光谱系数，实现加减乘除、`Sqrt`、`Lerp`、`IsBlack` 等运算 |
| `RGBSpectrum.h/.cpp` | RGB 三通道光谱，继承 `CoefficientSpectrum<3>` |
| `SampledSpectrum.h/.cpp` | 60 波段（400-700nm）采样光谱；支持 `FromSampled` 从测量数据构建、`TOXYZ` 转 CIE-XYZ、`ToRGB` 转 sRGB（含 XYZ→RGB 矩阵）；内置 CIE 颜色匹配函数的高斯拟合 |

### 数学库

| 文件 | 职责 |
|---|---|
| `Vector3f.h/.cpp` | 三维浮点向量，支持点乘、叉乘、归一化、标量运算 |
| `Vector4f.h/.cpp` | 四维浮点向量 |
| `Point2f.h/.cpp` | 二维浮点点（用于纹理坐标、随机采样） |
| `Matrix4f.h/.cpp` | 4×4 浮点矩阵；静态工厂：`Scale`, `Translate`, `LookAt`, `Perspective`, `RotateX/Y/Z` |

### 纹理

| 文件 | 职责 |
|---|---|
| `Texture.h/.cpp` | 纹理抽象基类 |
| `ImageTexture.h/.cpp` | 图像纹理，使用 `stb_image.h` 加载 |

---

## 渲染管线现状

### 光栅化管线（已实现）
- MVP 矩阵变换（Model → View → Projection → NDC → Screen Space）
- 三角形重心坐标包围盒扫描填充（`DrawTriangle`）
- Z-Buffer 深度测试（NDC 深度 `z_ndc`）
- 透视矫正颜色插值（`inv_w` 技术）
- 输出到 PPM 文件（`WriteToImage`）
- 目前已渲染 Stanford Bunny（`bunny_render.ppm`）

### 光线追踪管线（搭建中）
- `cast_ray` 函数已有基础框架
- 目前法线可视化作为临时着色：`return max(0, normal.z) * 255`
- **TODO**：接入 BSDF/BRDF 着色计算，实现路径追踪

---

## 代码约定

- **命名风格**：类名 `PascalCase`，成员变量 `camelCase`，函数 `PascalCase`（PBRT 风格）
- **文件组织**：每个类声明在 `.h` 中，实现在同名 `.cpp`（即使 `.cpp` 目前可能只有 `#include` 的占位内容）
- **坐标系**：着色局部坐标系中 **Z 轴 = 表面法线方向**（与 PBRT 一致）
- **光谱类型别名**：`using Spectrum = RGBSpectrum;`（可切换为 `SampledSpectrum` 以启用全谱渲染）
- **光线偏置**：`t_min = 0.001f` 防止自交（Shadow Acne）
- **图像格式**：输出为 `.ppm`（P6 二进制），分辨率 800×600

---

## 当前实现进度（2026-07-26）

### 材质系统（BxDF 层）

| 模块 | 状态 |
|---|---|
| `BxDF` 基类 | ✅ v4 风格接口（返回 `optional<BSDFSample>`） |
| `DiffuseBxDF` | ✅ 余弦加权采样，继承 BxDF |
| `SpecularReflection` | ✅ `Sample_f` 签名已更新为新接口 |
| `SpecularTransmission` | 🔲 待接入 `Refract` |
| `BSDF` 容器 | ✅ 坐标系变换，`optional` 解包 |

### 基础设施

| 模块 | 状态 |
|---|---|
| `CoefficientSpectrum` | ✅ 运算符完整，`using namespace std` 已清除，`Sqrt` const 修复 |
| `FrComplex` | ✅ 标量版（`std::complex`）+ 光谱版，替代旧 `FrConductor` |
| `BRDFUtils` | ✅ `SameHemisphere`、同心圆盘采样、余弦 PDF |
| `Refract` | ✅ 已加入 `BxDF.h`（与 `Reflect` 并列） |
| `BxDFReflectionType` | ✅ `enum class` 位运算符已重载（`&`、`\|`、`!`） |

---

## 已知 TODO / 开发方向

1. **海洋场景 BRDF**：项目名称暗示目标是海面渲染。`cast_ray` 中的注释明确预留了"通过 `rec.normal` 调用海面/物体 BRDF 公式来反演回波能量"的接口。
2. **路径追踪积分器**：需要实现完整的路径追踪循环（多次弹射、俄罗斯轮盘赌终止）
3. **BVH 加速结构**：当前 `HittableList` 是线性遍历，需要 BVH 优化复杂场景
4. **多线程渲染**：CPU 软渲染性能瓶颈，可引入 `std::thread` 或 OpenMP
5. **颜色管线**：目前光栅化管线输出灰度色（`unsigned char color`），需扩展为 RGB

---

## 重要注意事项

- OBJ 加载路径为相对路径 `./models/stanford-bunny.obj`，需确保工作目录正确
- `.vcxproj` 为 Visual Studio 项目文件，编译入口为 `main.cpp`
- `stb_image.h` 为第三方单头文件，**不需要修改**
- `SampledSpectrum.h` 中 `CIE_XYZ` 函数定义在头文件全局作用域（非 `inline`），多 `.cpp` 包含时可能引发链接重定义问题，修改时需注意
