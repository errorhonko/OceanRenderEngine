# 项目目录与模块导航

2026-10-14：实体目录按模块整理，主项目 VS 筛选器与目录对应。
仅迁移文件、统一 include/工程路径及编译编码配置；不修改模型算法，不转换为 CMake。

| 目录 | 职责 |
| --- | --- |
| src/app | 原 main.cpp 栅格化入口 |
| src/core | 数学、射线/交点接口、采样、光谱容器 |
| src/geometry | 通用图元、共享顶点网格、包围盒和 BVH |
| src/reflection | 材质/BxDF、Fresnel、微表面、纹理、雷达 BRDF 适配 |
| src/render | 相机、路径积分器、灯光 |
| src/lidar | 发射器、接收器、ADC、信号处理、回波仿真、扫描和点云输出 |
| src/ocean | 海浪谱、频域/FFT/高度场、坡度统计、白冠场 |
| src/scene | 场景组装、OBJ/MTL 导入、静态示例 |
| src/legacy | 旧栅格化器和 OBJ 读取器，保留给现有入口使用 |
| third_party | tinyobjloader 与 stb 原始依赖，不混入核心模块 |
| tests | 原验收工程与 fixtures 原位保留 |
| experiments | 原数值实验、导入示例及文档原位保留 |
| gpu | 已独立的 DX12 代码，未迁移或改动 |

## 包含与编译约定

- 工程增加 src 和仓库根目录作为 include 搜索路径。
- 内部引用用明确模块路径，例如 `#include "reflection/bxdfs/ConductorBxDF.h"`。
- 普通 .h/.cpp 同模块放置，保留现有 header-only 风格。
- 不再增加根目录转发头，也不配置几十个模块目录作为搜索路径。
- Windows 资源 `resource.h`、`.rc` 及 VS 工程文件留在根目录。
- `FresnelConductor.h` 为既有旧接口，现显示在工程中，但其旧 Spectrum 调用尚未迁移；
  本次仅归类，不宣称该接口已经修复或可直接启用。
- `SpecularReflection.cpp` 是仅包含头文件的既有文件，仍不加入编译单元。
- SLAM 尚未实现，不保留会误导进度的空筛选器。

## 依赖与查找原则

目录是职责导航，不代表模块完全无依赖。reflection/lidar 是反射模型适配，不是接收器。
scene/import 的 ObjMaterialBinding 将 MTL 转换为 SurfaceMaterial；geometry 使用绑定后的交点数据。
共用算法先查 FresnelUtils、MicrofacetReflection、BxDF/分布类，避免为雷达重新实现一套。

## 本次迁移清单

| 原位置 | 新位置 |
| --- | --- |
| BeckmannDistribution.h | src/reflection/microfacet/BeckmannDistribution.h |
| Bounds3f.h | src/geometry/acceleration/Bounds3f.h |
| BRDFUtils.h | src/reflection/microfacet/BRDFUtils.h |
| BSDF.h | src/reflection/bxdfs/BSDF.h |
| BVHAccel.h | src/geometry/acceleration/BVHAccel.h |
| BxDF.h | src/reflection/bxdfs/BxDF.h |
| camera.h | src/render/cameras/camera.h |
| CoefficientSpectrum.h | src/core/spectrum/CoefficientSpectrum.h |
| ConductorBxDF.h | src/reflection/bxdfs/ConductorBxDF.h |
| DeltaLidarBeamProfile.h | src/lidar/emitter/beam/DeltaLidarBeamProfile.h |
| DielectricBxDF.h | src/reflection/bxdfs/DielectricBxDF.h |
| DielectricMaterial.h | src/reflection/materials/DielectricMaterial.h |
| DiffuseAreaLight.h | src/render/lights/DiffuseAreaLight.h |
| DiffuseBxDF.h | src/reflection/bxdfs/DiffuseBxDF.h |
| DiffuseMaterial.h | src/reflection/materials/DiffuseMaterial.h |
| DistantLight.h | src/render/lights/DistantLight.h |
| ElfouhailySpectrum.cpp | src/ocean/spectrum/ElfouhailySpectrum.cpp |
| ElfouhailySpectrum.h | src/ocean/spectrum/ElfouhailySpectrum.h |
| FirstOrderLidarResponse.h | src/lidar/receiver/analog/FirstOrderLidarResponse.h |
| Fresnel.h | src/reflection/fresnel/Fresnel.h |
| FresnelConductor.h | src/reflection/fresnel/FresnelConductor.h |
| FresnelDielectric.h | src/reflection/fresnel/FresnelDielectric.h |
| FresnelNoOp.h | src/reflection/fresnel/FresnelNoOp.h |
| FresnelUtils.h | src/reflection/fresnel/FresnelUtils.h |
| GaussianLidarBeamProfile.h | src/lidar/emitter/beam/GaussianLidarBeamProfile.h |
| GaussianPulseProfile.h | src/lidar/emitter/pulse/GaussianPulseProfile.h |
| HashUtils.h | src/core/math/HashUtils.h |
| HeliosUrbanStaticScene.h | src/scene/examples/HeliosUrbanStaticScene.h |
| HeliosUrbanStaticSceneParts.h | src/scene/examples/HeliosUrbanStaticSceneParts.h |
| HitRocord.h | src/core/ray/HitRocord.h |
| Hittable.h | src/core/ray/Hittable.h |
| HittableList.h | src/core/ray/HittableList.h |
| IdealTransimpedanceAmplifier.h | src/lidar/receiver/analog/IdealTransimpedanceAmplifier.h |
| IdealUniformAdc.h | src/lidar/receiver/adc/IdealUniformAdc.h |
| ImageTexture.h | src/reflection/textures/ImageTexture.h |
| IndependentSampler.h | src/core/sampling/IndependentSampler.h |
| Integrator.h | src/render/integrators/Integrator.h |
| Lambertian.h | src/reflection/materials/Lambertian.h |
| LambertianLidarScattering.h | src/reflection/lidar/LambertianLidarScattering.h |
| LaserEmitter.h | src/lidar/emitter/LaserEmitter.h |
| LidarAnalogWaveform.h | src/lidar/receiver/analog/LidarAnalogWaveform.h |
| LidarBeamProfile.h | src/lidar/emitter/beam/LidarBeamProfile.h |
| LidarDigitalBaselineEstimator.h | src/lidar/signal_processing/LidarDigitalBaselineEstimator.h |
| LidarDigitalMultiEchoEstimator.h | src/lidar/signal_processing/LidarDigitalMultiEchoEstimator.h |
| LidarDigitalPeakDetector.h | src/lidar/signal_processing/LidarDigitalPeakDetector.h |
| LidarDigitalRangeEstimator.h | src/lidar/signal_processing/LidarDigitalRangeEstimator.h |
| LidarDigitalWaveform.h | src/lidar/receiver/adc/LidarDigitalWaveform.h |
| LidarIntegrator.h | src/lidar/simulation/LidarIntegrator.h |
| LidarMultiFrameDriver.h | src/lidar/scan/LidarMultiFrameDriver.h |
| LidarPcdWriter.h | src/lidar/output/LidarPcdWriter.h |
| LidarPointCloudBuilder.h | src/lidar/output/LidarPointCloudBuilder.h |
| LidarPointCloudFrame.h | src/lidar/output/LidarPointCloudFrame.h |
| LidarPointSample.h | src/lidar/output/LidarPointSample.h |
| LidarPoseTrajectory.h | src/lidar/scan/LidarPoseTrajectory.h |
| LidarPulseMeasurement.h | src/lidar/simulation/LidarPulseMeasurement.h |
| LidarPulseMetadata.h | src/lidar/simulation/LidarPulseMetadata.h |
| LidarPulseProfile.h | src/lidar/emitter/pulse/LidarPulseProfile.h |
| LidarRangeCalibration.h | src/lidar/signal_processing/LidarRangeCalibration.h |
| LidarRangeEstimator.h | src/lidar/signal_processing/LidarRangeEstimator.h |
| LidarReceiver.h | src/lidar/receiver/LidarReceiver.h |
| LidarReceiverPipeline.h | src/lidar/receiver/LidarReceiverPipeline.h |
| LidarReturnEstimator.h | src/lidar/simulation/LidarReturnEstimator.h |
| LidarReturnSelection.h | src/lidar/signal_processing/LidarReturnSelection.h |
| LidarRotaryScanPlan.h | src/lidar/scan/LidarRotaryScanPlan.h |
| LidarScanDriver.h | src/lidar/scan/LidarScanDriver.h |
| LidarScanPlan.h | src/lidar/scan/LidarScanPlan.h |
| LidarScanSequence.h | src/lidar/scan/LidarScanSequence.h |
| LidarScatteringModel.h | src/reflection/lidar/LidarScatteringModel.h |
| LidarStaticTestScene.h | src/scene/examples/LidarStaticTestScene.h |
| LidarVoltageNoise.h | src/lidar/receiver/analog/LidarVoltageNoise.h |
| LidarVoltageWaveform.h | src/lidar/receiver/analog/LidarVoltageWaveform.h |
| LidarWaveform.h | src/lidar/simulation/LidarWaveform.h |
| Light.h | src/render/lights/Light.h |
| LinearPhotodetector.h | src/lidar/receiver/analog/LinearPhotodetector.h |
| load_obj.h | src/legacy/obj/load_obj.h |
| main.cpp | src/app/main.cpp |
| material.h | src/reflection/materials/material.h |
| Matrix4f.h | src/core/math/Matrix4f.h |
| MeshTriangle.h | src/geometry/mesh/MeshTriangle.h |
| MicrofacetReflection.h | src/reflection/microfacet/MicrofacetReflection.h |
| MixMaterial.h | src/reflection/materials/MixMaterial.h |
| ObjMaterialBinding.h | src/scene/import/ObjMaterialBinding.h |
| ObjMeshLoader.h | src/scene/import/ObjMeshLoader.h |
| ObjSceneAsset.h | src/scene/import/ObjSceneAsset.h |
| OceanBeckmannLidarScattering.h | src/reflection/lidar/OceanBeckmannLidarScattering.h |
| OceanFFT.cpp | src/ocean/field/OceanFFT.cpp |
| OceanFFT.h | src/ocean/field/OceanFFT.h |
| OceanFrequencyField.cpp | src/ocean/field/OceanFrequencyField.cpp |
| OceanFrequencyField.h | src/ocean/field/OceanFrequencyField.h |
| OceanHeightField.cpp | src/ocean/field/OceanHeightField.cpp |
| OceanHeightField.h | src/ocean/field/OceanHeightField.h |
| OceanNormalDistribution.h | src/ocean/statistics/OceanNormalDistribution.h |
| OceanSlopeNormalDistribution.h | src/ocean/statistics/OceanSlopeNormalDistribution.h |
| OceanSlopeVariance.h | src/ocean/statistics/OceanSlopeVariance.h |
| OceanSurfaceMesh.cpp | src/ocean/field/OceanSurfaceMesh.cpp |
| OceanSurfaceMesh.h | src/ocean/field/OceanSurfaceMesh.h |
| OceanWhitecapField.h | src/ocean/whitecap/OceanWhitecapField.h |
| PathIntegrator.h | src/render/integrators/PathIntegrator.h |
| PhotonCountingLidarDetector.h | src/lidar/receiver/photon_counting/PhotonCountingLidarDetector.h |
| Point2f.h | src/core/math/Point2f.h |
| PointLight.h | src/render/lights/PointLight.h |
| Rasterizer.h | src/legacy/rasterizer/Rasterizer.h |
| Ray.h | src/core/ray/Ray.h |
| RGBSpectrum.h | src/core/spectrum/RGBSpectrum.h |
| RoughConductorLidarScattering.h | src/reflection/lidar/RoughConductorLidarScattering.h |
| RoughDielectricLidarScattering.h | src/reflection/lidar/RoughDielectricLidarScattering.h |
| SampledSpectrum.h | src/core/spectrum/SampledSpectrum.h |
| Sampler.h | src/core/sampling/Sampler.h |
| ScaledBxDF.h | src/reflection/bxdfs/ScaledBxDF.h |
| SceneBuilder.h | src/scene/SceneBuilder.h |
| SimplePathIntegrator.h | src/render/integrators/SimplePathIntegrator.h |
| SpectralLambertianLidarScattering.h | src/reflection/lidar/SpectralLambertianLidarScattering.h |
| SpectralReflectanceCurve.h | src/core/spectrum/SpectralReflectanceCurve.h |
| SpecularReflection.cpp | src/reflection/bxdfs/SpecularReflection.cpp |
| SpecularReflection.h | src/reflection/bxdfs/SpecularReflection.h |
| SpecularTransmission.h | src/reflection/bxdfs/SpecularTransmission.h |
| Sphere.h | src/geometry/primitives/Sphere.h |
| StaticScene.h | src/scene/StaticScene.h |
| stb_image.h | third_party/stb/stb_image.h |
| Stratified2DSampler.h | src/core/sampling/Stratified2DSampler.h |
| SurfaceLidarScattering.h | src/reflection/lidar/SurfaceLidarScattering.h |
| SurfaceMaterial.h | src/reflection/materials/SurfaceMaterial.h |
| Texture.h | src/reflection/textures/Texture.h |
| ThinDielectricBxDF.h | src/reflection/bxdfs/ThinDielectricBxDF.h |
| TinyObjLoaderImplementation.cpp | src/scene/import/TinyObjLoaderImplementation.cpp |
| Triangle.h | src/geometry/primitives/Triangle.h |
| TriangleMesh.h | src/geometry/mesh/TriangleMesh.h |
| TriangleMeshAggregate.h | src/geometry/mesh/TriangleMeshAggregate.h |
| TrowbridgeReitzDistribution.h | src/reflection/microfacet/TrowbridgeReitzDistribution.h |
| UniformConeLidarBeamProfile.h | src/lidar/emitter/beam/UniformConeLidarBeamProfile.h |
| UniformInfiniteLight.h | src/render/lights/UniformInfiniteLight.h |
| UniformLightSampler.h | src/render/lights/UniformLightSampler.h |
| Vector3f.h | src/core/math/Vector3f.h |
| Vector4f.h | src/core/math/Vector4f.h |
| WhitecapCoverage.h | src/ocean/whitecap/WhitecapCoverage.h |
| WhitecapLidarScattering.h | src/reflection/lidar/WhitecapLidarScattering.h |

迁移前的当前内容备份位于本地 Codex 工作区的
`output/project-organization/backup-20261014-150825`，包含当时未提交的修改，未放入仓库。

## 整理后验证

- 136 个文件完成移动；根目录仅保留 Windows 资源头 `resource.h`。
- 对照迁移前备份，136 个搬迁文件及受影响测试的非 include 正文保持不变。
- 6 个 VS 工程引用、主工程/CPU 测试筛选器对应关系及扫描到的全部引号 include 引用通过检查。
- CPU Release / Debug 全量各 2300 项通过，零失败、零跳过。
- 固定 SDK 后，实际解决方案 x64 Debug/Release 均重建通过，默认清单生成正常；
  新生成的 CPU 程序各通过 2300 项，RX 7900 XTX 上硬件 GPU 验收也通过。
  两个海面实验工程 Debug/Release 编译通过，未重跑完整数值研究。
- 主工程及 CPU 测试工程在四种 Debug/Release、Win32/x64 配置中统一 `/utf-8`，
  避免 UTF-8 中文注释被默认 GBK 误读导致类型声明消失；资源脚本原编码不改。
- 目录迁移时曾遇到 SDK 28000 缺少 `MidlrtMd.dll` 导致的 LNK1327。
  现已将六个工程固定到 `10.0.26100.0`，无需关闭清单生成，构建错误已绕开。
  未修改 SDK 安装或复制 DLL；详情见 [SDK 验证记录](Sdk26100Verification.md)。
- 文件迁移随版本检查点提交；未删除旧算法、模型、测试资源或用户修改。本地备份不随仓库发布。
