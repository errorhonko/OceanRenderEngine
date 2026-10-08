# 高度排序白冠代理基线

这是空间分配代理，不是实际破碎检测器。MonahanWhitecapCoverage 给出平均覆盖率；OceanWhitecapField 选取最高的等水平面积单元，最后一个单元使用部分覆盖。平面没有波峰时实际覆盖率为零，TargetCoverage 保留请求值，HasHeightContrast 为 false。

位置与 OceanSurfaceMesh 相同，X/Z 范围为 [-L/2,L/2)，周期延拓。不要把此覆盖场用于其他物体或已平移/旋转的海面而不转换坐标。高度场与覆盖场的 N、L 必须相同；当前 Update 只能检查 N，L 由调用方确保。

## 接入现有 LiDAR

```cpp
#include "OceanWhitecapField.h"
#include "WhitecapLidarScattering.h"
#include "OceanBeckmannLidarScattering.h"

// frequency、heightField、oceanMesh、world、emitter、receiver 已构造。
auto whitecaps = std::make_shared<OceanWhitecapField>(
    frequency.Resolution(), frequency.PatchLength());
heightField.Update(timeSeconds);
oceanMesh.Update(heightField);
world.Refit(); // world 为已有 TriangleMeshAggregate
whitecaps->Update(heightField, windSpeed10m);
auto water = std::make_shared<OceanBeckmannLidarScattering>(
    unresolvedSlopes, 1.0, 1.33);
// 0.6 仅是示例参数，不是测量得到的通用泡沫反射率。
WhitecapLidarScattering scattering(water, whitecaps, 0.6);
LidarIntegrator lidar(world, emitter, receiver, samplesPerPulse, scattering);
auto pulse = lidar.SimulatePulse(sampler);
```

scattering 必须比 lidar 活得久（现有积分器保存引用）。覆盖场及水面模型由包装类共享持有。每个慢时间先更新高度、网格/BVH与覆盖场，然后冻结这些数据执行整次脉冲；不要在并发射线查询时 Update。它没有自己的泡沫寿命或输运，每次更新都会重新排名。

局部 BRDF 为 (1-W)fWater+W rhoFoam/pi；不再另乘 cosine 或接收器权重，不修改 BeckmannDistribution。几何法线负责正面判断，W=0 恢复正面水面，W=1 恢复正面 Lambert 泡沫。rhoFoam 为调用方提供的目标波长反射率，固定参数不是完整光谱模型。

守恒口径是水平投影面积比例，不是坡面实际面积。排序和面积检查只能证明实现符合此定义，不能验证破碎位置或泡沫光学准确性。当前散射包装只接入 LiDAR 的 Evaluate 接口，不提供普通 PathIntegrator 的 BSDF 采样。

## 后续实验

Hulin 2025：先复现二维单向聚焦波包的线性等效破碎指标，再验证随机相位与三维 Elfouhaily 推广；破碎源与泡沫面积、输运、寿命分开建模。保留本基线作为空间及回波统计对照，不将逐帧覆盖率匹配当成物理破碎阈值的标定。
