# 旋转多线 LiDAR 基线

实现文件：`LidarRotaryScanPlan.h`。它实现 `LidarScanPlan`，可直接传给通用 `LidarScanDriver`；无需修改回波、电子接收、测距算法。

## 参数和时序

- `channels`：通道表，支持非均匀俯仰角、方位角偏移、发射时间偏移和局部原点偏移。
- `azimuthSamplesPerRevolution`：每个通道一圈的水平采样数 N。
- `rotationFrequencyHz`：恒定旋转频率 f，单位 Hz。
- `startAzimuthRadians`：圈起点方位角，单位 rad。
- `startTimeSeconds`：相对仿真时钟的圈起点时间，单位 s。
- `firstPulseId`：本圈第一条实际测量的 ID；后续 ID 按事件输出顺序递增。

一圈时长 T=1/f，相邻水平采样列间隔 dt=1/(f N)。通道 c 在第 j 列的发射时间：

`t(j,c) = startTime + j*dt + channel.firingOffsetSeconds`

发射偏移必须位于 [0,dt)，每列内部按偏移升序排列，同偏移保持配置顺序；默认 0 表示各通道同时发射。实际方位角：

`phi(j,c) = startAzimuth + 2*pi*j/N + 2*pi*f*offset + channel.azimuthOffsetRadians`

方向使用 `(cos(theta)*sin(phi), sin(theta), cos(theta)*cos(phi))`。局部 Y 是旋转轴，正方位由局部 +Z 转向 +X；世界旋转轴由安装位姿的 up 方向决定。角度基于相对时序计算，不用大时刻相减；同时检查 double 是否能区分计划中的时间偏移。只采样一圈内的 N 列，不重复圈末端方向。

## 四线示例

```cpp
constexpr double pi = 3.14159265358979323846;
LidarRotaryScanConfig scan;
scan.channels = {
    {0, -15*pi/180},
    {1,  -5*pi/180},
    {2,   5*pi/180},
    {3,  15*pi/180}
};
scan.rotationFrequencyHz = 10;
scan.azimuthSamplesPerRevolution = 1024;
scan.startTimeSeconds = 0;
scan.firstPulseId = 0;
LidarRotaryScanPlan plan(scan);
// pose 定义传感器安装位置和朝向。
// config 的波形、噪声窗、搜索窗等必须先按 LidarScanDriver.md 配置。
LidarScanDriver driver(world, scattering, profile, plan, pose, config);
auto frame = driver.SimulateFrame();
```

该示例一圈 0.1 s，有 4*1024=4096 次测量机会，不保证 4096 个有效点。理论事件率为 40960/s，并不表示本 CPU 波形模型已达到此实时吞吐量。光斑内 MC 样本数不增加测量次数，也不增加最终点数。

下一圈应将 startTime 增加 T，将 firstPulseId 增加 plan.Size()，以免跨帧重用脉冲 ID/seed。当前每个 plan 表示一个有限圈，未实现持续流式发送。

## 模型边界

- 理想恒速多通道计划，不是某款设备的标定复现。
- 发射方向和时间可同时或错时；各通道仍独立接收，不模拟串扰。
- 驱动仍采用固定场景、固定安装位姿，未实现移动平台或动态目标。
- 局部原点偏移在当前共址模型中同时作为收发原点；不能表示不同发射孔径共享另一接收孔径的双站几何。
- 每个通道每次发射至多一个测量点；channelId 保存在脉冲记录中，尚未增加到独立点字段或 ROS 输出。
- 搜索距离受波形记录和时间窗约束，并不会因为扫描覆盖 360 度就自动改变。几何范围与波形检出范围不是同一个概念。

## 验收

`LidarRotaryScanAcceptance.cpp` 使用四通道、四水平列的一圈计划，共 16 次测量。真实前方三角形平面仅被四束正向光照到，完整追踪/波形/ADC/测距链路产生四个有效点，其余事件仍保留为漏检记录。无噪声验收的距离及平面位置误差容限为 5 cm。

另验证四分之一/半圈方位、固定俯仰角、通道排序、错时角度推进、ID 与通道关联、重放、圈末去重、非法参数及时间舍入边界。

后续独立步骤：点云导出（含通道和逐点时间）、平台位姿轨迹、扫描畸变与去畸变、真实通道表标定。
