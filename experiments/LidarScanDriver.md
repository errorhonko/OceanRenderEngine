# 整帧 LiDAR 扫描基线

## 已实现的调用关系

`LidarScanPlan -> LidarFiringEvent -> 固定安装位姿 -> LaserEmitter/LidarReceiver -> LidarIntegrator -> LidarWaveform -> LidarReceiverPipeline -> LidarPointCloudBuilder`

`LidarScanDriver::SimulateFrame()` 按发射顺序返回：

- `pulses`：所有脉冲的扫描角度、元数据、接收诊断、光学能量及可选点。
- `points`：仅通过测距和点云筛选的测量点，顺序与有效脉冲对应。

漏检、削顶或校正失败不会被转换为原点处的点。无几何回波的脉冲仍运行电子接收模型；有噪声时仍有误检的可能。

## 模型边界

固定场景、固定发射位置；接收器与发射器共址，接收光轴与光束中心共同转动。每次脉冲至多生成一个点，不拆分多个回波。点的位置由估计距离和光束中心方向构造，不强制吸附到真实几何上。

场景、散射模型和脉冲轮廓由调用方拥有，必须保持存活，不能传入立即销毁的临时对象。驱动保存配置副本。驱动不更新海面，不实现平台运动、机械回扫、距离模糊、跨脉冲串扰或接收电路连续状态。即使记录窗口相互重叠，当前也仍按独立记录模拟；不是物理串扰模拟。

## 时间配置

`scan.startTimeSeconds` 是相对仿真时钟的起始秒数。驱动配置中的 `waveform.startTimeSeconds`、`reception.noiseWindow` 和 `reception.searchWindow` 都是相对各次发射的时间偏移。驱动在调用底层之前统一加上该脉冲的发射时刻。底层波形及 Pipeline 的原有时间语义不变。

两个接收窗口必须位于波形记录内且互不重叠，噪声窗口至少包含两个箱中心。噪声窗口是否真正避开回波需由调用方根据场景设计，驱动不会使用几何真值自动选窗。过大的时钟无法分辨纳秒时间箱时会拒绝配置。

## 最小调用示例

```cpp
// world 已构建；示例对应距离约 10 m 的平面验收场景。
LambertianLidarScattering scattering(0.5);
GaussianPulseProfile profile(4e-9);
LidarScanDriverConfig config;
config.scan.originWorld = {0, 10, 0};
config.scan.azimuthCount = 4;
config.scan.elevationCount = 3;
config.scan.startTimeSeconds = 1.00000001;
config.scan.pulseIntervalSeconds = 1e-4;
config.pulseEnergyJ = 1e-8f;
config.samplesPerPulse = 5;
config.reception.noiseSigmaVoltageV = 0;
config.reception.adcBitCount = 16;
config.reception.noiseWindow = {0, 20e-9};
config.reception.searchWindow = {40e-9, 115e-9};
config.reception.rangeMethod = LidarReceiverRangeMethod::Centroid;
config.reception.calibration = LidarRangeCalibration(2e-9);
config.rangeSource = LidarPointRangeSource::Corrected;
LidarScanDriver driver(world, scattering, profile, config);
auto frame = driver.SimulateFrame();
// frame.pulses 保留全部扫描记录，frame.points 收集有效点。
auto replay = driver.SimulatePulse(7); // 可独立重放一个脉冲。
```

示例的 2 ns 校正是该验收用接收响应的基线设置，不代表所有脉宽、算法和时间窗口都能用此校正精确消除偏差。实际使用应提前独立标定，扫描时不利用场景真值重新拟合。

## 随机数与能量

由两个全局 seed 与 pulseId 分别派生射线及噪声 seed，并在结果中保存。同一脉冲重放不依赖前面脉冲消耗的样本数。在同一工具链中可复现，不承诺不同标准库的随机分布逐位一致，也不保证有限的 32 位射线 seed 永无碰撞。

`LidarIntegrator` 已完成 MC 的 `1/N` 权重；驱动合成波形时不重复平均。`receivedEnergyJ` 为积分器返回能量之和，`recordedEnergyJ` 为有限记录窗口实际收集的能量；二者不一定相等。

## 验收

`LidarScanDriverAcceptance.cpp` 用实际三角形平面执行几何追踪、脉冲合成、电子接收、ADC、测距、校正和构点。4×3 扫描在无噪声条件下应生成 12 个点，单脉冲 5 条 MC 射线不变成 5 个点。平面距离和高度误差容限为 5 cm（该参数组合的链路验收容限，不是所有场景的精度保证）。

覆盖空场景、半边平面、零反射率、削顶策略、蛇形扫描、有限发散光束、噪声重放、非零时刻及记录偏移、配置验证。

下一阶段：增加旋转多线计划；点云导出、真实海面扫描、平台运动分别验收。

## 通用计划入口

`LidarScanPlan` 提供 `Size()` 和 `Firing(index)`。`LidarScanSequence` 实现矩形计划，原来的 `Sample(index)` 世界坐标接口仍保留。

`LidarFiringEvent` 包含 pulseId、channelId、double 发射时间，以及传感器局部原点和方向。局部轴沿用 X 右、Y 上、Z 前，位置 m，时间 s。`raster` 为可选行列/角度诊断，不要求所有计划都有矩形索引。矩形模型只有一个物理通道，俯仰行不是不同激光通道。

通用构造：

```cpp
LidarScanSequenceConfig raster;
raster.azimuthCount = 4;
raster.elevationCount = 3;
raster.startTimeSeconds = 1.00000001;
LidarScanSequence plan(raster);
LidarScanPoseConfig pose;
pose.originWorld = {0,10,0};
// config 的光学/接收参数按前面的平面示例设置。
LidarScanDriver driver(world, scattering, profile, plan, pose, config);
auto frame = driver.SimulateFrame();
```

此入口忽略 `config.scan`，扫描计划与安装位姿由 plan / pose 显式提供。`config.scan` 仅用于旧的四参数矩形快捷构造。安装位姿独立于局部方向，是固定刚体变换，不包含平台运动或 ROS 坐标适配。

计划必须非空、pulseId 唯一、时间非递减；不同通道允许同一时刻发射，ID 不要求随时间递增。驱动构造时保存快照，因此源计划后续修改或销毁不影响重放。快照占用 O(发射次数) 内存，暂不是无限流调度器。world、scattering、profile 仍须比驱动存活更久。

结果中的 `pulse.scan.firing` 为原始局部事件，`pulse.scan.pulse` 为实际追踪和构点使用的世界坐标元数据。矩形列索引改为 `pulse.scan.firing.raster->azimuthIndex`，先判断 raster 是否存在。channelId 当前保存在脉冲记录中，尚未加入独立点字段或 ROS 输出。

新增 `LidarScanPlanAcceptance.cpp`，验证自定义非矩形计划的真实平面链路、同时发射、通道保留、计划快照、坐标变换、非法数据拒绝和矩形快捷入口兼容。未增加旋转扫描、运动补偿或跨通道串扰。
