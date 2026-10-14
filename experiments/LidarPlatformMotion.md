# 平台位姿轨迹：第一版

## 接口

LidarPoseTrajectory 保存带 double 时间戳的 LidarPoseKeyframe。每个关键帧是传感器在世界坐标中的位置、前向和上向，不直接输入欧拉角。不修改现有世界 Y 向上、局部 X 右 Y 上 Z 前的约定。

位置按时间线性插值；姿态先转换为单位四元数，再作最短弧 SLERP，近似重合时使用归一化线性插值。关键帧时间必须严格递增、有限且非负。所有姿态必须可构成有效正交坐标基。超出覆盖范围抛异常，不隐式外推。一个关键帧仅覆盖它的那个时刻，不能代替无限固定轨迹。

这不是动力学模型，没有速度/加速度约束、IMU 输出或样条平滑；关键帧处速度可以不连续。当前轨迹直接描述传感器，车体轨迹乘安装外参尚未实现。

## 接入

```cpp
LidarPoseTrajectory motion({
    {1.0, {{0,1.5f,0}, {0,0,1}, {0,1,0}}},
    {1.3, {{0,1.5f,1}, {0.173648f,0,0.984808f}, {0,1,0}}}
});
// plan 的第一帧时间从 1.0 s 开始。
LidarMultiFrameDriver driver(
    world, scattering, profile, plan, motion, config,
    LidarMultiFrameConfig{3, 0.1, 0});
auto frame = driver.SimulateFrame(1);

LidarPcdWriter::Save(worldPath, frame.cloud);
LidarPcdWriter::Save(rawPath, frame.cloud,
    LidarPcdCoordinates::SensorAtEmission);
```

固定 pose 的旧构造仍有效。单帧驱动构造时按每次发射时间查询位姿并保存；多帧驱动保存轨迹副本，生成每帧时再逐脉冲查询。源计划/源轨迹销毁后均可重放。world、scattering、profile 的生命周期要求不变。

每个脉冲都有自己实际使用的 sensorPose，发射位置、方向和共址接收位置都由该位姿确定。不是只更新帧头，不是把整帧都套一个起始位姿。场景本身不自动运动。

## 坐标与输出

- positionWorld：利用发射时刻真值位姿转换的测量点，仍含测距误差，不是直接使用几何真值点。
- positionSensor：该点在发射时刻的传感器局部坐标，未统一到一个参考时刻，因此保留扫描时间差。
- pulse.sensorPose：该脉冲实际使用的位姿。
- cloud.sensorPose：该帧第一条计划发射时刻的参考位姿。

关系为 pWorld = R(t_i) pSensor + translation(t_i)。将不同 t_i 的 pSensor 直接视为一个瞬时点云，会产生运动畸变。positionWorld 使用了已知仿真位姿，不能作为未经定位的传感器输出拿去检验 SLAM。

PCD 默认 World 模式不变，VIEWPOINT 是帧起始参考位姿，点已在世界坐标中，不要重复应用变换。SensorAtEmission 模式将 XYZ 换为逐点局部测量，并使用 identity VIEWPOINT，文件注释明确 sensor_at_emission。它不是去畸变后参考帧点云。

time 仍是发射时刻减帧起点的秒数。原始坐标不具备时拒绝输出，不默默回退世界点。逐脉冲位姿保存在内存记录中，PCD 本身不保存完整运动轨迹；后续去畸变/算法对接还需传递轨迹或同步 IMU，不能只凭一个 VIEWPOINT 恢复运动。

## 物理范围与验收

收发位置在单次脉冲传播期间冻结，只更新相邻脉冲之间的位姿。尚未实现传播过程中接收器运动、动态物体、估计位姿去畸变或跨脉冲接收状态。

LidarMotionAcceptance.cpp 验证位置插值、45 度中间姿态、跨 +/-180 度最短旋转、极小旋转、半圈旋转、时间覆盖、生命周期、真实墙面平移/转向测距、原始坐标变化、世界点墙面一致性、局部/世界往返和两种 PCD 模式。

下一步：实际移动旋转扫描并对比原始/真值位姿转换点云，再单独实现以帧参考时刻为目标的去畸变。
