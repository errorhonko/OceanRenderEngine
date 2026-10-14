# 通用点云输出与连续多帧

后续已加入可选位姿轨迹与逐点原始传感器坐标，见 LidarPlatformMotion.md。以下固定场景例子仍作为回归基线；世界坐标输出不是原始传感器坐标。

## 格式选择

本版提供 PCD 0.7 ASCII，是 PCL 原生点云文件格式，不依赖 PCL 编译库。遵循官方头部字段与顺序：
https://pointclouds.org/documentation/tutorials/pcd_file_format.html

ROS 的实时点云接口是 sensor_msgs/PointCloud2，不是 PCD 文件：
https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg

当前只做离线文件和通用字段，不宣称已接入 ROS、Isaac Sim 或所有软件。具体消费者的 ring/time 类型和单位仍需适配。ASCII 便于验收，尚未提供 binary/binary_compressed。

## 类与调用方式

- LidarPointCloudFrame：帧 ID、起止时间、固定安装位姿、有效测量点。
- LidarMultiFrameDriver：快照并重复任意有限 LidarScanPlan，按帧平移时间和脉冲 ID。
- LidarPcdWriter：Write(ostream, frame) 或 Save(path, frame, overwrite=false)。
- LidarPointSample 增加 channelId；单帧驱动把计划通道编号写入独立点。

示例（world/model/profile/plan/pose/config 已构造）：

```cpp
LidarMultiFrameConfig sequence;
sequence.frameCount = 3;
sequence.framePeriodSeconds = 0.1;
LidarMultiFrameDriver driver(
    world, model, profile, plan, pose, config, sequence);
for (std::size_t i = 0; i < driver.Size(); ++i) {
    auto frame = driver.SimulateFrame(i);
    // 调用方创建目录、命名文件；默认不覆盖已有文件。
    LidarPcdWriter::Save(
        outputDirectory / ("frame_" + std::to_string(i) + ".pcd"),
        frame.cloud);
    // frame.pulses 保留所有发射记录，包括未检出与被拒绝的点。
}
```

只逐帧生成，不自动缓存所有帧，也不休眠等待真实墙钟。可独立重放任意帧。同一工具链/配置可复现，不承诺跨标准库随机数逐位一致。

第 k 帧所有事件时间为 fma(k, framePeriodSeconds, 原事件时间)，不是只修改帧头。ID 步长取原计划的 maxId-minId+1，避免非连续 ID 的重复计划互相碰撞。frameId 独立递增。跨帧溢出、时间精度丢失、发射窗口重叠会被拒绝。

帧起点定义为该帧第一条计划发射的时间，endTime 为下一帧起点，采用 [start,end)。旋转计划建议 period=FrameDurationSeconds()。只模拟有限帧序列，尚不是无限在线数据流。

## PCD 字段

| 字段 | 类型 | 含义 |
|---|---|---|
| x y z | float32 | 世界位置，m；Y 向上 |
| intensity | float32 | 基线扣除后的 ADC 码幅值，不是反射率 |
| ring | uint32 | channelId，保留全 32 位 |
| time | float64 | 发射时间减帧起点，s |
| range | float64 | 当前选择的测量距离，m |
| amplitude_codes | float64 | 未降为 float 的原始码幅值 |
| peak_code | uint32 | 原始峰值数字码 |
| pulse_id_low/high | 两个 uint32 | 避免依赖 uint64 PCD 支持，重建完整脉冲编号 |
| flags | uint32 | bit0 峰值削顶、bit1 搜索区削顶、bit2 全记录越界 |
| range_source | uint32 | 0 Raw、1 Corrected |

VIEWPOINT 保存中央传感器的安装位置及 qw qx qy qz。点已在世界坐标中，不能再按 VIEWPOINT 重复变换。安装位姿不是动态轨迹。PCD 注释包含帧 ID、时间边界和字段语义；部分读取器会忽略注释，样例因此另有 manifest.json。

WIDTH=有效点数、HEIGHT=1，是无组织点云。未命中不导出原点或最大距离假点。空帧输出 POINTS 0；有的第三方读取器不接受空点云，此时用索引保留空帧时间记录。通用 XYZ/XYZI 读取器可能丢弃扩展字段，原文件仍保留全部信息。

序列化前检查全部点、时间边界和重复脉冲 ID；使用 classic locale 和足够精度。Save 默认拒绝覆盖。写入失败抛异常；文件写入目前非原子操作，磁盘错误可能留下不完整文件，调用方不得将其视为成功帧。

## 已验证与模型边界

静态场景三帧，每帧 64 次发射、38 个有效点，起点 1.0 / 1.1 / 1.2 s。无噪声场景几何能量相同；启用噪声时，各帧通过新的脉冲 ID 派生新 seed，不复用同一噪声记录。

当前仍固定安装位姿，不自动更新物体或海面。每脉冲独立接收，不包含前次脉冲拖尾、串扰、跨帧接收电路状态或运动去畸变。连续多帧在本版指扫描时间和事件序列连续，不表示完整动态物理系统。

LidarOutputAcceptance.cpp 使用独立字段解析测试格式、精度、通道、ID、削顶标记、空帧、locale、磁盘写入与覆盖防护；尚未通过真实第三方 PCL/ROS 软件加载验收。
