#pragma once

#include "core/math/Point2f.h"
#include "core/math/Vector3f.h"

// 空间光束方向样本，不包含时间脉冲、发射位置或总脉冲能量。
struct LidarBeamDirectionSample
{
    // 发射器局部单位方向，+Z 为光束主轴。
    Vector3f directionLocal = Vector3f(0.0f, 0.0f, 1.0f);

    // 实际采样方向的 PDF，关于立体角，单位 sr^-1。
    // Delta 方向约定为 0，由 deltaDirection 单独标记。
    double pdfPerSr = 0.0;

    // 连续分布为 b(omega) / q(omega)，无量纲。
    // b 为物理方向能量密度，q 为实际采样 PDF；直接按 b 采样时为 1。
    // 不包含 pulseEnergyJ 或蒙特卡洛的 1/N；Delta 样本约定为 1。
    double energyWeight = 1.0;

    bool deltaDirection = false;
};

// 仅描述空间光束；时间形状仍由 LidarPulseProfile 独立负责。
// 连续模型的物理能量密度及采样 PDF 均关于立体角，分别积分为 1。
// q 必须覆盖所有 b 非零的方向，才能使用 b/q 构造无偏估计。
// 截断后重新归一化的模型，其总能量由发射器的 pulseEnergyJ 指定。
class LidarBeamProfile
{
public:
    virtual ~LidarBeamProfile() = default;

    // 与现有 LaserEmitter 一致，u 的各分量为有限的 [0, 1] 样本。
    // 实现必须处理端点，不能让 log(0) 等操作产生无效方向。
    // 返回局部单位方向；非 Delta 样本的 PDF 必须为有限正值。
    virtual LidarBeamDirectionSample SampleDirection(
        const Point2f& u) const = 0;

    // 物理方向能量密度 b(omega)，单位 sr^-1。
    // 输入为有限局部单位方向；支持域外返回 0。
    // 纯 Delta 模型没有普通密度，此函数约定返回 0。
    virtual double EnergyDensityPerSr(
        const Vector3f& directionLocal) const = 0;

    // 实际采样密度 q(omega)，单位 sr^-1，不必与 b 相同。
    // 输入为有限局部单位方向；支持域外及纯 Delta 模型返回 0。
    virtual double PdfDirection(
        const Vector3f& directionLocal) const = 0;

    // 第一版模型为纯连续分布或纯 Delta 分布，不表达两者的混合。
    virtual bool IsDeltaDirection() const = 0;
};
