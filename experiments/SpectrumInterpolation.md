# 公共光谱插值层

2026-10-14。将两种光谱容器和旧 `SampledSpectrum` 的插值数值逻辑统一，
保留原来的业务类、所有权和数据校验。没有新增材质模型或改变光谱单位。

## 结构

公共头文件：`src/core/spectrum/SpectrumInterpolation.h`。

| 调用方 | 共用的数值逻辑 | 保留的业务约定 |
| --- | --- | --- |
| SpectralReflectanceCurve | FindInterval + Interval::Evaluate | rho 在 [0,1]；频带外报错；ASTER 解析保持独立 |
| SpectralOpticalConstants | FindInterval + 同一区间分别 Evaluate n/k | n/k 非负、不同时为零；频带外报错；来源和样品条件 |
| SampledSpectrum::Interpolate | InterpolateClamped → Weight + Interval::Evaluate | float 兼容入口；区间外采用端点值；不外推 |

`AverageSpectrumSamples()` 仍按原算法对分段线性曲线进行梯形积分再除以区间宽度，
它只通过已有 `Interpolate()` 入口间接使用公共计算，不变成单点查询。
旧可见光重采样区间、CIE/XYZ/RGB 公式不在本次重构范围内。

## 公共接口

- `Weight(x,x0,x1)`：有限且递增区间内的线性权重；越界报错，不隐式钳制。
- `FindInterval(span,x,projection)`：二分定位；返回 lower/upper 索引和 double 权重。
  精确命中节点时两个索引相同、权重为 0，保持原节点数值。
- `Interval::Evaluate(v0,v1)`：使用 `std::lerp`，支持 float/double 值。
- `InterpolateClamped(x,x0,x1,y0,y1)`：显式端点延拓入口，复用公共权重和求值。

FindInterval 要求表格的坐标已在构造时校验为有限、严格递增；不会每次做 O(N)
全表检查。它检查查询有限性、样本数量和范围。公共层不负责 nm、正波长、反射率
范围、n/k 合法性、文件单位或数据来源，这些仍由业务容器管理。

与旧 Interpolate 的区别：对正常有限递增数据保留返回规则，改用 std::lerp 的
数值实现；非法数值、相同/反向端点明确报错，不再接受未定义意义的区间。
只保证数值一致到浮点误差容限，不保证旧 float 加权加法逐位相同。
区间宽度若溢出公共计算明确报错；正的光谱波长区间没有正负极端值相减问题。

SampledSpectrum 头文件里的 CIE_XYZ 同时标记为 inline，避免多个翻译单元引用
旧光谱头时发生重复定义；函数计算内容未改变。测试专门用两个翻译单元验证链接。

## 验收范围

新增 `SpectrumInterpolationAcceptance.cpp` 和 `SpectrumInterpolationLinkAcceptance.cpp`，
覆盖区间定位、精确节点、float/double、非均匀间隔、越界与非法查询、旧端点延拓、
两个容器对相同标量数据的一致性、独立线性函数对照、旧区间积分/恒定光谱重采样、
多翻译单元链接。保留原有全部光学常数和反射率验收。

实际验证：x64 Debug/Release CPU 验收各 2407 项通过，零失败、零跳过；
本次新增公共插值/兼容性检查 53 项，既有光学常数 54 项仍全部通过。
Windows SDK 10.0.26100.0，默认 manifest；按各配置本次构建日志中的实际路径运行。
未运行 GPU 验收。旧 CIE 和其他既有 float 转换编译警告未在本次改变算法消除。

本次没有扩大旧 SampledSpectrum 的输入校验或修正其其他重采样/CIE 设计约定，
没有接入导体的光谱模式，没有下载真实光学常数。
