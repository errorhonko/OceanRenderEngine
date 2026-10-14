# Windows SDK 26100 固定与完整验证

日期：2026-10-14。构建工具 MSVC v145 / VS 2026。

## 配置范围

六个项目的 WindowsTargetPlatformVersion 均固定为 `10.0.26100.0`：

- 主程序 OceanRenderEngine。
- CPU 验收 OceanRenderEngine.Tests。
- GPU 静态库 OceanRenderEngine.GpuDx12。
- GPU 验收 OceanRenderEngine.GpuTests。
- OceanNormalStudy 实验。
- OceanVisibilityStudy 实验。

SDK 工具路径逐项目核对，实际解析到 `Windows Kits/10/bin/10.0.26100.0`。
不使用浮动 `10.0`，未关闭工程清单生成，也未修改 SDK 安装文件。

## 验证结果

| 范围 | x64 Debug | x64 Release |
| --- | --- | --- |
| 完整 .slnx 的主程序、CPU 验收、GPU 库及 GPU 验收重建 | 通过 | 通过 |
| 默认清单生成 / mt.exe | 通过 | 通过 |
| CPU 验收 | 2300 PASS，0 FAIL，0 SKIP | 2300 PASS，0 FAIL，0 SKIP |
| 硬件 GPU 验收 | 通过，无跳过 | 通过，无跳过 |
| OceanNormalStudy 编译 | 通过 | 通过 |
| OceanVisibilityStudy 编译 | 通过 | 通过 |

GPU 实测设备：AMD Radeon RX 7900 XTX，HARDWARE 后端，DXR 1.1。
Debug 开启 D3D12 debug layer；Release 未开启。每个配置的覆盖包括：

- 仿射 compute：0/1/63/64/65/257/4097 元素边界批次、非法输入、9094 个元素对照。
- DXR：12 个解析几何情形、正反面/有限线段、486 条射线对照（每条含最近命中与遮挡两种模式）。
- 持久场景：1152 条射线对照、构建一次多次追踪、批次缓存复用、空批次、场景移动与上下文归属。

这是既有 GPU 基础模块验收，不等于完整海面/材质/雷达模型已经迁移到 GPU。
未验证 Win32 运行；未运行主程序的 Bunny 渲染或重跑两个耗时数值实验。
编译存在既有浮点转换等警告，没有为通过验收修改算法或屏蔽警告。

## 复现

在 Visual Studio Developer PowerShell 中，从仓库根目录执行：

```powershell
msbuild .\OceanRenderEngine.slnx /t:Rebuild /m:2 /p:Configuration=Debug /p:Platform=x64
.\x64\Debug\OceanRenderEngine.Tests.exe
.\tests\OceanRenderEngine.GpuTests\bin\x64\Debug\OceanRenderEngine.GpuTests.exe
```

将 Debug 换成 Release 可验证另一配置。完整解决方案的 CPU 输出在仓库根目录
`x64/<配置>/`；单独构建测试项目时输出位置可能不同，始终运行日志刚生成的程序。
本次核查曾发现测试项目目录中的旧 Debug 程序只有 420 项，已排除该旧程序作为验证证据。

## 先前 SDK 28000 故障

本机 28000.2526 的 mt.exe 依赖 MidlrtMd.dll，但对应工具目录缺失 DLL，
直接运行帮助命令也返回 0xC0000135。缓存 MSI 文件表包含该 DLL，安装功能标记已安装，
而相应组件在此版本的登记路径为空；26100/22621 的对应路径和文件正常。
不同版本沿用相同 DLL 组件 GUID，但目标目录不同，怀疑多版本安装/升级时组件登记异常；
缺少安装日志与官方确认，不能将其认定为所有 SDK 28000 的通用缺陷。
固定 26100 绕开本机异常安装，没有从其他版本复制 DLL 或卸载 SDK。

## 本地记录

日志：Codex 工作区 `output/sdk-26100/Debug/` 与 `Release/` 中的
solution-build.log、cpu-run.log、gpu-run.log 和两个实验的 build.log。
工程配置备份：工作区 `output/sdk-26100/backup/`。
对应源码与配置随版本检查点提交；构建日志和备份仅保留在本地工作区。

发布前补充：不含模型缓存的暂存源码在纯英文临时目录下 Release 构建/运行通过，
2235 PASS / 0 FAIL / 2 SKIP（Toyblocks/Urban）。中文目录副本构建成功但测试资源定位出现
异常退出；这是新增发现的路径编码限制，未在本次发布修复，不影响上述原英文仓库路径验证结果。
