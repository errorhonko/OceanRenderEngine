# 城市静态场景导入：HELIOS++ urban_dyn

2026-10-13。此阶段只导入静态几何和建立 BVH，不执行车辆动画，不生成新点云，
不改变项目原有 `main.cpp`。后续扫描驱动可直接使用返回的 `StaticScene`。

## 接入方式

```cpp
#include "scene/examples/HeliosUrbanStaticScene.h"
auto imported = HeliosUrbanStaticScene::Load("models/helios_urban_static");
auto world = imported.scene; // Keep this alive while using *world in the scanner.
```

`HeliosUrbanStaticSceneParts.h` 保存固定版本 XML 的 42 个配置部件。
`HeliosUrbanStaticScene.h` 读取 27 个独立 OBJ（带 MTL），缓存解析转换结果，
执行静态放置，再交给通用 `SceneBuilder` 建立网格 BVH 和场景 BVH。
这些是数据集专用预设，不是新增通用 XML 解析库；不执行资源中的脚本或动态指令。

重复资产只解析一次，但当前 SceneBuilder 仍按位置复制几何，不是共享底层 BVH 的实例化。
缓存只存在于本次 Load 中；返回场景拥有独立静态几何快照。

## 布局和坐标

所有源模型声明为 Z-up，源单位假定 1 m/unit。右手坐标转换为：

`C(x,y,z) = (x,z,-y)`。

按 HELIOS++ 源码的规则，XML 的变换过滤器先积累属性，最后统一按
`p_engine = C(s R_source p_source + t_source)` 应用；不是把文件里出现的
translate、rotate、scale 逐个直接施加到世界顶点。多个默认 global 旋转依列出顺序组合。
引擎实现等价的 `s (C R C^-1) p_engine_local + C t`，正常方向只接受旋转。

不逐个模型居中、落地或缩成 Toyblocks 尺寸，不给全城附加公共平移。
保留建筑、地形、车辆的原坐标与配置变换。没有复制 HELIOS++ 最后为数值稳定做的
内部全场景包围盒中心平移；以后需要平移时，应对全场景与轨迹统一应用并记录。
两个原动态车辆冻结在 OBJ 初始位置，即任何 `dmotion` 执行之前。

## onGround 放置

原 XML 中 18 个放置配置带 `onGround=1`，不是“把 y 设成 0”。
与上游算法思路一致，先用 Terrain 三角形各角点（保留重复次数）拟合正交最小二乘平面。
本实现用 3×3 对称协方差的 Jacobi 最小特征向量获得平面法线，不新增 SVD 库依赖。

物体先旋转、缩放、平移，再找最低顶点 q，计算拟合平面在 q.x/q.z 的高度，
对整个部件只做竖直平移。部件可以包含多个 OBJ 网格组，最低点由整个部件决定。

**拟合平面放置不保证物体接触真实非平面地形**：实际地形的起伏可能导致局部浮空/相交。
这是上游这种 onGround 启发式的边界，不应声称已经做了地形接触求解。
我们的 tinyobj 三角化、float 坐标和清理策略与 HELIOS++ 的解析器/双精度实现不完全相同，
因此这是保留布局和规则的静态适配，不是 HELIOS++ 输出的逐位复现。

## 真正遇到的资源问题

`vivaro.obj`、`vivaro.001.obj`、`vivaro.000.obj` 各有 161994 个源三角形，其中 6 个零面积。
原始严格导入器正确地拒绝这些文件。增加显式 `skipDegenerateTriangles` 选项后，
该预设输出每车 161988 个面，并通过 `ObjImportDiagnostics` 报告清理。

只跳过源坐标下精确零面积面，用 double 差值/叉积判断，没有面积 epsilon。
错误索引、非有限属性、非法材质编号、单位转换造成的折叠/溢出仍报错。
默认 Convert 保持原先拒绝策略；失败时不覆盖调用方的诊断报告。
全零面积组不生成空网格；全文件没有剩余三角形则失败。原资源未编辑。

唯一源资产清理 18 个面，考虑车辆重复放置共减少 54 个三角形贡献；没有进一步简化。

## 实际导入规模

- 独立 OBJ：27；源配置部件：42；实际网格对象：53。
- 全场景有效三角形：2445341（约 244.5 万）。
- Terrain 水平范围约 x=[-136.687,124.942] m、z=[-226.206,113.939] m。
- 所有 OBJ/MTL 引用读取无警告；无纹理加载、没有把 MTL 变成雷达光谱或语义标签。
- `Inliner-Bahn.obj` 原本没有 MTL/usemtl 声明，合法保留未分配材质，不凭空补建。
  此模型含五边形面，由 tinyobj 三角化；这也是与上游解析器不逐位相同的原因之一。
- 独立 Release 导入/构建探测的一次用时约 1.68 s，只是本机单次观测，不是扫描速度或性能保证。

## 验收与复现

`HeliosUrbanStaticSceneAcceptance.cpp` 包含不依赖外部数据的倾斜平面拟合、旋转顺序、
布局变换、法线/UV、地面放置数学测试；有本地资源时继续完整导入和真实 BVH 求交测试。
缺资源时真实城市部分显式 SKIP，数学检查仍执行。
新增 OBJ 验收检查默认严格行为、清理计数、空组、很小的正常面、错误属性与尺度折叠。
独立城市专项 39 项通过；新增 OBJ 清理策略检查 10 项通过。
CPU 全量回归 Release / Debug 各 1989 项通过，零失败、零跳过；原 Toyblocks 链路仍通过。

复现步骤：

1. 在允许本地脚本的 PowerShell 会话运行 `models/helios_urban_static/FetchUrbanStatic.ps1`。
2. 构建运行 Visual Studio CPU `OceanRenderEngine.Tests`。
3. 如需独立报告，从继承 **x64 Native Tools** 环境的 PowerShell 7 运行：

```powershell
pwsh -NoProfile -File .\experiments\UrbanStaticImport\Run.ps1
```

可用 `-OutputDirectory` 指定新目录，默认存入 Git 忽略的 `out/UrbanStaticImport/run-...`。
`summary.txt` 给出资源统计和放置偏移；`scene-layout.json` 仅保存部件包围盒元数据，
不是完整网格，也不是扫描结果。已有报告不覆盖。独立入口没有替换工程入口。

## 下一小步

选择处于道路上方的发射位姿，先跑一帧城市扫描；按百米级范围扩大接收记录，
再调整通道/方位密度。几何正常后才加多帧平台轨迹、实际地形接触放置、材质差异和车辆运动。
这里地面有海拔和起伏，不能照搬 Toyblocks 的绝对 y=1.5 m。应先查询道路地形的实际高度，
再把雷达放在路面上方约 1.5 m；传感器位置不是树木/车辆的拟合平面 grounding 参考。

## 来源与许可

2026-10-14 补充：新增可选 `Load(directory, ObjMaterialBindingOptions)` 自动逐面材质绑定，
旧 `Load(directory)` 仍为本文的纯几何模式。沥青谱与显式默认值的接入、74 项新增验收及
真实资产求交探针记录见 [OBJ/MTL 自动材质绑定](ObjMaterialBinding.md)。
这不是重新完成城市点云扫描，也不把默认模型当作实测材料。

固定资源与规则来自 HELIOS++ commit `2b3585aa766213ae9a46c4790ad4f3594ffa213d`：

- [场景 XML](https://github.com/3dgeo-heidelberg/helios/blob/2b3585aa766213ae9a46c4790ad4f3594ffa213d/data/scenes/dyn/urban_dyn_scene.xml)
- [变换规则 ScenePart.cpp](https://github.com/3dgeo-heidelberg/helios/blob/2b3585aa766213ae9a46c4790ad4f3594ffa213d/src/assetloading/ScenePart.cpp)
- [地面拟合/放置 Scene.cpp](https://github.com/3dgeo-heidelberg/helios/blob/2b3585aa766213ae9a46c4790ad4f3594ffa213d/src/scene/Scene.cpp)
- [global 旋转顺序 XmlUtils.cpp](https://github.com/3dgeo-heidelberg/helios/blob/2b3585aa766213ae9a46c4790ad4f3594ffa213d/src/assetloading/XmlUtils.cpp)

原始模型/XML/许可文件仅保存本地并被 Git 忽略。上游根代码许可不自动确立每个模型或
衍生数据集的再发布权；发布前需核实适用资产许可与署名要求。本阶段不提交、不推送。
