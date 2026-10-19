# 可选 HELIOS++ 经验材质基线

## 范围与复用

参考资产固定版本 `2b3585aa766213ae9a46c4790ad4f3594ffa213d` 的材质规则，
以公式重新实现，而非复制上游 C++ 源文件。上游工程使用 LGPL 许可；源链接见文末。
这是**修正逐材质初始化、适配本引擎能量单位后的单站角度基线**，不是上游扫描器的逐位复现。
没有修改本地原始 OBJ/MTL，没有检出或修改独立 HELIOS++ 源码工程，也没有改默认场景入口。

复用现有 tinyobj 解析、ObjMaterialBinding 百分数/光谱优先级与安全路径检查、
SpectralReflectanceCurve、SurfaceMaterial、逐面绑定、SurfaceLidarScattering、
LidarIntegrator、接收几何、传播时间、1/N、波形累积与后续测距链路。
不新增光谱解析器、不复制光束/接收器实现、不把 RGB 当近红外反射率。

## 初始化修正

上游 MaterialsFileReader 保存前面的条目时没有逐个 setSpecularity，只有文件结束时
对最后一个条目调用。光谱成功分支会补算，但缺光谱/default reflectance 分支不会。
本实现的 HeliosEmpiricalAngularResponse 构造函数对**每个条目**计算：

`s = sum(Ks) / (sum(Kd) + sum(Ks))`，分母为零时 s=0。

无论条目位置、反射率来源、是否有光谱，都执行同样的初始化。不复现这个初始化缺口。
Kd/Ks 分量要求有限且在 [0,1]，Ns 有限非负；不钳制错误输入。

## 角度规则与测度

入射、出射方向均为从表面向外，使用几何法线。设 c=cos(theta)>0：

- Kd/Ks 都非零：gain=(1-s)c+s*abs(2c*c-1)^Ns。
- 仅 Kd 非零：gain=c。
- Kd/Ks 都零：gain=1。
- 仅 Ks 非零：明确拒绝，与上游支持的三个分类保持边界一致。

保留上游 abs(cos(2theta)) 的掠射侧第二瓣及 Ns=0 的宽角响应，不擅自替换成标准
Phong、GGX 或 Fresnel。用代数化简避免先除以 c 再乘 c；c<=0 返回零，因此精确切线
为零而方向无关分支的 c->0+ 极限不为零。这是正面反射约定，不是连续物理 BRDF。

反射率采用 rho∈[0,1]，新增入口返回：

`EvaluateProjectedResponse = rho(lambda) * gain / pi`。

现有物理散射模型的默认同名入口仍返回 `Evaluate(...) * cos(theta_return)`。
EstimateSingleReturn 只将投影响应乘接收立体角、发射样本能量和接收效率，**不再乘第二次表面余弦**。
接收孔径投影余弦仍属于 CollectionSolidAngle，没有移到材质内部。
这样经验 Lambert 与物理 Lambert 的回波能量一致。

这个 1/pi 是本引擎的明确归一化选择，不是声称 HELIOS 原始强度有同样的单位和尺度。
没有复制百分数数值50直接参与计算、上游输出1e9缩放、功率与脉冲能量之间的约定。
因此对比的是材质角度形状，不能拿本引擎 J 值与 HELIOS 原始 intensity 数字直接相等比较。

经验模型的 Evaluate（纯 BRDF）明确报错，避免被普通路径追踪或 BRDF 混合器误用。
目前只支持单站：归一化入射/出射方向差的长度<=1e-5，容限只用于浮点归一化误差。
双站请求报错，不静默推广成一个未经验证的通用双向模型。

## 开关和使用

原默认 ObjMaterialBindingOptions 仍为 Lambert，缺反射率仍严格报错。

```cpp
auto materials = ObjMaterialBindingOptions::HeliosEmpiricalBaseline(spectraDirectory);
materials.createDiffuseRenderMaterials = false; // 纯雷达可选
auto urban = HeliosUrbanStaticScene::Load(urbanDirectory, materials);
SurfaceLidarScattering scattering;
// 扫描驱动使用 *urban.scene 与 scattering，其他接收/ADC/测距配置不变。
```

也可以仅设置 `materials.lidarAngularModel = ObjLidarAngularModel::HeliosEmpirical`，
自行决定 unassignedMaterial；工厂会显式为无 usemtl 的面配置 rho=.5 的方向无关响应。
缺反射率的自动经验条目使用假设 rho=.5，来源为 HeliosDefault，并保存诊断。
显式 missingLidarFallback 和完整 overrides 仍具有权威性，不把用户模型反解成 rho 后重建。
显式 fallback 不保证也是经验模型，调用方应查看来源。

与上游刻意不同：声明了但读取失败的光谱仍报错，不静默退回50%；透明性仍受已有
allowOpaqueApproximation/override 策略控制；纹理、折射、逆反射和分类标签传播没有因此实现。
tinyobj 解码表未保存“Ns 是否显式出现”；当前适配使用解码后的 shininess。
对一般资产缺省 Ns 可能是 tinyobj 的1而不是 HELIOS 的10，不能擅自把显式 Ns=1 改为10。
urban 所有 Phong 条目都显式填写 Ns；树木缺省 Ns 不影响其方向无关响应。
因此没有为此修改第三方解析器或再写一个 MTL 解析器。

## 验证内容

专项覆盖分类、镜面比例、首/中/末与重排不变性、上游角度公式对照、Ns=0、45度、
近掠射/精确切线、非法输入、光谱插值与频带、单站限制、坐标/方向缩放、几何法线、
默认模式不变、百分数优先级、显式 fallback/override、真实 vivaro 材质表，以及
OBJ->逐面分派->回波能量/时间->波形、1/N 与余弦不重复。
最终 CPU x64 Debug / Release 各 **3002 PASS、0 FAIL、0 SKIP**，其中新增专项81项。
使用项目 v145 工具集及固定 Windows SDK 10.0.26100.0，运行构建日志实际输出的可执行文件。
没有验证 GPU 分支，也没有声称运行了一帧完整 urban 扫描或复现 HELIOS 原始 intensity。
日志和关键源码快照：`out/HeliosEmpiricalAcceptance/run-20261016-223045-b91412/`。
60度入射的专项中，经验方向无关/经验 Lambert 能量比为1.9999999731（预期2）；
物理 Lambert 与经验 Lambert 分别约6.36619798e-9 / 6.36619806e-9 J。
初次自造 OBJ 漏写 mtllib，导致走未分配材质，现已修正并加入材质表/警告前置检查。
首轮编译的测试辅助函数 Material 命名冲突已改为 MakeMaterial，最终全量运行均通过。

## 来源

- [Material.cpp](https://github.com/3dgeo-heidelberg/helios/blob/2b3585aa766213ae9a46c4790ad4f3594ffa213d/src/scene/Material.cpp)
- [MaterialsFileReader.cpp](https://github.com/3dgeo-heidelberg/helios/blob/2b3585aa766213ae9a46c4790ad4f3594ffa213d/src/assetloading/MaterialsFileReader.cpp)
- [SpectralLibrary.cpp](https://github.com/3dgeo-heidelberg/helios/blob/2b3585aa766213ae9a46c4790ad4f3594ffa213d/src/assetloading/SpectralLibrary.cpp)
- [EnergyMaths.cpp](https://github.com/3dgeo-heidelberg/helios/blob/2b3585aa766213ae9a46c4790ad4f3594ffa213d/src/maths/EnergyMaths.cpp)

以固定版本实际源码为角度公式依据；不把 Wiki 中不完全相同的展示式视为逐行实现。
