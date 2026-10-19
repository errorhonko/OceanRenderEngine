# ECOSTRESS 原始光谱镜像：本地保存与初步检查

检查日期：2026-10-19。只下载、保存、检查数据；未修改引擎、场景绑定或材料模型。

## 来源与完整性

- 数据发布页：<https://zenodo.org/records/15881238>，Harrison Nicholls / PROTEUS，v3，2025-07-14。
- 下载文件：`ecostress_orig.zip`。不是经过球面反射率/单次散射反照率转换的 `ecostress.zip`。
- 发布者记录：2025-06-04 从 Gerardo Rivera 获得数据。
- 压缩包：41,526,412 bytes（41.53 MB，39.60 MiB）。
- 解压内容：119,272,379 bytes（119.27 MB，113.75 MiB），不含文件系统额外开销。
- MD5：`8df79582088b5dccc2d56907261b8449`，与发布页一致。
- SHA-256：`487561421483fd9abde2304cb6abf42d3f961a488e3ec254520f08b2deaf9e15`。
- ZIP 有 3499 个成员，其中 44 个是目录；实际 3455 个文件 = 3451 份光谱 + 4 个附属文件。

这是公开镜像快照，不是 NASA 服务器直接提供的全量下载，不能宣称等同于 NASA 当前全套。
NASA 下载页列出 6139 个光谱及附属文件：<https://speclib.jpl.nasa.gov/download>。
不要把 ZIP 成员数、文件数、光谱数、材质种类数混为一谈。

## 目录与原始类别

- `ecostress_orig.zip`：按原始字节保存的压缩包。
- `raw/ecostress_raw/`：原样解压的光谱。
- `raw/readme.txt`：原始说明、归属和论文引用要求。
- `raw/convert.py`、`raw/plot.py`、`raw/plot.png`：随包附属文件。脚本没有执行，不能假设其转换适合本项目。
- `download-manifest.json`：来源、获取时间、校验值和体积。
- `archive-contents.csv`：ZIP 成员清单。
- `metadata-index.csv`：3451 份光谱的头部元数据索引。
- `reader-audit.csv`：现有 C++ 读取器逐文件的接受/拒绝结果，以及可读数据在 905/1550 nm 的插值。
- `raw-sha256.csv`：所有解压文件的 SHA-256，供原样复制后复核。

以下按原始物理文件夹统计，而非按头部 Type 字符串重新分类；二者存在少量标签不一致：

| 文件夹 | 光谱文件数 |
| --- | ---: |
| mineral 矿物 | 1609 |
| vegetation 植被 | 1040 |
| rock 岩石 | 470 |
| nonphotosyntheticvegetation 非光合植被 | 123 |
| manmade 人工材料 | 72 |
| soil 土壤 | 69 |
| meteorites 陨石（含 Lunar 分类） | 59 |
| water 水/冰/雪/泡沫 | 9 |
| 合计 | 3451 |

人工材料包含 5 份 Concrete、28 份 General Construction Material、16 份 Reflectance target、5 份 Road、18 份 Roofing Material。
具体有混凝土、沥青、焦油、砖、涂料、木材、玻璃、橡胶、瓦、铝、铜、镀锌钢等。
同名材料的不同编号代表不同样品，不能只按显示名称合并。

## 数据如何解释

文本先是描述头部，再是两列数值：波长与测量值。头部包含 Name、Type、Class、Subclass、Sample No.、Owner、Description、Measurement、X/Y Units、端点和样本数。

本快照波长以微米声明，但存在 `micrometer` / `micrometers` 拼写差异；数据不统一按波长递增排列。
Y Units 包含反射率、透射率以及历史拼写变体。不能统一解释成 reflectance(percent)。
测量几何包含半球反射、方向-半球反射、双向反射和透射，必须逐样品核对。
原始包说明对测量几何的概述不能覆盖每个样品头部的差异。

对明确的波长/百分比反射率数据，内部单位为：

```
wavelengthNm = wavelengthMicrometers * 1000
rho = reflectancePercent / 100
```

rho 不是完整 BRDF，也不包含 n/k、粗糙度或相函数。用 Lambert 时的 `rho/pi` 是额外选择的基线模型，
并非原始实验给出了 Lambert 角度分布；尤其镜面金属、光泽涂层、玻璃需要谨慎。
一条测量曲线也不能自动分解出涂层和基底的参数。

## 现有读取器的实测兼容性

用当前项目 `SpectralReflectanceCurve::FromAsterFile()` 对全部 3451 份光谱逐一执行，未修改或放宽读取器：

- 520 份直接接受，均覆盖 905 和 1550 nm。
- 2931 份拒绝。按每文件遇到的第一个错误统计：
  - 360 份：X Units 不在现有严格支持的拼写中。
  - 943 份：Y Units 不在现有支持范围中（包括其他物理量和拼写变体）。
  - 1628 份：波长不是严格递增。
- 人工材料：45 份接受，27 份拒绝。
  - 11 份 TIR 涂料波长为递减排列。
  - 16 份 TIR 反射率靶的 Y Units 写成 `Reflectence (percent)`。
  - 这 27 份本身也都不覆盖 905/1550 nm；仅修正格式仍不能用于这两个波长。

拒绝不等于数据损坏；这里记录的是现有接口的严格兼容性。不同错误可能在同一文件叠加，以上不是所有异常的独立计数。
没有自动排序、补值、截断、合并重复波长、改写单位或把透射率当反射率。后续如需正规化应新建派生文件并记录转换，原始数据保持不变。

## 与城市场景相关的样例

下表是现有读取器执行分段线性插值得到的无量纲 rho，已四舍五入，不是恰好在这些波长独立测得的新数据。

| 样品 | 编号 | rho(905 nm) | rho(1550 nm) |
| --- | --- | ---: | ---: |
| 建筑沥青 | 0095UUUASP | 0.113973 | 0.149967 |
| 沥青混凝土（先前 HELIOS asphalt） | 0425UUUASP | 0.277631 | 0.361647 |
| 建筑混凝土 | 0397UUUCNC | 0.339318 | 0.307110 |
| 裸红砖 | 0413UUUBRK | 0.260050 | 0.562700 |
| 黑色光泽涂料 | 0402UUUPNT | 0.037841 | 0.043305 |
| 橄榄绿光泽涂料 | 0385UUUPNT | 0.414775 | 0.635700 |
| 松木 | 0404UUUWOD | 0.918261 | 0.564886 |
| 黑色橡胶 | 0833UUURBR | 0.067250 | 0.063700 |

沥青不同样品差异明显，不应把先前 HELIOS 的 asphalt 曲线当作所有道路的统一真值。
黑色和绿色涂料也不能仅凭可见 RGB 颜色推出近红外反射率。
库内 Sea Foam / Sea water 样品从约 2.0795 um 起，不覆盖 905/1550 nm，不能直接用来校准当前海面/白冠的这两个波长。

## 接入建议与边界

下一小步：先从可读的人工材料中按样品编号选择一份道路、一份混凝土、一份砖、一份涂料、一份木材，
建立显式场景绑定；普通非透明材料可以选 spectral Lambert 基线，已有 HELIOS 经验角响应仍是独立的可选模型。
需要物理金属/涂层响应的材料继续使用已有 n/k 和模型，缺失粗糙度不能从这份 rho 数据假定出来。
玻璃只给反射曲线不足以完整描述传播/透射，不应自动绑定为不透明 Lambert。

本次未改变场景任何材料绑定，未提交/推送 Git。`.gitignore` 默认排除下载数据、派生索引和随包脚本，
仅允许本项目说明与复现检查工具被跟踪，避免无意将第三方测量资料再发布。

## 归属、再发布与引用

`raw/readme.txt` 保留原始说明及 `Copyright 2017, California Institute of Technology. ALL RIGHTS RESERVED.`。
没有将这批测量擅自标记为 CC0 或 HELIOS 的软件许可证；镜像可下载不代表所有底层数据具有统一再发布许可。
公开发表/再分发前，应核对原始提供者要求。

原始说明要求研究使用时引用：

- Meerdink et al. (2019), The ECOSTRESS spectral library version 1.0. <https://doi.org/10.1016/j.rse.2019.05.015>
- Baldridge et al. (2009), The ASTER Spectral Library Version 2.0. <https://doi.org/10.1016/j.rse.2008.11.007>
- NASA/JPL 官方测量说明入口：<https://speclib.jpl.nasa.gov/documents>。
