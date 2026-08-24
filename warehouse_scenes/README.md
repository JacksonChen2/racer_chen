# Warehouse 场景资产目录

该目录集中保存 Warehouse 实验使用的规范场景资产。原工作区中的文件暂时保留，
以兼容现有 launch、烘焙脚本和正在运行或可恢复的实验。

## 目录结构

- `isaac/`：Isaac Sim 的基础、Simple、Loaded 和带工业 BS 的 USD/USD A 场景。
- `isaac_assets/`：带 BS 场景引用的工业接入点模型。
- `sionna/`：28 GHz Sionna RT XML、按材质拆分的 PLY 网格及转换报告。
- `scenario_profiles.yaml`：场景到资产、探索边界、出生点和 BS 坐标的统一索引。
- `radio_maps/`：按场景保存的 BS–UAV Sionna radio map、统计和图片。

当前实验使用的规范版本放在上述直观目录名下。盘点中发现但未被当前入口加载的
两个不同哈希版本也已保留，避免整理时丢失历史资产：

- `isaac/warehouse_loaded_root_variant.usd`：原 `racer_chen/` 根目录版本。
- `sionna/warehouse_loaded_3d_sionna_variant/`：`ros2_3d_sionna_ws` 的 Loaded
  RF 网格版本；其金属网格与当前原始 RACER 实验版本不同。

## 场景选择

| 配置名 | Isaac 几何 | 探索区域 |
|---|---|---|
| `warehouse_simple` | `warehouse_simple.usd` | Simple 厂房 |
| `warehouse_loaded` | `warehouse_loaded.usd` | 北侧货架区，`y=7.2..26.2 m` |
| `warehouse_loaded_center` | 同 Loaded | 同货架区，改用中央通道出生点 |
| `warehouse_loaded_full` | 同 Loaded | 完整封闭厂房，`y=-23..30 m` |

`warehouse_loaded_full` 的几何仍复用 `warehouse_loaded.usd` 和
`sionna/warehouse_loaded/warehouse.xml`，但提供了明确命名的组合层
`warehouse_loaded_full.usda` 与 `warehouse_loaded_full_with_industrial_ap.usda`。
组合层内记录了完整厂房边界；完整厂房不包括最南侧的室外装卸平台。

## 可直接打开的 Isaac 场景

- 不带 BS：`isaac/warehouse_simple.usd`、`isaac/warehouse_loaded.usd`
- 带 BS：`isaac/warehouse_simple_with_industrial_ap.usda`、
  `isaac/warehouse_loaded_with_industrial_ap.usda`
- Loaded Full：`isaac/warehouse_loaded_full.usda`、
  `isaac/warehouse_loaded_full_with_industrial_ap.usda`
- Warehouse Full（天花板中心 BS）：
  `isaac/warehouse_full_with_industrial_ap.usda`

带 BS 的两份 USDA 使用相对引用；不要只复制单个文件，应同时保留当前
`isaac/` 与 `isaac_assets/` 的目录层级。

## 规范来源与兼容路径

本目录的 Isaac 文件来自 `ros2_3d_py_ws/`，Sionna 文件来自
`ros2_original_fidelity_sionna_ws/src/racer_sionna_comm/assets/`。代码当前仍从这些
兼容路径加载。后续更新或重新烘焙场景时，应同步更新本目录，并以
`scenario_profiles.yaml` 记录配置对应关系。
