# Final RACER 验证记录

验证日期：2026-09-03。

## 已通过

- 独立 clean-path 构建：ROS 2 基础工作区 10 个 package 全部构建成功
- passive metrics overlay：`racer_sionna_comm` 构建成功
- 运行路径解析：`racer_original_core` 和 overlay `racer_sionna_comm` 均解析到
  `final_racer` 内部
- 参考结果：coverage `0.752913131313`、elapsed `300.01 s`、ideal/lossless、
  10 架 UAV 起飞点完全匹配、task-quality samples `3001`
- `hybrid_communication_racer` assignment：5/5 tests passed
- `racer_sionna_comm`：19/19 tests passed，其中 ideal proxy 10、link model 7、
  active links 2
- fidelity functional checks：9/11 passed，包括 ROS1/ROS2 B-spline 输出逐字节一致、
  LKH 输出一致、安全层隔离、callback serialization、退化 hover、trigger readiness、
  Isaac graph readiness 和 completion monitor

## 保留的两项已知差异

完整 fidelity test suite 中有两项失败，来自生成该参考结果的 Pairwise Robust 旧版
工作区本身，不是打包或路径迁移造成的：

1. `racer_source_fidelity`：报告 `path_searching/include/path_searching/astar2.h`
   存在已记录范围外的上游修改。
2. `racer_wire_schema_fidelity`：ROS 2 `DroneState` 使用 `int32[] grid_ids`，并增加
   `assignment_epoch`、`commitment_until`、`requesting_work` 字段，和纯 ROS1 schema
   不完全一致。

这些内容不能在本归档中“修复”，否则会改变产生 75.2913% 结果的算法。具体版本设计
见 `ros2_ws/PAIRWISE_ROBUST_RACER.md` 和 `ros2_ws/PORTING_CHANGES.yaml`。

未在本次打包过程中重跑 300 秒 Isaac 仿真，因为 GPU 上有其他用户的两个计算进程；
归档保留了已完成正式实验的完整结果，并通过了构建、入口、资源和参考结果检查。
