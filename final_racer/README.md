# Final RACER：75.2913% 完美通信基准

这是一个独立于外部 `hybrid_communication_racer` 工作目录的可运行归档。它固定了
2026-09-02 正式实验的旧版 RACER 算法、Isaac adapter、理想通信代理、被动任务指标
overlay、场景/飞行器资产、GT voxel 和完整参考结果。

参考正式实验的 joint mapping coverage 为 **75.2913%**
（精确值 `0.752913131313`），仿真时长 `300.01 s`，无碰撞；理想通信共尝试并成功
投递 `12,663,486` 个包。被动 observer 以 `10 Hz` 记录了 `3001` 个冗余率和全局地图
IoU 样本，不参与 RACER 规划或通信决策。

## 固定实验设置

- 10 架 UAV，五个起飞区域，精确起飞位置见 `config/experiment_75_2913.json`
- `seed=42`，distributed topology，ideal/lossless broadcast
- 15 秒 preflight 后运行 300 秒正式实验
- 100 Hz physics、10 Hz depth sensor、76,800 rays、8 sensor workers
- 20 Hz scene query、20 ms ideal coalesce window、无重传
- 不使用 `--fixed-exploration-horizon`
- 10 Hz 异步被动记录冗余率和 BS global-map IoU

## 构建与验证

系统依赖为 Ubuntu、ROS 2 Humble、`colcon` 和 Isaac Sim。Isaac Sim 默认位置是
`/home/jiazheng/software/isaacsim`，也可通过 `ISAAC_SIM_ROOT` 指定。

```bash
cd /home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/final_racer
./scripts/build.sh
./scripts/check_package.sh
./scripts/run_perfect_75_2913.sh --check
```

## 重新运行

建议先停止其他 Isaac/Qwen GPU 任务。默认资源检查与原 75.2913% 正式运行一致，要求
`nproc=18`，并允许检测到共享 GPU；若要强制独占 GPU，可设置
`RACER_ALLOW_SHARED_GPU=0`。

```bash
./scripts/run_perfect_75_2913.sh --run
```

新结果写入 `results/perfect_75_2913_<时间戳>/`。脚本会依次执行 15 秒 preflight、
等待 30 秒清理进程，再执行 300 秒正式实验，并验证覆盖率相对参考值误差不超过 10%。
历史正式实验的仿真 wall time 约 481 秒；连同 preflight、启动和清理，通常需要约
9–12 分钟，具体取决于 GPU/CPU 负载。

同一组五起飞点也可运行 300 秒的纯分布式 Sionna 对照。该入口保持
`final_racer` 的 RACER planner 和 Isaac adapter，仅用独立 overlay 替换无线通信代理；
配置为 100 MHz/66 RB 共享 OFDMA、UAV 23 dBm、固定 MCS 14、UDP 定向单播、
无 BS、无重传。RACER 内部 coverage callback 与完美通信设计相同，固定为 0.5 Hz
（每架 UAV 每 2 秒一次）；训练用任务指标在该对照中关闭，避免地图反序列化影响探索时序。

```bash
./scripts/run_sionna_distributed_10uav_5sites_300s.sh --check
./scripts/run_sionna_distributed_10uav_5sites_300s.sh --run
# 或依次运行 perfect 和 Sionna：
./scripts/run_pair_perfect_vs_sionna_10uav_5sites_300s.sh
```

## 目录说明

- `ros2_ws/`：固定旧版 RACER、Isaac adapter、理想通信代理源码与本地构建
- `original_racer_ros1_ws/`：fidelity tests 使用的原始 ROS1 RACER 对照源码
- `passive_metrics_overlay_ws/`：只用于异步被动指标记录的通信代理 overlay
- `sionna_distributed_overlay_ws/`：纯分布式共享 OFDMA/UDP 的 Sionna 通信 overlay
- `assets/`：本实验所需 Warehouse、无人机和 Sionna 几何资产
- `data/gt_occupied_voxels.txt`：IoU 使用的固定 GT
- `reference_result/`：75.2913% 的 preflight、formal 结果、完整日志和轨迹覆盖率图
- `scripts/`：构建、完整性检查和复现实验入口
- `SHA256SUMS`：归档输入与参考结果校验值（不包含可重建的 build/install/log）

`reference_result/reproduction_manifest.json` 是历史原件，因此其中保留了当时的绝对
路径作为溯源信息；运行代码和新生成结果不依赖这些历史路径。

测试详情和两项已知的 Pairwise Robust 差异见 `TEST_REPORT.md`。
