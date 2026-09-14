# 大模型 + 小模型完整算法框架

本目录包含从 RACER/Isaac/Sionna 状态到 Qwen 大模型指导、小模型动作、CRPO
训练，以及动作在通信层只执行一次的完整源码链。模型权重和实验输出是外部运行
资产，不属于算法源码；除此之外，训练、推理、IPC、动作执行、指标、配置、脚本
和测试均在仓库中。

## 端到端数据流

```text
Isaac 物理状态 + RACER/Sionna 通信状态
  -> shared_ipc / fast_state（带状态版本、step、slot、仿真时间）
  -> Qwen3-8B-FP8 大模型生成 W_task 与 omega_sem
  -> shared_guidance（带 guidance/source version）
  -> 小模型双编码 CRPO policy
       physical encoder + guidance encoder
       baseline logits + learned gate * bounded LLM prior
       physical feasibility mask
  -> 90 个 UAV relay bit + 10 个 BS upload bit
  -> EventDrivenOneShotGate 校验动作来源并仅消费一次
  -> 下一开放 PHY slot 调度一次，等待该动作的数据包完成
  -> 发布下一个状态、资源开销、任务指标与实际 delta_t
  -> duration-aware GAE + PPO/CRPO 更新
```

固定时钟基线和 event-driven one-shot 是相互隔离的两个运行模式。固定时钟模式
仍按 100 ms 决策、5 个 20 ms slot 持有动作；one-shot 模式不持有、不重放动作。

## 大模型层

- `agentic_crpo/agentic_crpo/qwen_global_agent.py`：Qwen 配置、prompt、输出解析与
  `W_task`/`omega_sem` 构造。
- `agentic_crpo/agentic_crpo/llm_worker.py`：独立大模型进程，只读取完整版本化状态，
  推理完成后发布 guidance。
- `agentic_crpo/agentic_crpo/shared_guidance.py`：大模型 guidance 的固定二进制布局。
- `agentic_crpo/agentic_crpo/state_builder.py`：区分大模型状态、小模型快速状态，避免
  将不可观测真值泄漏到策略。

大模型只提供软指导，不直接绕过小模型或物理可行性约束。Qwen 未选择的链路仍可
被小模型选择；物理不可用链路才会被硬 mask。

## 小模型与训练层

- `crpo_policy.py`：物理状态/指导状态双编码 actor、reward critic、cost critic。
- `llm_prior_policy.py`：把大模型偏好作为有界 logit residual，并由可学习 gate
  控制强度。
- `crpo_buffer.py`：动作、旧 log-prob、reward/cost value、版本和时间戳 rollout。
- `crpo_ppo.py`：行为策略/学习策略隔离、PPO ratio 与 CRPO 更新选择。
- `llm_prior_crpo_ppo.py`：大小模型联合 variant 的独立 checkpoint/架构保护。
- `event_driven_crpo.py`：按实际仿真 `delta_t` 缩放 reward/cost gamma 与 GAE lambda。
- `task_loss.py`、`constraint.py`、`reference_metrics.py`：任务损失、约束阈值和参考
  轨迹/覆盖率指标。

小模型输出是当前状态对应的一次动作提议。PPO/CRPO 使用最终 masked logits 的同一
分布计算采样概率和更新概率，保持 on-policy ratio 一致。

## 只执行一次的保证

Python 侧 `event_driven_backend.py` 与 C++ 侧
`training_overlay_ws/src/racer_sionna_comm/include/racer_sionna_comm/event_driven_one_shot.hpp`
共同实施以下不变量：

1. 动作 ticket 必须匹配最近发布状态的 `source_step_id`、通信版本、slot 和仿真时间。
2. 重复或更旧的 `action_version` 永久拒绝；同一版本不能再次进入 pending。
3. pending/in-flight 未结束时不接受另一个动作绑定到当前状态。
4. 动作只能在源状态之后的第一个开放物理 slot 应用一次。
5. 只有该动作产生的请求与数据包完成后，才发布下一个 transition。
6. 结果必须报告 `action_held_slots == 1`；Python 后端再次检查连续 step、时间和版本。

实际调度接入位于
`training_overlay_ws/src/racer_sionna_comm/src/communication_proxy_node.cpp`。

## 进程与入口

`agentic_crpo.process_supervisor` 管理四个独立进程组：Isaac、RACER/ROS 2、Qwen、
CRPO。模型对象、优化器、planner、CUDA tensor 不跨进程，只通过版本化共享内存
传递定长数据。

- 构建：`scripts/build.sh`
- 完整检查：`scripts/check.sh`
- 大小模型固定时钟：`scripts/run_qwen8b_fp8_llm_action_prior_training.sh`
- 大小模型 one-shot：`scripts/run_event_driven_one_shot_campaign.sh`
- one-shot 配置：
  `config/qwen8b_fp8_llm_action_prior_event_driven_one_shot_reference752763_traj020_red020_constraint012.yaml`

## 外部运行资产与可移植配置

Qwen3-8B-FP8 权重体积较大，不提交到普通 Git。克隆后通过环境变量指向本地
checkpoint；代码会优先使用该路径，而无需修改 YAML：

```bash
export RACER_QWEN8_MODEL_PATH=/path/to/Qwen3-8B-FP8
export RACER_CRPO_PYTHON=/path/to/python
export RACER_QWEN8_PERFECT_RESULT=/path/to/perfect_reference.json
final_racer_training/scripts/run_event_driven_one_shot_campaign.sh
```

也可使用通用变量 `RACER_QWEN_MODEL_PATH`；若两个变量同时设置，通用变量优先。
参考结果属于实验输入而不是模型代码，可用 `--perfect-reference` 或上面的环境变量
替换。仓库不提交训练 checkpoint、TensorBoard、仿真日志、radio-map cache 或模型
权重，避免把机器相关的大文件混入算法版本。

## 验证范围

- `agentic_crpo/tests/test_qwen_global_agent.py`：大模型 schema、prompt、FP8 后端和
  可移植 checkpoint 路径。
- `agentic_crpo/tests/test_llm_prior_crpo.py`：LLM prior 与小模型 policy/PPO 一致性。
- `agentic_crpo/tests/test_event_driven_backend.py`：状态/动作/结果版本与一次性消费。
- `agentic_crpo/tests/test_event_driven_crpo.py`：可变 `delta_t` 的 reward/cost GAE。
- `training_overlay_ws/src/racer_sionna_comm/test/test_event_driven_one_shot.cpp`：重复、
  stale、in-flight 和 slot 边界行为。

更详细的 IPC 和并发边界见 `FOUR_PROCESS_ARCHITECTURE.md`；软 LLM prior 数学定义
见 `LLM_ACTION_PRIOR_VARIANT.md`；one-shot 时序见 `EVENT_DRIVEN_ONE_SHOT.md`。
