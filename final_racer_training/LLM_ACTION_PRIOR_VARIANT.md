# Soft LLM Action-Prior CRPO Variant

This is a separate algorithm variant built on the current `final_racer_training`
CRPO implementation. It does not replace or modify the baseline policy,
trainer, configuration, checkpoint name, supervisor entry point, or launch
scripts.

## Policy

For ten UAVs, Qwen produces:

- a `10 x 10` task-dependency matrix `W_task`; and
- a 10-element semantic upload-importance vector `omega_sem`.

Their off-diagonal and vector entries align exactly with the existing 90 relay
bits and 10 upload bits. The variant constructs centered, standardized, and
bounded preferences `b(z)` and changes the Bernoulli logits by

```text
gate(s, z) = sigmoid(Linear(fused_feature))
prior(z)   = [relay_beta * b_relay(z), upload_beta * b_upload(z)]
logits     = baseline_logits(s, z) + gate(s, z) * prior(z)
final      = physical_feasibility_mask(logits)
```

Centering prevents the prior from merely increasing the total number of active
links. Standardization gives relay and upload preferences comparable numerical
scales. Clipping bounds the largest LLM contribution.

An LLM-unselected link receives a finite residual, not negative infinity. The
small CRPO policy can still select it when current channel, AoI, missing-byte,
or queue evidence is strong enough. Only a physically unavailable BS link is
hard-masked.

The exact final logits are used for action sampling, cached behavior-policy
log-probabilities, and PPO/CRPO action evaluation. This preserves the on-policy
probability ratio.

## Isolation from the baseline

New implementation files:

- `agentic_crpo/agentic_crpo/llm_prior_policy.py`
- `agentic_crpo/agentic_crpo/llm_prior_crpo_ppo.py`
- `agentic_crpo/agentic_crpo/train_llm_prior_crpo.py`
- `agentic_crpo/agentic_crpo/llm_prior_process_supervisor.py`
- `config/qwen8b_fp8_four_process_llm_action_prior.yaml`
- `scripts/run_llm_prior_four_process_campaign.sh`
- `scripts/run_qwen8b_fp8_llm_action_prior_training.sh`

The new architecture identifier is
`physical_guidance_dual_encoder_bs_mask_soft_llm_logit_prior_v1`.
Baseline and LLM-prior checkpoints reject one another explicitly. The new final
checkpoint is named `llm_prior_crpo_final.zip`, while the baseline continues to
use `crpo_final.zip`.

## Configuration

```yaml
llm_action_prior:
  enabled: true
  relay_beta: 0.5
  upload_beta: 0.5
  clip: 2.0
  initial_gate: 0.5
```

`relay_beta` and `upload_beta` are non-negative soft-prior strengths. A value
of zero disables that block's explicit logit residual while retaining the
existing dual-encoder guidance input. `initial_gate` must be strictly between
zero and one and is subsequently learned by CRPO/PPO.

Recommended first sweep:

```text
relay_beta  = 0.25, 0.5, 1.0
upload_beta = 0.25, 0.5, 1.0
```

The trainer records the learned gate, mean absolute logit residual, and the
Bernoulli product-distribution KL from the pre-prior policy under the
`llm_prior/` TensorBoard namespace.

## Run

Run the normal 30-episode campaign with the separate launcher:

```bash
cd /home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/final_racer_training
./scripts/run_qwen8b_fp8_llm_action_prior_training.sh
```

For one short real-stack integration episode:

```bash
RACER_QWEN8_EPISODES=1 RACER_QWEN8_DURATION=5 \
  ./scripts/run_qwen8b_fp8_llm_action_prior_training.sh
```

For a policy/CRPO mock smoke test without Qwen or Isaac:

```bash
PYTHONPATH=agentic_crpo /home/jiazheng/ai_envs/racer-crpo/bin/python \
  -m agentic_crpo.train_llm_prior_crpo \
  --config config/qwen8b_fp8_four_process_llm_action_prior.yaml \
  --mock-smoke --output-dir /tmp/final_racer_llm_prior_smoke
```

Resume only from this variant's checkpoint:

```bash
RACER_QWEN8_RESUME=/absolute/path/to/llm_prior_crpo_final.zip \
  ./scripts/run_qwen8b_fp8_llm_action_prior_training.sh
```

Do not resume this variant from a baseline `crpo_final.zip`; the architectures
and optimizer parameter sets are intentionally different.
