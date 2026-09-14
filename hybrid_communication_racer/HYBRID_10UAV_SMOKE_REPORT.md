# Final 10-UAV perfect-communication smoke report

Two paired 30-second runs used the final source/build, the verified 10-UAV
`warehouse_full` layout, ideal lossless communication, 100 Hz physics, 10 Hz
sensors, 76,800 camera rays, seed 42, and identical settings for `original` and
`global_cooperative`. All 10 UAVs entered trajectory execution in all four cases.

| Metric | Repeat | original | global_cooperative | Difference |
|---|---:|---:|---:|---:|
| Coverage at 30 s | 1 | 20.1299% | 21.1655% | +1.0357 pp |
| Coverage at 30 s | 2 | 16.9934% | 19.9868% | +2.9933 pp |
| Coverage at 30 s | mean | 18.5617% | 20.5761% | +2.0145 pp (+10.85%) |
| 5–20 s growth slope | 1 | 0.6530 pp/s | 0.6317 pp/s | -0.0214 pp/s |
| 5–20 s growth slope | 2 | 0.6312 pp/s | 0.7175 pp/s | +0.0863 pp/s |
| 5–20 s growth slope | mean | 0.6421 pp/s | 0.6746 pp/s | +0.0324 pp/s (+5.05%) |
| Coverage AUC | mean | 269.3263 pp·s | 282.1966 pp·s | +12.8703 pp·s (+4.78%) |

Behavior checks across both repeats:

- All 10 UAVs executed; neither mode crashed.
- Joint target batches contained no target pair inside the 3 m cluster radius.
- No UAV entered FINISH before a global-finish confirmation.
- The cooperative pool was refreshed 54 times in each run, spanning 71–140 grids.
- HELP/reassignment was active (14/28 events in repeat 1 and 11/21 in repeat 2).
- Unreachable reports occurred (4 and 1), but takeover count was 0 in both fair
  30-second runs; the runtime recovery branch therefore needs a longer or
  deliberately blocked-path test for end-to-end takeover validation.
- The 30-second coverage sample was above the paired original baseline in both
  repeats. The short-window slope was higher in one repeat and slightly lower in
  the other; its two-repeat mean was higher.

The raw results and machine-readable comparisons are in:

- `experiments/hybrid_communication_racer_10uav_smoke_30s_final_source_validation_20260830/`
- `experiments/hybrid_communication_racer_10uav_smoke_30s_final_source_repeat2_20260830/`

These are smoke tests, not converged exploration trials. Longer repeated runs are
needed before claiming final-completion coverage or statistical significance.
