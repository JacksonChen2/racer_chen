# Safety Supervisor audit and performance regression

Date: 2026-08-28

## Finding

The Isaac Sim `Safety Supervisor` is an execution-boundary mechanism added by
the simulator adapter. It does not exist in the original ROS1 RACER tree. The
original RACER collision-avoidance chain remains compiled and active:

1. Mapping clouds enter `SDFMap::inputPointCloud`; the ROS2 map port performs
   occupancy inflation and calls `updateESDF3d`.
2. Kinodynamic A* rejects inflated occupied states.
3. B-spline optimization evaluates ESDF distance and gradient costs.
4. While executing, the 20 Hz FSM safety timer calls
   `FastPlannerManager::checkTrajCollision`, samples the future B-spline every
   0.02 s for up to 6 m, and transitions to `PLAN_TRAJ` with
   `avoid_collision_ = true` when inflated occupancy is encountered.
5. Swarm trajectory conflicts also trigger `PLAN_TRAJ`.

The current `sdf_map.cpp`, `kinodynamic_astar.cpp`, and
`bspline_optimizer.cpp` are byte-identical to the original ROS1 files. The
`checkTrajCollision` and `safetyCallback` function bodies are also identical.

## Change

The dense mapping-camera point cloud is no longer copied into an extra safety
history. The old history concatenation, `np.unique`, and nearest-2000-point
path has been removed.

The additional Isaac execution supervisor now receives only the current
low-density 360-degree Warp safety scan:

- 2.5 degree horizontal by 5 degree vertical spacing (5,184 rays/UAV);
- one batched GPU mesh-query launch for all UAVs;
- range filtering and 0.08 m voxelization/compaction on the GPU;
- at most 1,200 current points per UAV passed to the supervisor;
- no safety-ray points are published to RACER's mapping `PointCloud2` topic.

The PhysX swept-sphere backstop, scene-query constraints, geofence and peer
CBF remain active. No RACER map, ESDF, planner, safety-distance or replanning
parameter was changed for this work.

## Profiling

Profiles now separately record:

- `safety_raycasting_ms`;
- `safety_pointcloud_process_ms`;
- `RACER_3D_SAFETY_DECISION_PROFILE` and final `safety_decision_profile`.

## Matched 10-UAV, 50-second Sionna regression

Both runs used the same starts, seed, communication settings, 76,800 mapping
rays/UAV, 10 Hz sensor rate and 100 Hz physics rate.

| Metric | Before | After |
|---|---:|---:|
| Wall clock | 692 s | 356.39 s |
| Sensor postprocess, mean/batch | 690.754 ms | 8.037 ms |
| Sensor total, mean/batch | 702.453 ms | 21.548 ms |
| Safety raycasting, mean/batch | not separated | 0.832 ms |
| Safety point processing, mean/batch | included above | 0.817 ms |
| Safety decision, mean/control batch | not separated | 36.344 ms |
| Collision events | 0 | 0 |
| Physics contact events | 0 | 0 |
| Signed minimum obstacle clearance | -0.1254 m | -0.0554 m |
| Minimum swept-sphere free travel | 0.2079 m | 0.3369 m |
| Minimum inter-UAV distance | 1.3208 m | 1.3198 m |
| Safety interventions | 3,806 | 4,715 |

Wall time fell by 48.5% (1.94x faster), sensor postprocessing by 98.84%
(85.95x), and total sensor-batch time by 96.93% (32.60x). Safety
interventions increased rather than being disabled. The signed-clearance
metric improved by 0.070 m, while both physical contact counters stayed zero.

The signed obstacle-clearance metric is computed from execution-safety hit
points relative to the modeled vehicle radius. A negative value is not itself
a contact event; contact sensors and physics contact counters remained zero in
both runs.
