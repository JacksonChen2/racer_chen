# Validation status

Status: **NOT RUN**

The repository owner explicitly requested source changes without local
verification. This Windows machine does not provide the target Ubuntu 22.04,
ROS 2 Humble or Isaac Sim 5.1 environment. Therefore none of the following
claims are made:

- colcon build success;
- Python import or lint success;
- ROS 2 interface generation success;
- topic/QoS compatibility in a live graph;
- Isaac action-graph compatibility with a selected UAV asset;
- trajectory feasibility or collision-free closed-loop operation;
- behavioral or performance equivalence with upstream RACER.

The source contains the intended migration and target instructions, but
“implemented” must not be read as “validated.” Before flight or publication,
run the following gates on the Linux target:

1. `colcon build --symlink-install` and package import tests.
2. Message/service introspection and single-node smoke tests.
3. Recorded-map numerical comparison for frontier sets, cost matrices,
   selected tours and B-spline samples.
4. Isaac single-UAV sensor/command loop.
5. Three-UAV map-loss/replay and asynchronous communication tests.
6. Collision clearance, velocity, acceleration, yaw-rate and completion-time
   regression thresholds against the ROS 1 baseline.

