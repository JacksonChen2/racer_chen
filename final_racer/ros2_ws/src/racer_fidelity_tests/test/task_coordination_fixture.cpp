#include <exploration_manager/task_coordination_policy.h>

#include <cstdlib>
#include <iostream>
#include <vector>

using fast_planner::TaskCoordinationGate;
using fast_planner::TaskCoordinationMode;
using fast_planner::buildMinimalRecoveryPlan;
using fast_planner::recoveryEpochCanApply;

namespace {
void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << std::endl;
    std::exit(1);
  }
}
}  // namespace

int main() {
  // Case 1: owned work stays in normal pairwise mode and cannot request global.
  TaskCoordinationGate gate;
  gate.configure(3, 2);
  gate.localTaskAvailable();
  require(gate.mode() == TaskCoordinationMode::NORMAL,
      "owned tasks must remain NORMAL");

  // Case 2: the first pair transfer returns directly to normal mode.
  gate.localTaskEmpty();
  gate.pairAttemptStarted();
  gate.pairSucceeded();
  require(gate.mode() == TaskCoordinationMode::NORMAL &&
          gate.pairFailureCount() == 0,
      "successful first pair must not request global recovery");

  // Case 3: only the configured count of failed pair cycles opens recovery.
  gate.localTaskEmpty();
  gate.pairAttemptStarted();
  require(!gate.pairAttemptFailed(), "first pair failure triggered recovery");
  require(!gate.pairAttemptFailed(), "second pair failure triggered recovery");
  require(gate.pairAttemptFailed(), "third pair failure did not trigger recovery");
  require(gate.mode() == TaskCoordinationMode::GLOBAL_RECOVERY,
      "failure threshold did not enter GLOBAL_RECOVERY");
  auto recovered = buildMinimalRecoveryPlan(
      {{10, 11}, {20, 21, 22}, {}}, {10, 11, 20, 21, 22}, {2}, 1,
      [](int, int task) { return static_cast<double>(task); });
  require(recovered.valid && recovered.assignments[0] == std::vector<int>({10, 11}),
      "recovery modified an unrelated active UAV");
  require(recovered.assignments[1].size() == 2 &&
          recovered.assignments[2].size() == 1,
      "recovery did not transfer exactly one donor task");
  gate.globalRecovered();
  require(gate.mode() == TaskCoordinationMode::NORMAL &&
          gate.cooldownRemaining() == 2,
      "successful recovery did not return to NORMAL with cooldown");

  // Case 4: only the selected donor loses ownership; every other route stays exact.
  auto minimal = buildMinimalRecoveryPlan(
      {{1, 2}, {3, 4, 5}, {}, {6}}, {1, 2, 3, 4, 5, 6}, {2}, 1,
      [](int, int task) { return task == 5 ? 0.0 : 1.0; });
  require(minimal.assignments[0] == std::vector<int>({1, 2}) &&
          minimal.assignments[3] == std::vector<int>({6}) &&
          minimal.assignments[1] == std::vector<int>({3, 4}) &&
          minimal.assignments[2] == std::vector<int>({5}),
      "minimal recovery rewrote non-donor ownership");

  // Case 5: a globally unassigned frontier is assigned before any donation.
  auto unassigned = buildMinimalRecoveryPlan(
      {{}, {}, {}}, {42}, {0, 1, 2}, 1,
      [](int idle, int) { return static_cast<double>(idle); });
  require(unassigned.remaining_global_tasks == 1 &&
          unassigned.assignments[0] == std::vector<int>({42}) &&
          unassigned.transfers.front().donor_uav_index == -1,
      "unassigned global frontier was not recovered");

  // Case 6: an empty global pool is a finish candidate; the runtime adds the
  // configured consecutive-confirmation requirement before publishing FINISH.
  auto empty = buildMinimalRecoveryPlan({{}, {}, {}}, {}, {0, 1, 2}, 1, {});
  require(empty.remaining_global_tasks == 0 && empty.active_uavs == 0 &&
          empty.transfers.empty(),
      "empty global pool was not represented as a finish candidate");
  gate.globalFinished();
  require(gate.mode() == TaskCoordinationMode::FINISH,
      "confirmed global completion did not enter FINISH");

  // Case 7: equality or an older recovery epoch cannot replace a newer pair result.
  require(!recoveryEpochCanApply(100, 4, {3, 8, 2}, {3, 7, 2}, {1}),
      "high-numbered stale global epoch overrode a post-snapshot pair commit");
  require(!recoveryEpochCanApply(7, 4, {3, 7, 2}, {3, 7, 2}, {1}),
      "stale global epoch overrode an equal/newer pair epoch");
  require(!recoveryEpochCanApply(6, 4, {3, 7, 2}, {3, 7, 2}, {1}),
      "older global epoch overrode a newer pair epoch");
  require(recoveryEpochCanApply(8, 4, {3, 7, 2}, {3, 7, 2}, {1}),
      "strictly newer recovery epoch was rejected");

  std::cout << "PASS: seven pairwise/global-recovery coordination cases" << std::endl;
  return 0;
}
