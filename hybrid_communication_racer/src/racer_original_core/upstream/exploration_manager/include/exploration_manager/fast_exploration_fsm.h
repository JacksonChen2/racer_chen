#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <exploration_manager/DroneState.h>
#include <exploration_manager/GlobalGridAssignment.h>
#include <exploration_manager/PairOpt.h>
#include <exploration_manager/PairOptResponse.h>
#include <bspline/Bspline.h>
#include <hybrid_communication_racer/msg/hybrid_drone_state.hpp>
#include <hybrid_communication_racer/msg/hybrid_global_assignment.hpp>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, IDLE };

class FastExplorationFSM {

public:
  FastExplorationFSM(/* args */) {
  }
  ~FastExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);
  void visualize(int content);
  void clearVisMarker();
  int getId();
  void findUnallocated(const vector<int>& actives, vector<int>& missed);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void trackingLostCallback(const std_msgs::EmptyConstPtr& msg);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

  // Swarm
  void droneStateTimerCallback(const ros::TimerEvent& e);
  void droneStateMsgCallback(const exploration_manager::DroneStateConstPtr& msg);
  void optTimerCallback(const ros::TimerEvent& e);
  void initialAssignmentTimerCallback(double now_s);
  void globalAssignmentMsgCallback(
      const exploration_manager::GlobalGridAssignmentConstPtr& msg);
  void applyGlobalAssignment(
      const exploration_manager::GlobalGridAssignment& msg);
#ifdef RACER_ORACLE_VARIANT
  void globalAssignmentTimerCallback();
#endif
  void optMsgCallback(const exploration_manager::PairOptConstPtr& msg);
  void optResMsgCallback(const exploration_manager::PairOptResponseConstPtr& msg);
  void swarmTrajCallback(const bspline::BsplineConstPtr& msg);
  void swarmTrajTimerCallback(const ros::TimerEvent& e);
  void publishPairTransaction(uint8_t phase);
  void clearOutgoingTransaction(const char* reason);
  void servicePairTransactions(double now_s);
  bool validateGridIds(const vector<int>& ids, const char* source) const;
  void applyCommittedPairAssignment(int peer_id, const vector<int>& ego_ids,
      const vector<int>& peer_ids, uint64_t new_epoch, double commitment_until);
  void releaseAssignmentForWork(const char* reason);
  bool globalCooperativeMode() const;
  void enterHelpMode(const char* reason, bool clear_assignment = true);
  void reportUnreachableTask(const char* reason);
  void hybridStateTimerCallback();
  void hybridStateMsgCallback(
      const hybrid_communication_racer::msg::HybridDroneState::ConstSharedPtr& msg);
  void globalCooperativeAssignmentTimerCallback();
  void hybridAssignmentMsgCallback(
      const hybrid_communication_racer::msg::HybridGlobalAssignment::ConstSharedPtr& msg);
  void applyHybridAssignment(
      const hybrid_communication_racer::msg::HybridGlobalAssignment& msg);

  struct PairTransaction {
    bool active{ false };
    bool commit_sent{ false };
    int peer_id{ -1 };
    uint64_t transaction_id{ 0 };
    uint64_t from_epoch{ 0 };
    uint64_t to_epoch{ 0 };
    uint64_t new_epoch{ 0 };
    double commitment_until{ 0.0 };
    double expires_at{ 0.0 };
    double last_publish_at{ -1.0 };
    vector<int> ego_ids;
    vector<int> other_ids;
  };

  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_, tracking_lost_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  // Swarm state
  ros::Publisher drone_state_pub_, opt_pub_, opt_res_pub_, swarm_traj_pub_, grid_tour_pub_,
      hgrid_pub_;
  ros::Subscriber drone_state_sub_, opt_sub_, opt_res_sub_, swarm_traj_sub_;
  ros::Timer drone_state_timer_, opt_timer_, swarm_traj_timer_;
  ros::Publisher global_assignment_pub_;
  ros::Subscriber global_assignment_sub_;
  bool initial_partition_enabled_{ true };
  bool initial_partition_sent_{ false };
  double initial_partition_interval_s_{ 0.5 };
  double initial_partition_balance_weight_m_{ 1.0 };
  double last_initial_partition_publish_s_{ -1.0 };
  exploration_manager::GlobalGridAssignment initial_assignment_msg_;
  uint32_t global_assignment_epoch_{ 0 };
  uint32_t last_applied_global_assignment_epoch_{ 0 };
#ifdef RACER_ORACLE_VARIANT
  double global_assignment_interval_s_{0.5};
  double global_balance_weight_m_{1.0};
  double last_global_assignment_time_s_{-1.0};
#endif
  PairTransaction outgoing_transaction_;
  PairTransaction incoming_transaction_;
  uint64_t next_transaction_sequence_{ 1 };
  uint64_t last_committed_transaction_id_{ 0 };
  int last_committed_transaction_peer_{ -1 };
  int consecutive_plan_failures_{ 0 };
  double idle_since_s_{ -1.0 };
  double last_idle_map_wakeup_check_s_{ -1.0 };
  vector<uint64_t> last_reported_assignment_epochs_;

  // Perfect-communication cooperative assignment. The original path above is
  // unchanged when exploration_assignment_mode_ == "original".
  string exploration_assignment_mode_{ "original" };
  ros::Publisher hybrid_state_pub_, hybrid_assignment_pub_;
  ros::Subscriber hybrid_state_sub_, hybrid_assignment_sub_;
  vector<hybrid_communication_racer::msg::HybridDroneState> hybrid_states_;
  vector<uint64_t> last_failure_sequences_;
  double cooperative_assignment_interval_s_{ 0.5 };
  double cooperative_state_freshness_s_{ 0.75 };
  double nominal_velocity_{ 1.5 };
  double utility_gain_scale_{ 0.001 };
  double utility_epsilon_{ 0.1 };
  double lambda_overlap_{ 2.0 };
  double lambda_age_{ 0.05 };
  double overlap_distance_{ 6.0 };
  double cluster_distance_{ 3.0 };
  double distance_min_{ 8.0 };
  double distance_max_{ 45.0 };
  double distance_gamma_{ 0.7 };
  bool distance_fallback_when_idle_{ true };
  int finish_confirmation_cycles_{ 6 };
  int local_failures_before_report_{ 2 };
  int unreachable_distinct_uavs_{ 2 };
  int unreachable_global_failures_{ 4 };
  double unreachable_retry_s_{ 20.0 };
  double last_cooperative_assignment_s_{ -1.0 };
  uint64_t cooperative_assignment_epoch_{ 0 };
  uint64_t last_applied_cooperative_epoch_{ 0 };
  int empty_global_pool_cycles_{ 0 };
  bool cooperative_global_finish_{ false };

  struct FailureRecord {
    int total_failures{ 0 };
    double confirmed_until{ 0.0 };
    std::unordered_map<int, int> failures_by_uav;
    std::unordered_map<int, double> last_failure_by_uav;
  };
  std::unordered_map<int, FailureRecord> failure_records_;
  std::unordered_map<int, double> task_first_seen_s_;
  std::unordered_map<int, int> previous_grid_unknown_;
  std::unordered_map<int, int> previous_grid_owner_;
  vector<double> coverage_contributions_;
  double initial_unknown_voxels_{ 0.0 };
  double global_coverage_ratio_{ 0.0 };

  int primary_grid_id_{ -1 };
  Eigen::Vector3d primary_target_{ Eigen::Vector3d::Zero() };
  double target_utility_{ 0.0 };
  double target_information_gain_{ 0.0 };
  double target_travel_time_{ 0.0 };
  double target_overlap_penalty_{ 0.0 };
  double adaptive_distance_limit_{ 0.0 };
  bool current_target_takeover_{ false };
  int failed_grid_id_{ -1 };
  uint64_t failure_sequence_{ 0 };
  uint64_t reassignment_count_{ 0 };
  uint64_t help_mode_count_{ 0 };
  uint64_t takeover_count_{ 0 };
};

}  // namespace fast_planner

#endif
