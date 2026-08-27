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

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

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
  vector<uint64_t> last_reported_assignment_epochs_;
};

}  // namespace fast_planner

#endif
