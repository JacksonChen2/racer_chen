
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <exploration_manager/HGrid.h>
#include <exploration_manager/GridTour.h>

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_env/multi_map_manager.h>
#include <active_perception/perception_utils.h>
#include <active_perception/hgrid.h>
// #include <active_perception/uniform_grid.h>
// #include <lkh_tsp_solver/lkh_interface.h>
// #include <lkh_mtsp_solver/lkh3_interface.h>

#include <fstream>
#include <limits>
#include <unordered_set>

using Eigen::Vector4d;

namespace fast_planner {
void FastExplorationFSM::init(ros::NodeHandle& nh) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
  nh.param("fsm/attempt_interval", fp_->attempt_interval_, 0.2);
  nh.param("fsm/pair_opt_interval", fp_->pair_opt_interval_, 1.0);
  nh.param("fsm/repeat_send_num", fp_->repeat_send_num_, 10);
  nh.param("fsm/pair_transaction_timeout", fp_->pair_transaction_timeout_, 1.5);
  nh.param("fsm/pair_retry_interval", fp_->pair_retry_interval_, 0.15);
  nh.param("fsm/assignment_commitment_duration", fp_->assignment_commitment_duration_, 3.0);
  nh.param("fsm/pair_min_improvement", fp_->pair_min_improvement_, 1.0);
  nh.param("fsm/pair_switch_penalty", fp_->pair_switch_penalty_, 1.5);
  nh.param("fsm/swarm_state_freshness", fp_->swarm_state_freshness_, 0.5);
  nh.param("fsm/work_steal_failure_threshold", fp_->work_steal_failure_threshold_, 5);
  nh.param("fsm/work_steal_idle_delay", fp_->work_steal_idle_delay_, 0.5);
  nh.param("fsm/initial_partition_enabled", initial_partition_enabled_, true);
  nh.param("fsm/initial_partition_interval", initial_partition_interval_s_, 0.5);
  nh.param("fsm/initial_partition_balance_weight", initial_partition_balance_weight_m_, 2.0);

  /* Initialize main modules */
  expl_manager_.reset(new FastExplorationManager);
  expl_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));

  planner_manager_ = expl_manager_->planner_manager_;
  last_reported_assignment_epochs_.assign(expl_manager_->ep_->drone_num_, 0);
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH", "IDL"
                                                                                              "E" };
  fd_->static_state_ = true;
  fd_->trigger_ = false;
  fd_->avoid_collision_ = false;
  fd_->go_back_ = false;

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this);

  trigger_sub_ =
      nh.subscribe("/move_base_simple/goal", 1, &FastExplorationFSM::triggerCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);
  // ROS2/Isaac execution boundary only: the physical vehicle can be stopped by
  // the independent safety layer while the original ideal tracker advances.
  // Recover the original collision-replan semantics from measured odometry;
  // do not select a new frontier, task, viewpoint, or goal here.
  tracking_lost_sub_ =
      nh.subscribe("/racer/tracking_lost", 1, &FastExplorationFSM::trackingLostCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);

  // Swarm, timer, pub and sub
  drone_state_timer_ =
      nh.createTimer(ros::Duration(0.04), &FastExplorationFSM::droneStateTimerCallback, this);
  drone_state_pub_ =
      nh.advertise<exploration_manager::DroneState>("/swarm_expl/drone_state_send", 10);
  drone_state_sub_ = nh.subscribe(
      "/swarm_expl/drone_state_recv", 10, &FastExplorationFSM::droneStateMsgCallback, this);

  opt_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::optTimerCallback, this);
  opt_pub_ = nh.advertise<exploration_manager::PairOpt>("/swarm_expl/pair_opt_send", 10);
  opt_sub_ = nh.subscribe("/swarm_expl/pair_opt_recv", 100, &FastExplorationFSM::optMsgCallback,
      this, ros::TransportHints().tcpNoDelay());

  opt_res_pub_ =
      nh.advertise<exploration_manager::PairOptResponse>("/swarm_expl/pair_opt_res_send", 10);
  opt_res_sub_ = nh.subscribe("/swarm_expl/pair_opt_res_recv", 100,
      &FastExplorationFSM::optResMsgCallback, this, ros::TransportHints().tcpNoDelay());

  global_assignment_pub_ = nh.advertise<exploration_manager::GlobalGridAssignment>(
      "/swarm_expl/global_assignment_send", 10, true);
  global_assignment_sub_ = nh.subscribe(
      "/swarm_expl/global_assignment_recv", 10,
      &FastExplorationFSM::globalAssignmentMsgCallback, this,
      ros::TransportHints().tcpNoDelay());

#ifdef RACER_ORACLE_VARIANT
  nh.param("oracle_global/assignment_interval", global_assignment_interval_s_, 0.5);
  nh.param("oracle_global/balance_weight_m", global_balance_weight_m_, 1.0);
  ROS_WARN("RACER_ORACLE_GLOBAL_COORDINATION enabled coordinator=1 interval=%.3fs balance=%.3fm",
      global_assignment_interval_s_, global_balance_weight_m_);
#else
  ROS_WARN("RACER_PAIRWISE_ROBUST enabled epoch transactions, initial partition, commitment, "
           "work stealing and known-free connectivity filtering");
#endif

  swarm_traj_pub_ = nh.advertise<bspline::Bspline>("/planning/swarm_traj_send", 100);
  swarm_traj_sub_ =
      nh.subscribe("/planning/swarm_traj_recv", 100, &FastExplorationFSM::swarmTrajCallback, this);
  swarm_traj_timer_ =
      nh.createTimer(ros::Duration(0.1), &FastExplorationFSM::swarmTrajTimerCallback, this);

  hgrid_pub_ = nh.advertise<exploration_manager::HGrid>("/swarm_expl/hgrid_send", 10);
  grid_tour_pub_ = nh.advertise<exploration_manager::GridTour>("/swarm_expl/grid_tour_send", 10);
}

int FastExplorationFSM::getId() {
  return expl_manager_->ep_->drone_id_;
}

bool FastExplorationFSM::validateGridIds(const vector<int>& ids, const char* source) const {
  for (const int id : ids) {
    if (!expl_manager_->hgrid_->isValidGridId(id)) {
      ROS_ERROR("Reject %s with invalid grid id %d (valid range [0,%d)).", source, id,
          expl_manager_->hgrid_->getGridCount());
      return false;
    }
  }
  return true;
}

void FastExplorationFSM::publishPairTransaction(uint8_t phase) {
  if (!outgoing_transaction_.active) return;

  exploration_manager::PairOpt msg;
  msg.from_drone_id = getId();
  msg.to_drone_id = outgoing_transaction_.peer_id;
  msg.phase = phase;
  msg.transaction_id = outgoing_transaction_.transaction_id;
  msg.from_assignment_epoch = outgoing_transaction_.from_epoch;
  msg.to_assignment_epoch = outgoing_transaction_.to_epoch;
  msg.new_assignment_epoch = outgoing_transaction_.new_epoch;
  msg.commitment_until = outgoing_transaction_.commitment_until;
  msg.stamp = ros::Time::now().toSec();
  msg.ego_ids = outgoing_transaction_.ego_ids;
  msg.other_ids = outgoing_transaction_.other_ids;
  for (int i = 0; i < std::max(1, fp_->repeat_send_num_); ++i) opt_pub_.publish(msg);
  outgoing_transaction_.last_publish_at = msg.stamp;
}

void FastExplorationFSM::clearOutgoingTransaction(const char* reason) {
  if (outgoing_transaction_.active)
    ROS_WARN("Pair transaction %lu with UAV %d cleared: %s",
        static_cast<unsigned long>(outgoing_transaction_.transaction_id),
        outgoing_transaction_.peer_id, reason);
  outgoing_transaction_ = PairTransaction();
  expl_manager_->ed_->wait_response_ = false;
}

void FastExplorationFSM::servicePairTransactions(double now_s) {
  if (incoming_transaction_.active && now_s > incoming_transaction_.expires_at) {
    ROS_WARN("Prepared pair transaction %lu expired without commit.",
        static_cast<unsigned long>(incoming_transaction_.transaction_id));
    incoming_transaction_ = PairTransaction();
  }

  if (!outgoing_transaction_.active) return;
  // Before PREPARED it is safe to abandon a proposal. Once COMMIT has been sent, the responder
  // may already have applied it; aborting locally would create split ownership. Keep retrying the
  // idempotent COMMIT until either its ACK or the responder's DroneState confirms the epoch.
  if (!outgoing_transaction_.commit_sent && now_s > outgoing_transaction_.expires_at) {
    clearOutgoingTransaction("timeout");
    return;
  }
  if (outgoing_transaction_.last_publish_at < 0.0 ||
      now_s - outgoing_transaction_.last_publish_at >= fp_->pair_retry_interval_) {
    publishPairTransaction(outgoing_transaction_.commit_sent
            ? exploration_manager::PairOpt::PHASE_COMMIT
            : exploration_manager::PairOpt::PHASE_PROPOSE);
  }
}

void FastExplorationFSM::applyCommittedPairAssignment(int peer_id, const vector<int>& ego_ids,
    const vector<int>& peer_ids, uint64_t new_epoch, double commitment_until) {
  if (peer_id <= 0 || peer_id > static_cast<int>(expl_manager_->ed_->swarm_state_.size()) ||
      !validateGridIds(ego_ids, "committed ego assignment") ||
      !validateGridIds(peer_ids, "committed peer assignment"))
    return;

  auto& ego = expl_manager_->ed_->swarm_state_[getId() - 1];
  auto& peer = expl_manager_->ed_->swarm_state_[peer_id - 1];
  ego.grid_ids_ = ego_ids;
  peer.grid_ids_ = peer_ids;
  ego.assignment_epoch_ = new_epoch;
  peer.assignment_epoch_ = new_epoch;
  ego.commitment_until_ = commitment_until;
  peer.commitment_until_ = commitment_until;
  ego.requesting_work_ = ego.grid_ids_.empty();
  peer.requesting_work_ = peer.grid_ids_.empty();
  ego.recent_interact_time_ = ros::Time::now().toSec();
  peer.recent_interact_time_ = ego.recent_interact_time_;
  expl_manager_->ed_->reallocated_ = true;

  if ((state_ == IDLE || state_ == FINISH) && !ego.grid_ids_.empty()) {
    fd_->go_back_ = false;
    consecutive_plan_failures_ = 0;
    transitState(PLAN_TRAJ, "pairCommit");
    ROS_WARN("UAV %d resumed exploration after committed work transfer.", getId());
  }
}

void FastExplorationFSM::releaseAssignmentForWork(const char* reason) {
  auto& ego = expl_manager_->ed_->swarm_state_[getId() - 1];
  if (outgoing_transaction_.active || incoming_transaction_.active) {
    // A prepared transaction can still be safely abandoned by its initiator, but an uncertain
    // COMMIT must finish before any unilateral assignment change.
    if (outgoing_transaction_.active && !outgoing_transaction_.commit_sent) {
      publishPairTransaction(exploration_manager::PairOpt::PHASE_ABORT);
      clearOutgoingTransaction("local work release before commit");
    } else {
      ego.requesting_work_ = true;
      ROS_WARN_THROTTLE(1.0,
          "UAV %d defers work release until pair transaction %lu is resolved.", getId(),
          static_cast<unsigned long>(outgoing_transaction_.active
                  ? outgoing_transaction_.transaction_id
                  : incoming_transaction_.transaction_id));
      return;
    }
  }
  if (!ego.grid_ids_.empty()) {
    ROS_WARN("UAV %d releases %zu grids for work stealing after %s.",
        getId(), ego.grid_ids_.size(), reason);
    ego.grid_ids_.clear();
    ++ego.assignment_epoch_;
  }
  ego.commitment_until_ = 0.0;
  ego.requesting_work_ = true;
  ego.recent_attempt_time_ = 0.0;
  expl_manager_->ed_->reallocated_ = true;
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  ROS_INFO_STREAM_THROTTLE(
      1.0, "[FSM]: Drone " << getId() << " state: " << fd_->state_str_[int(state_)]);

  switch (state_) {
    case INIT: {
      // Wait for odometry ready
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(1.0, "no odom");
        return;
      }
      if ((ros::Time::now() - fd_->fsm_init_time_).toSec() < 2.0) {
        ROS_WARN_THROTTLE(1.0, "wait for init");
        return;
      }
      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER: {
      // Do nothing but wait for trigger
      ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case FINISH: {
      ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }

    case IDLE: {
      double check_interval = (ros::Time::now() - fd_->last_check_frontier_time_).toSec();
      if (check_interval > 100.0) {
        // if (!expl_manager_->updateFrontierStruct(fd_->odom_pos_)) {
        ROS_WARN("Go back to (0,0,1)");
        // if (getId() == 1) {
        //   expl_manager_->ed_->next_pos_ = Eigen::Vector3d(-3, 1.9, 1);
        // } else
        // expl_manager_->ed_->next_pos_ = Eigen::Vector3d(0, 0.0, 1);
        // Eigen::Vector3d dir = (fd_->start_pos_ - fd_->odom_pos_);
        // expl_manager_->ed_->next_yaw_ = atan2(dir[1], dir[0]);

        expl_manager_->ed_->next_pos_ = fd_->start_pos_;
        expl_manager_->ed_->next_yaw_ = 0.0;

        fd_->go_back_ = true;
        transitState(PLAN_TRAJ, "FSM");
        // } else {
        //   fd_->last_check_frontier_time_ = ros::Time::now();
        // }
      }
      break;
    }

    case PLAN_TRAJ: {
      if (fd_->static_state_) {
        // Plan from static state (hover)
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();
        fd_->start_yaw_ << fd_->odom_yaw_, 0, 0;
      } else {
        // Replan from non-static state, starting from 'replan_time' seconds later
        LocalTrajData* info = &planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }
      // Inform traj_server the replanning
      replan_pub_.publish(std_msgs::Empty());
      int res = callExplorationPlanner();
      if (res == SUCCEED) {
        consecutive_plan_failures_ = 0;
        expl_manager_->ed_->swarm_state_[getId() - 1].requesting_work_ = false;
        transitState(PUB_TRAJ, "FSM");
      } else if (res == FAIL) {  // Keep trying to replan
        fd_->static_state_ = true;
        ++consecutive_plan_failures_;
        ROS_WARN("Plan fail");
        if (consecutive_plan_failures_ >= fp_->work_steal_failure_threshold_) {
          releaseAssignmentForWork("consecutive plan failures");
          fd_->last_check_frontier_time_ = ros::Time::now();
          idle_since_s_ = ros::Time::now().toSec();
          transitState(IDLE, "workSteal");
        }
      } else if (res == NO_GRID) {
        fd_->static_state_ = true;
        fd_->last_check_frontier_time_ = ros::Time::now();
        idle_since_s_ = ros::Time::now().toSec();
        expl_manager_->ed_->swarm_state_[getId() - 1].requesting_work_ = true;
        ROS_WARN("No grid");
        transitState(IDLE, "FSM");
        visualize(1);
        // clearVisMarker();
      }
      break;
    }

    case PUB_TRAJ: {
      double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
      if (dt > 0) {
        bspline_pub_.publish(fd_->newest_traj_);
        fd_->static_state_ = false;

        // fd_->newest_traj_.drone_id = planner_manager_->swarm_traj_data_.drone_id_;
        fd_->newest_traj_.drone_id = expl_manager_->ep_->drone_id_;
        swarm_traj_pub_.publish(fd_->newest_traj_);

        thread vis_thread(&FastExplorationFSM::visualize, this, 2);
        vis_thread.detach();
        transitState(EXEC_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      auto tn = ros::Time::now();
      // Check whether replan is needed
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (tn - info->start_time_).toSec();

      if (!fd_->go_back_) {
        bool need_replan = false;
        if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
          ROS_WARN("Replan: cluster covered=====================================");
          need_replan = true;
        } else if (info->duration_ - t_cur < fp_->replan_thresh1_) {
          // Replan if traj is almost fully executed
          ROS_WARN("Replan: traj fully executed=================================");
          need_replan = true;
        } else if (t_cur > fp_->replan_thresh3_) {
          // Replan after some time
          ROS_WARN("Replan: periodic call=======================================");
          need_replan = true;
        }

        if (need_replan) {
          if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
            // Update frontier and plan new motion
            thread vis_thread(&FastExplorationFSM::visualize, this, 1);
            vis_thread.detach();
            transitState(PLAN_TRAJ, "FSM");
          } else {
            // No frontier detected, finish exploration
            fd_->last_check_frontier_time_ = ros::Time::now();
            idle_since_s_ = ros::Time::now().toSec();
            expl_manager_->ed_->swarm_state_[getId() - 1].requesting_work_ = true;
            transitState(IDLE, "FSM");
            ROS_WARN("Idle since no frontier is detected");
            fd_->static_state_ = true;
            replan_pub_.publish(std_msgs::Empty());
            // clearVisMarker();
            visualize(1);
          }
        }
      } else {
        // Check if reach goal
        auto pos = info->position_traj_.evaluateDeBoorT(t_cur);
        if ((pos - expl_manager_->ed_->next_pos_).norm() < 1.0) {
          replan_pub_.publish(std_msgs::Empty());
          clearVisMarker();
          transitState(FINISH, "FSM");
          return;
        }
        if (t_cur > fp_->replan_thresh3_ || info->duration_ - t_cur < fp_->replan_thresh1_) {
          // Replan for going back
          replan_pub_.publish(std_msgs::Empty());
          transitState(PLAN_TRAJ, "FSM");
          thread vis_thread(&FastExplorationFSM::visualize, this, 1);
          vis_thread.detach();
        }
      }

      break;
    }
  }
}

int FastExplorationFSM::callExplorationPlanner() {
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

  int res;
  if (fd_->avoid_collision_ || fd_->go_back_) {  // Only replan trajectory
    // ROS2/Isaac sensor boundary only: just before the unchanged return
    // planner runs, expose the collision-monitored corridor actually
    // traversed by the physical body.  Keeping this disabled during normal
    // exploration prevents measured tube surfaces from becoming frontiers.
    if (fd_->go_back_) expl_manager_->sdf_map_->prepareReturnCorridor();
    res = expl_manager_->planTrajToView(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
        fd_->start_yaw_, expl_manager_->ed_->next_pos_, expl_manager_->ed_->next_yaw_);
    fd_->avoid_collision_ = false;
  } else {  // Do full planning normally
    res = expl_manager_->planExploreMotion(
        fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);
  }

  if (res == SUCCEED) {
    auto info = &planner_manager_->local_data_;
    info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    fd_->newest_traj_ = bspline;
  }
  return res;
}

void FastExplorationFSM::visualize(int content) {
  // content 1: frontier; 2 paths & trajs
  auto info = &planner_manager_->local_data_;
  auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;

  auto getColorVal = [&](const int& id, const int& num, const int& drone_id) {
    double a = (drone_id - 1) / double(num + 1);
    double b = 1 / double(num + 1);
    return a + b * double(id) / ed_ptr->frontiers_.size();
  };

  if (content == 1) {
    // Draw frontier
    static int last_ftr_num = 0;
    static int last_dftr_num = 0;
    for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
          visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4), "frontier", i, 4);

      // getColorVal(i, expl_manager_->ep_->drone_num_, expl_manager_->ep_->drone_id_)
      // double(i) / ed_ptr->frontiers_.size()

      // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first,
      // ed_ptr->frontier_boxes_[i].second,
      //     Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
    }
    for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
    last_ftr_num = ed_ptr->frontiers_.size();

    // for (int i = 0; i < ed_ptr->dead_frontiers_.size(); ++i)
    //   visualization_->drawCubes(
    //       ed_ptr->dead_frontiers_[i], 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);
    // for (int i = ed_ptr->dead_frontiers_.size(); i < last_dftr_num; ++i)
    //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);
    // last_dftr_num = ed_ptr->dead_frontiers_.size();

    // // Draw updated box
    // Vector3d bmin, bmax;
    // planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax, false);
    // visualization_->drawBox(
    //     (bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0, 4);

    // vector<Eigen::Vector3d> bmins, bmaxs;
    // planner_manager_->edt_environment_->sdf_map_->mm_->getChunkBoxes(bmins, bmaxs, false);
    // for (int i = 0; i < bmins.size(); ++i) {
    //   visualization_->drawBox((bmins[i] + bmaxs[i]) / 2.0, bmaxs[i] - bmins[i],
    //       Vector4d(0, 1, 1, 0.3), "updated_box", i + 1, 4);
    // }

  } else if (content == 2) {

    // Hierarchical grid and global tour --------------------------------
    // vector<Eigen::Vector3d> pts1, pts2;
    // expl_manager_->uniform_grid_->getPath(pts1, pts2);
    // visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0.3, 0, 1), "partition", 0,
    // 6);

    if (expl_manager_->ep_->drone_id_ == 1) {
      vector<Eigen::Vector3d> pts1, pts2;
      expl_manager_->hgrid_->getGridMarker(pts1, pts2);
      visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0, 1, 0.5), "partition", 1, 6);

      vector<Eigen::Vector3d> pts;
      vector<string> texts;
      expl_manager_->hgrid_->getGridMarker2(pts, texts);
      static int last_text_num = 0;
      for (int i = 0; i < pts.size(); ++i) {
        visualization_->drawText(pts[i], texts[i], 1, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
      }
      for (int i = pts.size(); i < last_text_num; ++i) {
        visualization_->drawText(
            Eigen::Vector3d(0, 0, 0), string(""), 1, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
      }
      last_text_num = pts.size();

      // // Pub hgrid to ground node
      // exploration_manager::HGrid hgrid;
      // hgrid.stamp = ros::Time::now().toSec();
      // for (int i = 0; i < pts1.size(); ++i) {
      //   geometry_msgs::Point pt1, pt2;
      //   pt1.x = pts1[i][0];
      //   pt1.y = pts1[i][1];
      //   pt1.z = pts1[i][2];
      //   hgrid.points1.push_back(pt1);
      //   pt2.x = pts2[i][0];
      //   pt2.y = pts2[i][1];
      //   pt2.z = pts2[i][2];
      //   hgrid.points2.push_back(pt2);
      // }
      // hgrid_pub_.publish(hgrid);
    }

    auto grid_tour = expl_manager_->ed_->grid_tour_;
    // auto grid_tour = expl_manager_->ed_->grid_tour2_;
    // for (auto& pt : grid_tour) pt = pt + trans;

    visualization_->drawLines(grid_tour, 0.05,
        PlanningVisualization::getColor(
            (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),
        "grid_tour", 0, 6);

    // Publish grid tour to ground node
    exploration_manager::GridTour tour;
    for (int i = 0; i < grid_tour.size(); ++i) {
      geometry_msgs::Point point;
      point.x = grid_tour[i][0];
      point.y = grid_tour[i][1];
      point.z = grid_tour[i][2];
      tour.points.push_back(point);
    }
    tour.drone_id = expl_manager_->ep_->drone_id_;
    tour.stamp = ros::Time::now().toSec();
    grid_tour_pub_.publish(tour);

    // visualization_->drawSpheres(
    //     expl_manager_->ed_->grid_tour_, 0.3, Eigen::Vector4d(0, 1, 0, 1), "grid_tour", 1, 6);
    // visualization_->drawLines(
    //     expl_manager_->ed_->grid_tour2_, 0.05, Eigen::Vector4d(0, 1, 0, 0.5), "grid_tour", 2, 6);

    // Top viewpoints and frontier tour-------------------------------------

    // visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1), "point-average", 0, 6);

    // auto frontier = ed_ptr->frontier_tour_;
    // for (auto& pt : frontier) pt = pt + trans;
    // visualization_->drawLines(frontier, 0.07,
    //     PlanningVisualization::getColor(
    //         (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_), 0.6),
    //     "frontier_tour", 0, 6);

    // for (int i = 0; i < ed_ptr->other_tours_.size(); ++i) {
    //   visualization_->drawLines(
    //       ed_ptr->other_tours_[i], 0.07, Eigen::Vector4d(0, 0, 1, 1), "other_tours", i, 6);
    // }

    // Locally refined viewpoints and refined tour-------------------------------

    // visualization_->drawSpheres(
    //     ed_ptr->refined_points_, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05, Vector4d(0.5, 0, 1, 1),
    //     "refined_view", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->refined_tour_, 0.07,
    //     PlanningVisualization::getColor(
    //         (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_), 0.6),
    //     "refined_tour", 0, 6);

    // visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0,
    // 0, 0, 1),
    //                           "refined_view", 0, 6);
    // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05,
    // Vector4d(1, 1, 0, 1),
    //                           "refine_pair", 0, 6);
    // for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
    //   visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
    //                               visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
    //                               ed_ptr->frontiers_.size()),
    //                               "n_points", i, 6);
    // for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
    //   visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

    // Trajectory-------------------------------------------

    // visualization_->drawSpheres(
    //     { ed_ptr->next_goal_ /* + trans */ }, 0.3, Vector4d(0, 0, 1, 1), "next_goal", 0, 6);

    // vector<Eigen::Vector3d> next_yaw_vis;
    // next_yaw_vis.push_back(ed_ptr->next_goal_ /* + trans */);
    // next_yaw_vis.push_back(
    //     ed_ptr->next_goal_ /* + trans */ +
    //     2.0 * Eigen::Vector3d(cos(ed_ptr->next_yaw_), sin(ed_ptr->next_yaw_), 0));
    // visualization_->drawLines(next_yaw_vis, 0.1, Eigen::Vector4d(0, 0, 1, 1), "next_goal", 1, 6);
    // visualization_->drawSpheres(
    //     { ed_ptr->next_pos_ /* + trans */ }, 0.3, Vector4d(0, 1, 0, 1), "next_pos", 0, 6);

    // Eigen::MatrixXd ctrl_pt = info->position_traj_.getControlPoint();
    // for (int i = 0; i < ctrl_pt.rows(); ++i) {
    //   for (int j = 0; j < 3; ++j) ctrl_pt(i, j) = ctrl_pt(i, j) + trans[j];
    // }
    // NonUniformBspline position_traj(ctrl_pt, 3, info->position_traj_.getKnotSpan());

    visualization_->drawBspline(info->position_traj_, 0.1,
        PlanningVisualization::getColor(
            (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),
        false, 0.15, Vector4d(1, 1, 0, 1));

    // visualization_->drawLines(
    //     expl_manager_->ed_->path_next_goal_, 0.1, Eigen::Vector4d(0, 1, 0, 1), "astar", 0, 6);
    // visualization_->drawSpheres(
    //     expl_manager_->ed_->kino_path_, 0.1, Eigen::Vector4d(0, 0, 1, 1), "kino", 0, 6);
    // visualization_->drawSpheres(plan_data->kino_path_, 0.1, Vector4d(1, 0, 1, 1), "kino_path", 0,
    // 0); visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1),
    // "next_goal", 1, 6);

    // // Draw trajs of other drones
    // vector<NonUniformBspline> trajs;
    // planner_manager_->swarm_traj_data_.getValidTrajs(trajs);
    // for (int k = 0; k < trajs.size(); ++k) {
    //   visualization_->drawBspline(trajs[k], 0.1, Eigen::Vector4d(1, 1, 0, 1), false, 0.15,
    //       Eigen::Vector4d(0, 0, 1, 1), k + 1);
    // }
  }
}

void FastExplorationFSM::clearVisMarker() {
  for (int i = 0; i < 10; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    // visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "dead_frontier", i, 4);
    // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
    // "frontier_boxes", i, 4);
  }
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "frontier_tour", 0, 6);
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "grid_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

  // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
}

void FastExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  if (state_ == WAIT_TRIGGER) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;

    auto getColorVal = [&](const int& id, const int& num, const int& drone_id) {
      double a = (drone_id - 1) / double(num + 1);
      double b = 1 / double(num + 1);
      return a + b * double(id) / ed->frontiers_.size();
    };

    // ft->searchFrontiers();
    // ft->computeFrontiersToVisit();
    // ft->updateFrontierCostMatrix();

    // ft->getFrontiers(ed->frontiers_);
    // ft->getFrontierBoxes(ed->frontier_boxes_);

    expl_manager_->updateFrontierStruct(fd_->odom_pos_);

    cout << "odom: " << fd_->odom_pos_.transpose() << endl;
    vector<int> tmp_id1;
    vector<vector<int>> tmp_id2;
    bool status = expl_manager_->findGlobalTourOfGrid(
        { fd_->odom_pos_ }, { fd_->odom_vel_ }, tmp_id1, tmp_id2, true);

    // Draw frontier and bounding box
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed->frontiers_[i], 0.1,
          visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4), "frontier", i, 4);
      // getColorVal(i, expl_manager_->ep_->drone_num_, expl_manager_->ep_->drone_id_)
      // double(i) / ed->frontiers_.size()
      // visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
      // Vector4d(0.5, 0, 1, 0.3),
      //                         "frontier_boxes", i, 4);
    }
    for (int i = ed->frontiers_.size(); i < 50; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
    if (status)
      visualize(2);
    else
      visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "grid_tour", 0, 6);

    // Draw grid tour
  }
}

void FastExplorationFSM::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {

  // // Debug traj planner
  // Eigen::Vector3d pos;
  // pos << msg->pose.position.x, msg->pose.position.y, 1;
  // expl_manager_->ed_->next_pos_ = pos;

  // Eigen::Vector3d dir = pos - fd_->odom_pos_;
  // expl_manager_->ed_->next_yaw_ = atan2(dir[1], dir[0]);
  // fd_->go_back_ = true;
  // transitState(PLAN_TRAJ, "triggerCallback");
  // return;

  if (state_ != WAIT_TRIGGER) return;
  fd_->trigger_ = true;
  cout << "Triggered!" << endl;
  fd_->start_pos_ = fd_->odom_pos_;
  ROS_WARN_STREAM("Start expl pos: " << fd_->start_pos_.transpose());

  if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
    transitState(PLAN_TRAJ, "triggerCallback");
  } else
    transitState(FINISH, "triggerCallback");
}

void FastExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Check safety and trigger replan if necessary
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      ROS_WARN("Replan: collision detected==================================");
      fd_->avoid_collision_ = true;
      transitState(PLAN_TRAJ, "safetyCallback");
    }
  }
}

void FastExplorationFSM::trackingLostCallback(const std_msgs::EmptyConstPtr& msg) {
  (void)msg;
  if (state_ != EXPL_STATE::PLAN_TRAJ && state_ != EXPL_STATE::PUB_TRAJ &&
      state_ != EXPL_STATE::EXEC_TRAJ)
    return;

  // Keep the already selected next_pos_/next_yaw_. This only invalidates the
  // unexecuted trajectory and restores a physically valid initial condition.
  replan_pub_.publish(std_msgs::Empty());
  fd_->static_state_ = true;
  fd_->avoid_collision_ = true;
  transitState(PLAN_TRAJ, "trackingLostCallback");
  ROS_WARN("Execution tracking lost: replan current goal from measured odometry");
}

void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  if (!fd_->have_odom_) {
    fd_->have_odom_ = true;
    fd_->fsm_init_time_ = ros::Time::now();
  }
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  ROS_INFO_STREAM("[" + pos_call + "]: Drone "
                  << getId()
                  << " from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]);
}

void FastExplorationFSM::droneStateTimerCallback(const ros::TimerEvent& e) {
  // Broadcast own state periodically
  exploration_manager::DroneState msg;
  msg.drone_id = getId();

  auto& state = expl_manager_->ed_->swarm_state_[msg.drone_id - 1];

  if (fd_->static_state_) {
    state.pos_ = fd_->odom_pos_;
    state.vel_ = fd_->odom_vel_;
    state.yaw_ = fd_->odom_yaw_;
  } else {
    LocalTrajData* info = &planner_manager_->local_data_;
    double t_r = (ros::Time::now() - info->start_time_).toSec();
    state.pos_ = info->position_traj_.evaluateDeBoorT(t_r);
    state.vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
    state.yaw_ = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
  }
  state.stamp_ = ros::Time::now().toSec();
  msg.pos = { float(state.pos_[0]), float(state.pos_[1]), float(state.pos_[2]) };
  msg.vel = { float(state.vel_[0]), float(state.vel_[1]), float(state.vel_[2]) };
  msg.yaw = state.yaw_;
  for (auto id : state.grid_ids_) msg.grid_ids.push_back(id);
  msg.assignment_epoch = state.assignment_epoch_;
  msg.commitment_until = state.commitment_until_;
  msg.requesting_work = state.requesting_work_;
  msg.recent_attempt_time = state.recent_attempt_time_;
  msg.stamp = state.stamp_;

  drone_state_pub_.publish(msg);
}

void FastExplorationFSM::droneStateMsgCallback(const exploration_manager::DroneStateConstPtr& msg) {
  // Update other drones' states
  if (msg->drone_id == getId()) return;
  if (msg->drone_id <= 0 ||
      msg->drone_id > static_cast<int>(expl_manager_->ed_->swarm_state_.size()) ||
      msg->pos.size() < 3 || msg->vel.size() < 3) {
    ROS_ERROR("Reject malformed drone state from id %d.", msg->drone_id);
    return;
  }

  // Simulate swarm communication loss
  Eigen::Vector3d msg_pos(msg->pos[0], msg->pos[1], msg->pos[2]);
  // if ((msg_pos - fd_->odom_pos_).norm() > 6.0) return;

  auto& drone_state = expl_manager_->ed_->swarm_state_[msg->drone_id - 1];
  if (drone_state.stamp_ + 1e-4 >= msg->stamp) return;  // Avoid unordered msg

  drone_state.pos_ = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]);
  drone_state.vel_ = Eigen::Vector3d(msg->vel[0], msg->vel[1], msg->vel[2]);
  drone_state.yaw_ = msg->yaw;
  vector<int> received_ids;
  std::unordered_set<int> received_set;
  for (const auto id : msg->grid_ids)
    if (received_set.insert(id).second) received_ids.push_back(id);
  if (msg->assignment_epoch >= drone_state.assignment_epoch_ &&
      validateGridIds(received_ids, "drone state")) {
    drone_state.grid_ids_ = received_ids;
    drone_state.assignment_epoch_ = msg->assignment_epoch;
    drone_state.commitment_until_ = msg->commitment_until;
    drone_state.requesting_work_ = msg->requesting_work;
  }
  last_reported_assignment_epochs_[msg->drone_id - 1] =
      std::max(last_reported_assignment_epochs_[msg->drone_id - 1], msg->assignment_epoch);
  drone_state.stamp_ = msg->stamp;
  drone_state.recent_attempt_time_ = msg->recent_attempt_time;

  // A COMMITTED response can be lost even though the peer has already committed. Its periodic
  // state broadcast is an independent, versioned confirmation that safely completes our side.
  if (outgoing_transaction_.active && outgoing_transaction_.commit_sent &&
      outgoing_transaction_.peer_id == msg->drone_id &&
      msg->assignment_epoch == outgoing_transaction_.new_epoch &&
      received_ids == outgoing_transaction_.other_ids) {
    applyCommittedPairAssignment(outgoing_transaction_.peer_id,
        outgoing_transaction_.ego_ids, outgoing_transaction_.other_ids,
        outgoing_transaction_.new_epoch, outgoing_transaction_.commitment_until);
    last_committed_transaction_id_ = outgoing_transaction_.transaction_id;
    last_committed_transaction_peer_ = outgoing_transaction_.peer_id;
    clearOutgoingTransaction("committed via state confirmation");
  }

  // std::cout << "Drone " << getId() << " get drone " << int(msg->drone_id) << "'s state" <<
  // std::endl; std::cout << drone_state.pos_.transpose() << std::endl;
}

void FastExplorationFSM::optTimerCallback(const ros::TimerEvent& e) {
  if (state_ == INIT) return;

  auto tn = ros::Time::now().toSec();
  servicePairTransactions(tn);

#ifdef RACER_ORACLE_VARIANT
  globalAssignmentTimerCallback();
  return;
#else
  initialAssignmentTimerCallback(tn);
  // Do not let the original pairwise optimizer race the one-time initial partition. Once the
  // first global epoch is applied, all subsequent allocation remains pairwise.
  if (initial_partition_enabled_ && last_applied_global_assignment_epoch_ == 0) return;
#endif

  if (outgoing_transaction_.active || incoming_transaction_.active) return;

  // Select nearby drone not interacting with recently
  auto& states = expl_manager_->ed_->swarm_state_;
  auto& state1 = states[getId() - 1];
  if ((state_ == IDLE || state_ == FINISH) && idle_since_s_ >= 0.0 &&
      tn - idle_since_s_ >= fp_->work_steal_idle_delay_)
    state1.requesting_work_ = true;
  const bool urgent = state1.grid_ids_.empty() || state1.requesting_work_;

  // Avoid frequent attempt
  if (tn - state1.recent_attempt_time_ < fp_->attempt_interval_) return;
  if (!urgent && tn < state1.commitment_until_) return;

  int select_id = -1;
  double max_interval = -1.0;
  for (int i = 0; i < states.size(); ++i) {
    if (i + 1 == getId()) continue;
    // Preserve RACER's lower-id initiator rule for normal balancing. An idle UAV may contact any
    // peer so high-id agents can also steal work; epoch locking prevents conflicting commits.
    if (!urgent && i + 1 < getId()) continue;
    // Check if have communication recently
    // or the drone just experience another opt
    // or the drone is interacted with recently /* !urgent &&  */
    // or the candidate drone dominates enough grids
    if (tn - states[i].stamp_ > fp_->swarm_state_freshness_) continue;
    if (tn - states[i].recent_attempt_time_ < fp_->attempt_interval_) continue;
    if (tn - states[i].recent_interact_time_ < fp_->pair_opt_interval_) continue;
    if (states[i].grid_ids_.size() + state1.grid_ids_.size() == 0) continue;
    if (!urgent && tn < states[i].commitment_until_) continue;
    if (urgent && states[i].grid_ids_.empty()) continue;

    double interval = urgent ? static_cast<double>(states[i].grid_ids_.size())
                             : tn - states[i].recent_interact_time_;
    if (interval <= max_interval) continue;
    select_id = i + 1;
    max_interval = interval;
  }
  if (select_id == -1) return;

  // Enter cooldown before the expensive ACVRP call. Previously rejected reallocations returned
  // without updating this stamp and were recomputed at the 20 Hz opt-timer rate.
  state1.recent_attempt_time_ = tn;

  std::cout << "\nSelect: " << select_id << std::endl;
  ROS_WARN("Pair opt %d & %d", getId(), select_id);

  // Do pairwise optimization with selected drone, allocate the union of their domiance grids
  unordered_map<int, char> opt_ids_map;
  auto& state2 = states[select_id - 1];
  for (auto id : state1.grid_ids_) opt_ids_map[id] = 1;
  for (auto id : state2.grid_ids_) opt_ids_map[id] = 1;
  vector<int> opt_ids;
  for (auto pair : opt_ids_map) opt_ids.push_back(pair.first);

  std::cout << "Pair Opt id: ";
  for (auto id : opt_ids) std::cout << id << ", ";
  std::cout << "" << std::endl;

  // Find missed grids to reallocated them
  vector<int> actives, missed;
  expl_manager_->hgrid_->getActiveGrids(actives);
  findUnallocated(actives, missed);
  std::cout << "Missed: ";
  for (auto id : missed) std::cout << id << ", ";
  std::cout << "" << std::endl;
  opt_ids.insert(opt_ids.end(), missed.begin(), missed.end());
  std::sort(opt_ids.begin(), opt_ids.end());
  opt_ids.erase(std::unique(opt_ids.begin(), opt_ids.end()), opt_ids.end());
  if (!validateGridIds(opt_ids, "pair optimization input")) return;

  // Do partition of the grid
  vector<Eigen::Vector3d> positions = { state1.pos_, state2.pos_ };
  vector<Eigen::Vector3d> velocities = { Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0) };
  vector<int> first_ids1, second_ids1, first_ids2, second_ids2;
  if (state_ != WAIT_TRIGGER) {
    expl_manager_->hgrid_->getConsistentGrid(
        state1.grid_ids_, state1.grid_ids_, first_ids1, second_ids1);
    expl_manager_->hgrid_->getConsistentGrid(
        state2.grid_ids_, state2.grid_ids_, first_ids2, second_ids2);
  }

  auto t1 = ros::Time::now();

  vector<int> ego_ids, other_ids;
  expl_manager_->allocateGrids(positions, velocities, { first_ids1, first_ids2 },
      { second_ids1, second_ids2 }, opt_ids, ego_ids, other_ids);

  // ACVRP may legally leave an empty route. During work stealing, give the idle UAV the closest
  // transferable grid while leaving at least one grid with its active peer.
  if (urgent && ego_ids.empty() && other_ids.size() > 1) {
    auto best = other_ids.begin();
    double best_dist = std::numeric_limits<double>::infinity();
    for (auto it = other_ids.begin(); it != other_ids.end(); ++it) {
      const double dist = (expl_manager_->hgrid_->getCenter(*it) - state1.pos_).norm();
      if (dist < best_dist) {
        best_dist = dist;
        best = it;
      }
    }
    ego_ids.push_back(*best);
    other_ids.erase(best);
  }

  double alloc_time = (ros::Time::now() - t1).toSec();

  std::cout << "Ego1  : ";
  for (auto id : state1.grid_ids_) std::cout << id << ", ";
  std::cout << "\nOther1: ";
  for (auto id : state2.grid_ids_) std::cout << id << ", ";
  std::cout << "\nEgo2  : ";
  for (auto id : ego_ids) std::cout << id << ", ";
  std::cout << "\nOther2: ";
  for (auto id : other_ids) std::cout << id << ", ";
  std::cout << "" << std::endl;

  // Check results
  double prev_app1 = expl_manager_->computeGridPathCost(state1.pos_, state1.grid_ids_, first_ids1,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
  double prev_app2 = expl_manager_->computeGridPathCost(state2.pos_, state2.grid_ids_, first_ids2,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
  std::cout << "prev cost: " << prev_app1 << ", " << prev_app2 << ", " << prev_app1 + prev_app2
            << std::endl;
  double cur_app1 = expl_manager_->computeGridPathCost(state1.pos_, ego_ids, first_ids1,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
  double cur_app2 = expl_manager_->computeGridPathCost(state2.pos_, other_ids, first_ids2,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
  std::cout << "cur cost : " << cur_app1 << ", " << cur_app2 << ", " << cur_app1 + cur_app2
            << std::endl;
  const double previous_cost = prev_app1 + prev_app2;
  const double switching_cost = urgent ? 0.0 : 2.0 * fp_->pair_switch_penalty_;
  const double proposed_cost = cur_app1 + cur_app2 + switching_cost;
  const double improvement = previous_cost - proposed_cost;
  if ((!urgent && improvement < fp_->pair_min_improvement_) ||
      (urgent && ego_ids.empty())) {
    ROS_WARN_THROTTLE(1.0,
        "Pair reallocation skipped after cooldown: improvement %.3f m, urgent=%d, ego=%zu.",
        improvement, int(urgent), ego_ids.size());
    return;
  }

  if (!state1.grid_ids_.empty() && !ego_ids.empty() &&
      !expl_manager_->hgrid_->isConsistent(state1.grid_ids_[0], ego_ids[0])) {
    ROS_ERROR("Path 1 inconsistent");
  }
  if (!state2.grid_ids_.empty() && !other_ids.empty() &&
      !expl_manager_->hgrid_->isConsistent(state2.grid_ids_[0], other_ids[0])) {
    ROS_ERROR("Path 2 inconsistent");
  }

  if (!validateGridIds(ego_ids, "proposed ego assignment") ||
      !validateGridIds(other_ids, "proposed peer assignment"))
    return;

  outgoing_transaction_ = PairTransaction();
  outgoing_transaction_.active = true;
  outgoing_transaction_.peer_id = select_id;
  outgoing_transaction_.transaction_id =
      (static_cast<uint64_t>(getId()) << 56U) | next_transaction_sequence_++;
  outgoing_transaction_.from_epoch = state1.assignment_epoch_;
  outgoing_transaction_.to_epoch = state2.assignment_epoch_;
  outgoing_transaction_.new_epoch =
      std::max(state1.assignment_epoch_, state2.assignment_epoch_) + 1;
  outgoing_transaction_.commitment_until = tn + fp_->assignment_commitment_duration_;
  outgoing_transaction_.expires_at = tn + fp_->pair_transaction_timeout_;
  outgoing_transaction_.ego_ids = std::move(ego_ids);
  outgoing_transaction_.other_ids = std::move(other_ids);
  expl_manager_->ed_->wait_response_ = true;
  publishPairTransaction(exploration_manager::PairOpt::PHASE_PROPOSE);

  ROS_WARN("UAV %d prepared pair transaction %lu with UAV %d (gain %.3f m, alloc %.3f s).",
      getId(), static_cast<unsigned long>(outgoing_transaction_.transaction_id), select_id,
      improvement, alloc_time);
}

void FastExplorationFSM::initialAssignmentTimerCallback(double now_s) {
  if (!initial_partition_enabled_ || initial_partition_sent_ || getId() != 1 ||
      state_ == WAIT_TRIGGER)
    return;
  if (last_initial_partition_publish_s_ >= 0.0 &&
      now_s - last_initial_partition_publish_s_ < initial_partition_interval_s_)
    return;

  auto& states = expl_manager_->ed_->swarm_state_;
  if (states.empty()) return;
  if (initial_assignment_msg_.epoch != 0) {
    bool all_applied = true;
    for (const auto epoch : last_reported_assignment_epochs_)
      all_applied = all_applied && epoch >= initial_assignment_msg_.epoch;
    if (all_applied) {
      initial_partition_sent_ = true;
      ROS_WARN("Initial multi-UAV partition committed by all %zu UAVs.", states.size());
      return;
    }
    global_assignment_pub_.publish(initial_assignment_msg_);
    last_initial_partition_publish_s_ = now_s;
    return;
  }

  for (const auto& state : states) {
    if (state.stamp_ <= 0.0 || now_s - state.stamp_ > fp_->swarm_state_freshness_) return;
  }

  vector<int> active_grids;
  expl_manager_->hgrid_->getActiveGrids(active_grids);
  std::sort(active_grids.begin(), active_grids.end());
  active_grids.erase(std::unique(active_grids.begin(), active_grids.end()), active_grids.end());
  if (active_grids.empty() || !validateGridIds(active_grids, "initial partition")) return;

  vector<vector<int>> assignments(states.size());
  vector<Eigen::Vector3d> route_ends;
  route_ends.reserve(states.size());
  for (const auto& state : states) route_ends.push_back(state.pos_);

  // Give every UAV one nearby grid first, then greedily extend spatially coherent routes.
  vector<int> remaining = active_grids;
  for (std::size_t drone = 0; drone < states.size() && !remaining.empty(); ++drone) {
    auto best = remaining.begin();
    double best_dist = std::numeric_limits<double>::infinity();
    for (auto it = remaining.begin(); it != remaining.end(); ++it) {
      const double dist = (route_ends[drone] - expl_manager_->hgrid_->getCenter(*it)).norm();
      if (dist < best_dist) {
        best_dist = dist;
        best = it;
      }
    }
    assignments[drone].push_back(*best);
    route_ends[drone] = expl_manager_->hgrid_->getCenter(*best);
    remaining.erase(best);
  }
  for (const int grid_id : remaining) {
    const Eigen::Vector3d center = expl_manager_->hgrid_->getCenter(grid_id);
    int best_drone = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    for (int drone = 0; drone < static_cast<int>(states.size()); ++drone) {
      const double cost = (route_ends[drone] - center).norm() +
          initial_partition_balance_weight_m_ * assignments[drone].size();
      if (cost < best_cost) {
        best_cost = cost;
        best_drone = drone;
      }
    }
    assignments[best_drone].push_back(grid_id);
    route_ends[best_drone] = center;
  }

  initial_assignment_msg_ = exploration_manager::GlobalGridAssignment();
  initial_assignment_msg_.coordinator_id = 1;
  initial_assignment_msg_.stamp = now_s;
  initial_assignment_msg_.epoch = ++global_assignment_epoch_;
  initial_assignment_msg_.offsets.push_back(0);
  for (const auto& assignment : assignments) {
    initial_assignment_msg_.grid_ids.insert(initial_assignment_msg_.grid_ids.end(),
        assignment.begin(), assignment.end());
    initial_assignment_msg_.offsets.push_back(
        static_cast<int32_t>(initial_assignment_msg_.grid_ids.size()));
  }
  applyGlobalAssignment(initial_assignment_msg_);
  global_assignment_pub_.publish(initial_assignment_msg_);
  last_initial_partition_publish_s_ = now_s;
  ROS_WARN("Initial spatial partition epoch=%u assigned %zu grids to %zu UAVs.",
      initial_assignment_msg_.epoch, active_grids.size(), states.size());
}

#ifdef RACER_ORACLE_VARIANT
void FastExplorationFSM::globalAssignmentTimerCallback() {
  if (getId() != 1 || state_ == WAIT_TRIGGER) return;

  const double now_s = ros::Time::now().toSec();
  if (last_global_assignment_time_s_ >= 0.0 &&
      now_s - last_global_assignment_time_s_ < global_assignment_interval_s_) {
    return;
  }

  auto& states = expl_manager_->ed_->swarm_state_;
  if (states.empty()) return;
  for (const auto& state : states) {
    if (state.stamp_ <= 0.0 || now_s - state.stamp_ > 0.2) return;
  }

  vector<int> active_grids;
  expl_manager_->hgrid_->getActiveGrids(active_grids);
  if (active_grids.empty()) return;
  std::sort(active_grids.begin(), active_grids.end());
  active_grids.erase(std::unique(active_grids.begin(), active_grids.end()),
      active_grids.end());

  vector<vector<int>> assignments(states.size());
  vector<Eigen::Vector3d> route_ends;
  route_ends.reserve(states.size());
  for (const auto& state : states) route_ends.push_back(state.pos_);

  // Greedy global allocation over the shared HGrid.  The route-end update
  // rewards spatially coherent assignments, while the load term prevents a
  // single nearby UAV from claiming the whole map.
  for (const int grid_id : active_grids) {
    const Eigen::Vector3d center = expl_manager_->hgrid_->getCenter(grid_id);
    int best_drone = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    for (int drone = 0; drone < static_cast<int>(states.size()); ++drone) {
      const double cost = (route_ends[drone] - center).norm() +
          global_balance_weight_m_ * assignments[drone].size();
      if (cost < best_cost) {
        best_cost = cost;
        best_drone = drone;
      }
    }
    assignments[best_drone].push_back(grid_id);
    route_ends[best_drone] = center;
  }

  exploration_manager::GlobalGridAssignment msg;
  msg.coordinator_id = 1;
  msg.stamp = now_s;
  msg.epoch = ++global_assignment_epoch_;
  msg.offsets.reserve(assignments.size() + 1);
  msg.offsets.push_back(0);
  for (const auto& assignment : assignments) {
    msg.grid_ids.insert(msg.grid_ids.end(), assignment.begin(), assignment.end());
    msg.offsets.push_back(static_cast<int32_t>(msg.grid_ids.size()));
  }

  applyGlobalAssignment(msg);
  global_assignment_pub_.publish(msg);
  last_global_assignment_time_s_ = now_s;
  ROS_WARN_THROTTLE(2.0,
      "RACER_ORACLE_GLOBAL_ASSIGNMENT epoch=%u active=%zu drones=%zu",
      msg.epoch, active_grids.size(), assignments.size());
}
#endif

void FastExplorationFSM::globalAssignmentMsgCallback(
    const exploration_manager::GlobalGridAssignmentConstPtr& msg) {
  if (!msg || msg->coordinator_id != 1) return;
  applyGlobalAssignment(*msg);
}

void FastExplorationFSM::applyGlobalAssignment(
    const exploration_manager::GlobalGridAssignment& msg) {
  auto& states = expl_manager_->ed_->swarm_state_;
  if (msg.epoch <= last_applied_global_assignment_epoch_ ||
      msg.offsets.size() != states.size() + 1 || msg.offsets.front() != 0 ||
      msg.offsets.back() != static_cast<int32_t>(msg.grid_ids.size())) {
    return;
  }
  vector<int> all_grid_ids(msg.grid_ids.begin(), msg.grid_ids.end());
  if (!validateGridIds(all_grid_ids, "global assignment")) return;
  auto unique_grid_ids = all_grid_ids;
  std::sort(unique_grid_ids.begin(), unique_grid_ids.end());
  if (std::adjacent_find(unique_grid_ids.begin(), unique_grid_ids.end()) !=
      unique_grid_ids.end()) {
    ROS_ERROR("Reject global assignment epoch %u with duplicate grid ownership.", msg.epoch);
    return;
  }
  for (std::size_t drone = 0; drone < states.size(); ++drone) {
    const int32_t begin = msg.offsets[drone];
    const int32_t end = msg.offsets[drone + 1];
    if (begin < 0 || end < begin ||
        end > static_cast<int32_t>(msg.grid_ids.size())) return;
  }

  bool ego_changed = false;
  const double commitment_until = msg.stamp + fp_->assignment_commitment_duration_;
  for (std::size_t drone = 0; drone < states.size(); ++drone) {
    const auto begin = msg.grid_ids.begin() + msg.offsets[drone];
    const auto end = msg.grid_ids.begin() + msg.offsets[drone + 1];
    vector<int> next(begin, end);
    if (static_cast<int>(drone) + 1 == getId() &&
        next != states[drone].grid_ids_) {
      ego_changed = true;
    }
    states[drone].grid_ids_ = std::move(next);
    states[drone].recent_interact_time_ = msg.stamp;
    states[drone].assignment_epoch_ = msg.epoch;
    states[drone].commitment_until_ = commitment_until;
    states[drone].requesting_work_ = states[drone].grid_ids_.empty();
  }
  outgoing_transaction_ = PairTransaction();
  incoming_transaction_ = PairTransaction();
  last_applied_global_assignment_epoch_ = msg.epoch;
  last_reported_assignment_epochs_[getId() - 1] = msg.epoch;
  if (ego_changed) {
    expl_manager_->ed_->reallocated_ = true;
    if ((state_ == IDLE || state_ == FINISH) && !states[getId() - 1].grid_ids_.empty()) {
      fd_->go_back_ = false;
      consecutive_plan_failures_ = 0;
      transitState(PLAN_TRAJ, "globalAssignmentMsgCallback");
    }
  }
}

void FastExplorationFSM::findUnallocated(const vector<int>& actives, vector<int>& missed) {
  // Create map of all active
  unordered_map<int, char> active_map;
  for (auto ativ : actives) {
    active_map[ativ] = 1;
  }

  // Remove allocated ones
  for (auto state : expl_manager_->ed_->swarm_state_) {
    for (auto id : state.grid_ids_) {
      if (active_map.find(id) != active_map.end()) {
        active_map.erase(id);
      } else {
        // ROS_ERROR("Inactive grid %d is allocated.", id);
      }
    }
  }

  missed.clear();
  for (auto p : active_map) {
    missed.push_back(p.first);
  }
}

void FastExplorationFSM::optMsgCallback(const exploration_manager::PairOptConstPtr& msg) {
  if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) return;
  if (msg->from_drone_id <= 0 ||
      msg->from_drone_id > static_cast<int>(expl_manager_->ed_->swarm_state_.size()))
    return;

  auto send_response = [&](int32_t status) {
    exploration_manager::PairOptResponse response;
    response.from_drone_id = getId();
    response.to_drone_id = msg->from_drone_id;
    response.status = status;
    response.stamp = ros::Time::now().toSec();
    response.transaction_id = msg->transaction_id;
    response.responder_assignment_epoch =
        expl_manager_->ed_->swarm_state_[getId() - 1].assignment_epoch_;
    for (int i = 0; i < std::max(1, fp_->repeat_send_num_); ++i)
      opt_res_pub_.publish(response);
  };

  if (msg->phase == exploration_manager::PairOpt::PHASE_ABORT) {
    if (incoming_transaction_.active &&
        incoming_transaction_.transaction_id == msg->transaction_id)
      incoming_transaction_ = PairTransaction();
    send_response(exploration_manager::PairOptResponse::STATUS_REJECTED);
    return;
  }

  vector<int> proposer_ids;
  vector<int> responder_ids;
  std::unordered_set<int> proposer_set;
  std::unordered_set<int> responder_set;
  bool duplicate_id = false;
  for (const auto id : msg->ego_ids)
    if (proposer_set.insert(id).second)
      proposer_ids.push_back(id);
    else
      duplicate_id = true;
  for (const auto id : msg->other_ids)
    if (responder_set.insert(id).second)
      responder_ids.push_back(id);
    else
      duplicate_id = true;
  if (duplicate_id || !validateGridIds(proposer_ids, "pair proposal") ||
      !validateGridIds(responder_ids, "pair proposal")) {
    send_response(exploration_manager::PairOptResponse::STATUS_REJECTED);
    return;
  }
  std::size_t overlap_count = 0;
  for (const int id : proposer_set)
    if (responder_set.find(id) != responder_set.end()) ++overlap_count;
  if (overlap_count != 0) {
    ROS_ERROR("Reject pair transaction %lu: %zu grids have two owners.",
        static_cast<unsigned long>(msg->transaction_id), overlap_count);
    send_response(exploration_manager::PairOptResponse::STATUS_REJECTED);
    return;
  }

  auto& proposer = expl_manager_->ed_->swarm_state_[msg->from_drone_id - 1];
  auto& responder = expl_manager_->ed_->swarm_state_[getId() - 1];

  if (msg->phase == exploration_manager::PairOpt::PHASE_PROPOSE) {
    if (last_committed_transaction_id_ == msg->transaction_id &&
        last_committed_transaction_peer_ == msg->from_drone_id) {
      send_response(exploration_manager::PairOptResponse::STATUS_COMMITTED);
      return;
    }
    if (incoming_transaction_.active) {
      send_response(incoming_transaction_.transaction_id == msg->transaction_id
              ? exploration_manager::PairOptResponse::STATUS_PREPARED
              : exploration_manager::PairOptResponse::STATUS_BUSY);
      return;
    }
    const double now_s = ros::Time::now().toSec();
    const bool work_steal = proposer.requesting_work_ || responder.requesting_work_ ||
        proposer.grid_ids_.empty() || responder.grid_ids_.empty();
    if (msg->new_assignment_epoch !=
            std::max(msg->from_assignment_epoch, msg->to_assignment_epoch) + 1 ||
        (!work_steal &&
            (now_s < proposer.commitment_until_ || now_s < responder.commitment_until_)) ||
        outgoing_transaction_.active || proposer.assignment_epoch_ != msg->from_assignment_epoch ||
        responder.assignment_epoch_ != msg->to_assignment_epoch) {
      send_response(exploration_manager::PairOptResponse::STATUS_BUSY);
      return;
    }

    // The transaction must preserve the complete pair-owned union, plus only grids that this UAV
    // also sees as globally unallocated. This prevents a stale initiator from creating a third
    // UAV's duplicate ownership while still allowing RACER's missed-grid recovery.
    vector<int> active_grids, missed_grids;
    expl_manager_->hgrid_->getActiveGrids(active_grids);
    findUnallocated(active_grids, missed_grids);
    std::unordered_set<int> expected_union(
        proposer.grid_ids_.begin(), proposer.grid_ids_.end());
    expected_union.insert(responder.grid_ids_.begin(), responder.grid_ids_.end());
    expected_union.insert(missed_grids.begin(), missed_grids.end());
    std::unordered_set<int> proposed_union = proposer_set;
    proposed_union.insert(responder_set.begin(), responder_set.end());
    if (proposed_union != expected_union) {
      ROS_WARN("Reject pair transaction %lu: proposed grid union is stale or incomplete.",
          static_cast<unsigned long>(msg->transaction_id));
      send_response(exploration_manager::PairOptResponse::STATUS_BUSY);
      return;
    }

    incoming_transaction_ = PairTransaction();
    incoming_transaction_.active = true;
    incoming_transaction_.peer_id = msg->from_drone_id;
    incoming_transaction_.transaction_id = msg->transaction_id;
    incoming_transaction_.from_epoch = msg->from_assignment_epoch;
    incoming_transaction_.to_epoch = msg->to_assignment_epoch;
    incoming_transaction_.new_epoch = msg->new_assignment_epoch;
    incoming_transaction_.commitment_until = msg->commitment_until;
    incoming_transaction_.expires_at =
        ros::Time::now().toSec() + fp_->pair_transaction_timeout_;
    incoming_transaction_.ego_ids = std::move(proposer_ids);
    incoming_transaction_.other_ids = std::move(responder_ids);
    responder.recent_attempt_time_ = ros::Time::now().toSec();
    send_response(exploration_manager::PairOptResponse::STATUS_PREPARED);
    return;
  }

  if (msg->phase != exploration_manager::PairOpt::PHASE_COMMIT) {
    send_response(exploration_manager::PairOptResponse::STATUS_REJECTED);
    return;
  }
  if (last_committed_transaction_id_ == msg->transaction_id &&
      last_committed_transaction_peer_ == msg->from_drone_id) {
    send_response(exploration_manager::PairOptResponse::STATUS_COMMITTED);
    return;
  }
  if (!incoming_transaction_.active ||
      incoming_transaction_.transaction_id != msg->transaction_id ||
      proposer.assignment_epoch_ != incoming_transaction_.from_epoch ||
      responder.assignment_epoch_ != incoming_transaction_.to_epoch) {
    send_response(exploration_manager::PairOptResponse::STATUS_REJECTED);
    return;
  }

  applyCommittedPairAssignment(msg->from_drone_id, incoming_transaction_.other_ids,
      incoming_transaction_.ego_ids, incoming_transaction_.new_epoch,
      incoming_transaction_.commitment_until);
  last_committed_transaction_id_ = incoming_transaction_.transaction_id;
  last_committed_transaction_peer_ = incoming_transaction_.peer_id;
  incoming_transaction_ = PairTransaction();
  send_response(exploration_manager::PairOptResponse::STATUS_COMMITTED);
}

void FastExplorationFSM::optResMsgCallback(
    const exploration_manager::PairOptResponseConstPtr& msg) {
  if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) return;
  if (!outgoing_transaction_.active ||
      msg->from_drone_id != outgoing_transaction_.peer_id ||
      msg->transaction_id != outgoing_transaction_.transaction_id)
    return;

  if (msg->status == exploration_manager::PairOptResponse::STATUS_PREPARED) {
    if (outgoing_transaction_.commit_sent) return;
    const auto& ego = expl_manager_->ed_->swarm_state_[getId() - 1];
    if (ego.assignment_epoch_ != outgoing_transaction_.from_epoch ||
        msg->responder_assignment_epoch != outgoing_transaction_.to_epoch) {
      publishPairTransaction(exploration_manager::PairOpt::PHASE_ABORT);
      clearOutgoingTransaction("responder epoch changed before commit");
      return;
    }
    outgoing_transaction_.commit_sent = true;
    outgoing_transaction_.expires_at =
        ros::Time::now().toSec() + fp_->pair_transaction_timeout_;
    publishPairTransaction(exploration_manager::PairOpt::PHASE_COMMIT);
    return;
  }

  if (msg->status == exploration_manager::PairOptResponse::STATUS_COMMITTED) {
    applyCommittedPairAssignment(outgoing_transaction_.peer_id, outgoing_transaction_.ego_ids,
        outgoing_transaction_.other_ids, outgoing_transaction_.new_epoch,
        outgoing_transaction_.commitment_until);
    last_committed_transaction_id_ = outgoing_transaction_.transaction_id;
    last_committed_transaction_peer_ = outgoing_transaction_.peer_id;
    clearOutgoingTransaction("committed");
    return;
  }

  // Once COMMIT is in flight, a delayed BUSY/REJECTED response to the earlier PROPOSE cannot be
  // treated as an abort. The peer may already have committed; keep reconciling idempotently.
  if (outgoing_transaction_.commit_sent) return;

  clearOutgoingTransaction(msg->status == exploration_manager::PairOptResponse::STATUS_BUSY
          ? "peer busy or epoch mismatch"
          : "proposal rejected");
}

void FastExplorationFSM::swarmTrajCallback(const bspline::BsplineConstPtr& msg) {
  // Get newest trajs from other drones, for inter-drone collision avoidance
  auto& sdat = planner_manager_->swarm_traj_data_;

  // Ignore self trajectory
  if (msg->drone_id == sdat.drone_id_) return;

  // Ignore outdated trajectory
  if (sdat.receive_flags_[msg->drone_id - 1] == true &&
      ros::Time(msg->start_time).toSec() <=
        sdat.swarm_trajs_[msg->drone_id - 1].start_time_ + 1e-3)
    return;

  // Convert the msg to B-spline
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];

  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  // // Transform of drone's basecoor, optional step (skip if use swarm_pilot)
  // Eigen::Vector4d tf;
  // planner_manager_->edt_environment_->sdf_map_->getBaseCoor(msg->drone_id, tf);
  // double yaw = tf[3];
  // Eigen::Matrix3d rot;
  // rot << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  // Eigen::Vector3d trans = tf.head<3>();
  // for (int i = 0; i < pos_pts.rows(); ++i) {
  //   Eigen::Vector3d tmp = pos_pts.row(i);
  //   tmp = rot * tmp + trans;
  //   pos_pts.row(i) = tmp;
  // }

  sdat.swarm_trajs_[msg->drone_id - 1].setUniformBspline(pos_pts, msg->order, 0.1);
  sdat.swarm_trajs_[msg->drone_id - 1].setKnot(knots);
  sdat.swarm_trajs_[msg->drone_id - 1].start_time_ = ros::Time(msg->start_time).toSec();
  sdat.receive_flags_[msg->drone_id - 1] = true;

  if (state_ == EXEC_TRAJ) {
    // Check collision with received trajectory
    if (!planner_manager_->checkSwarmCollision(msg->drone_id)) {
      ROS_ERROR("Drone %d collide with drone %d.", sdat.drone_id_, msg->drone_id);
      fd_->avoid_collision_ = true;
      transitState(PLAN_TRAJ, "swarmTrajCallback");
    }
  }
}

void FastExplorationFSM::swarmTrajTimerCallback(const ros::TimerEvent& e) {
  // Broadcast newest traj of this drone to others
  if (state_ == EXEC_TRAJ) {
    swarm_traj_pub_.publish(fd_->newest_traj_);

  } else if (state_ == WAIT_TRIGGER) {
    // Publish a virtual traj at current pose, to avoid collision
    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = ros::Time::now();
    bspline.traj_id = planner_manager_->local_data_.traj_id_;

    Eigen::MatrixXd pos_pts(4, 3);
    for (int i = 0; i < 4; ++i) pos_pts.row(i) = fd_->odom_pos_.transpose();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    NonUniformBspline tmp(pos_pts, planner_manager_->pp_.bspline_degree_, 1.0);
    Eigen::VectorXd knots = tmp.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    bspline.drone_id = expl_manager_->ep_->drone_id_;
    swarm_traj_pub_.publish(bspline);
  }
}

}  // namespace fast_planner
