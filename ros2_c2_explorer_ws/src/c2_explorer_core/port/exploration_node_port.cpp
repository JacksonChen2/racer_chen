#include <exploration_manager/c2_exploration_fsm.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "c2_explorer_node");
  ros::NodeHandle node("~");
  c2_expl::C2ExplorationFSM fsm;
  fsm.init(node);
  ros::spin();
  return 0;
}
