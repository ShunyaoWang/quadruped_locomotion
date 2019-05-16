/*
 *  test_foothold_optimization.cpp
 *  Descriotion:
 *
 *  Created on: 24, Apr, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#include "free_gait_ros/FootstepOptimization.hpp"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "foothold_optimization_test_node");
  ros::NodeHandle nh("~");
  FootstepOptimization footstepOptimiztion(nh);
//  footstepOptimiztion.initialize();
  ros::Rate rate(100);
  free_gait::State robot_state;
  robot_state.setPoseBaseToWorld(Pose(Position(4,-3,0.4), RotationQuaternion(1,0,0,0)));

  Position foothold_in_world(4.5, -3.175, 0);
  while (ros::ok()) {
      footstepOptimiztion.getOptimizedFoothold(foothold_in_world, robot_state, free_gait::LimbEnum::LF_LEG,"odom");
      ROS_INFO_STREAM("Optimized Foot hold position "<<foothold_in_world<<std::endl);
      rate.sleep();
      ros::spinOnce();
    }
  return 0;
}
