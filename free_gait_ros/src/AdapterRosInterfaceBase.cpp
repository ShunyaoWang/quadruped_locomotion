/*
 * AdapterRosInterfaceBase.cpp
 *
 *  Created on: Nov 29, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <free_gait_ros/AdapterRosInterfaceBase.hpp>

#include <pluginlib/pluginlib_exceptions.h>


namespace free_gait {

AdapterRosInterfaceBase::AdapterRosInterfaceBase()
    : nodeHandle_(NULL)
{
}

AdapterRosInterfaceBase::~AdapterRosInterfaceBase()
{
}

void AdapterRosInterfaceBase::setNodeHandle(ros::NodeHandle& nodeHandle)
{
  nodeHandle_ = &nodeHandle;
}

bool AdapterRosInterfaceBase::readRobotDescription()
{
  std::string robotDescriptionPath;
  if (nodeHandle_->hasParam("/free_gait/robot_description")) {
    nodeHandle_->getParam("/free_gait/robot_description", robotDescriptionPath);
  } else {
    throw pluginlib::PluginlibException("Did not find ROS parameter for robot description '/free_gait/robot_description'.");
  }
  robotDescriptionUrdfString_ = robotDescriptionPath;
//  ROS_INFO("=================================================");
//  if (nodeHandle_->hasParam(robotDescriptionPath)) {
//    nodeHandle_->getParam(robotDescriptionPath, robotDescriptionUrdfString_);
//  } else {
//    throw pluginlib::PluginlibException("Did not find ROS parameter for robot description '" + robotDescriptionPath + "'.");
//  }
//  ROS_INFO("=================================================");
  return true;
}
bool AdapterRosInterfaceBase::updateAdapterWithRobotState(AdapterBase& adapter) const
{
  // TODO(Shunyao): using Extra Feedback
//  State state = adapter.getState();
//  state.setAllJointPositions(all_joint_positions_);

}

bool AdapterRosInterfaceBase::subscribeToRobotState(const std::string& robotStateTopic)
{
  // topic: "/free_gait/robot_state",
  //joint_states_sub_ = nodeHandle_->subscribe("/robot_state", 1, &AdapterRosInterfaceBase::updateRobotState,this);

}

//void AdapterRosInterfaceBase::updateRobotState(const free_gait_msgs::RobotStateConstPtr& robotState)
//{
//  for(int i = 1;i<12;i++)
//  {
//    all_joint_positions_(i) = robotState->lf_leg_joints.position[i];
//  }

//}

} /* namespace free_gait */
