/*
 *  AdapterRosInterfaceDummy.cpp
 *  Descriotion: Adapter Ros interface with Dummy states, use for a visulization
 *               in Rviz
 *
 *  Created on: Dec 26, 2018
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#include "free_gait_ros/AdapterRosInterfaceDummy.hpp"

namespace free_gait {

AdapterRosInterfaceDummy::AdapterRosInterfaceDummy()
{

}

AdapterRosInterfaceDummy::~AdapterRosInterfaceDummy()
{

}

bool AdapterRosInterfaceDummy::subscribeToRobotState(const std::string& robotStateTopic)
{
  joint_states_sub_ = nodeHandle_->subscribe(robotStateTopic, 1, &AdapterRosInterfaceDummy::updateRobotState,this);
}
void AdapterRosInterfaceDummy::unsubscribeFromRobotState()
{
  joint_states_sub_.shutdown();
//  throw std::runtime_error("AdapterRosInterfaceDummy::unsubscribeFromRobotState() is not implemented.");

}
const std::string AdapterRosInterfaceDummy::getRobotStateMessageType()
{
  return ros::message_traits::datatype<moveit_msgs::RobotState>();
  throw std::runtime_error("AdapterRosInterfaceDummy::getRobotStateMessageType() is not implemented.");

}
bool AdapterRosInterfaceDummy::isReady() const
{
  throw std::runtime_error("AdapterRosInterfaceDummy::isReady() is not implemented.");

}


//! Update adapter.
bool AdapterRosInterfaceDummy::initializeAdapter(AdapterBase& adapter) const
{
  std::cout<<"Initial Adapter"<<std::endl;
//  throw std::runtime_error("AdapterRosInterfaceDummy::initializeAdapter() is not implemented.");

}
bool AdapterRosInterfaceDummy::updateAdapterWithRobotState(AdapterBase& adapter) const
{
    // TODO(Shunyao): How to  update state?
    return true;
  throw std::runtime_error("AdapterRosInterfaceDummy::updateAdapterWithRobotState() is not implemented.");

}

void AdapterRosInterfaceDummy::updateRobotState(const free_gait_msgs::RobotStateConstPtr& robotState)
{
  for(int i = 1;i<12;i++)
  {
    all_joint_positions_(i) = robotState->lf_leg_joints.position[i];
  }

}

}//namespace


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(free_gait::AdapterRosInterfaceDummy, free_gait::AdapterRosInterfaceBase)
