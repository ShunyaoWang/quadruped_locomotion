/*
 *  filename.hpp
 *  Descriotion:
 *
 *  Created on: Dec 26, 2018
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#pragma once
#include "free_gait_msgs/RobotState.h"
#include "free_gait_ros/free_gait_ros.hpp"
#include "free_gait_msgs/LegMode.h"

namespace free_gait {

class AdapterRosInterfaceGazebo : public AdapterRosInterfaceBase
{
public:
  AdapterRosInterfaceGazebo();
  ~AdapterRosInterfaceGazebo();

  virtual bool subscribeToRobotState(const std::string& robotStateTopic);
  virtual void unsubscribeFromRobotState();
  virtual const std::string getRobotStateMessageType();
  virtual bool isReady() const;

  //! Update adapter.
  virtual bool initializeAdapter(AdapterBase& adapter) const;
  virtual bool updateAdapterWithRobotState(AdapterBase& adapter) const;

  void updateRobotState(const free_gait_msgs::RobotStateConstPtr& robotState);
//  bool isInitialized();

private:
  ros::Subscriber joint_states_sub_;
  JointPositions all_joint_positions_;
  nav_msgs::Odometry base_pose_in_world_;
  Stance foot_in_support_;
  std::vector<free_gait_msgs::LegMode> leg_modes_;
  bool is_initialized_;

};

} // namespace
