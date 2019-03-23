/*
 *  filename.hpp
 *  Descriotion:
 *
 *  Created on: Dec 26, 2018
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#pragma once
#include "moveit_msgs/RobotState.h"
#include "free_gait_ros/free_gait_ros.hpp"

namespace free_gait {

class AdapterRosInterfaceDummy : public AdapterRosInterfaceBase
{
public:
  AdapterRosInterfaceDummy();
  ~AdapterRosInterfaceDummy();

  virtual bool subscribeToRobotState(const std::string& robotStateTopic);
  virtual void unsubscribeFromRobotState();
  virtual const std::string getRobotStateMessageType();
  virtual bool isReady() const;

  //! Update adapter.
  virtual bool initializeAdapter(AdapterBase& adapter) const;
  virtual bool updateAdapterWithRobotState(AdapterBase& adapter) const;

  void updateRobotState(const free_gait_msgs::RobotStateConstPtr& robotState);


private:
  ros::Subscriber joint_states_sub_;
  JointPositions all_joint_positions_;

};

} // namespace
