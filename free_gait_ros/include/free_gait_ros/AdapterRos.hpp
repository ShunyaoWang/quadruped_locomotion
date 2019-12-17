/*
 * AdapterRos.hpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include <free_gait_core/executor/AdapterBase.hpp>
#include <free_gait_ros/AdapterRosInterfaceBase.hpp>
#include <ros/node_handle.h>
#include <pluginlib/class_loader.h>
#include "pluginlib_tutorials/polygon_base.h"

// STD
#include <memory>
#include <string>

namespace free_gait {

class AdapterRos
{
 public:
  enum class AdapterType {
    Base,
    Preview,
    Gazebo
  };

  AdapterRos(ros::NodeHandle& nodeHandle, const AdapterType type = AdapterType::Base);
//  AdapterRos(ros::NodeHandle& nodeHandle, const AdapterType type);
  virtual ~AdapterRos();
  bool subscribeToRobotState(const std::string& robotStateTopic = "");//moren parameters
  void unsubscribeFromRobotState();
  const std::string getRobotStateMessageType();
  bool isReady() const ;
  bool updateAdapterWithState();
  const AdapterBase& getAdapter() const;
  AdapterBase* getAdapterPtr();

 private:
  ros::NodeHandle& nodeHandle_;
  pluginlib::ClassLoader<polygon_base::RegularPolygon> tutor_loader_;
  std::unique_ptr<polygon_base::RegularPolygon> tutor_;
  pluginlib::ClassLoader<AdapterBase> adapterLoader_;
  std::unique_ptr<AdapterBase> adapter_;
  pluginlib::ClassLoader<AdapterRosInterfaceBase> adapterRosInterfaceLoader_;
  std::unique_ptr<AdapterRosInterfaceBase> adapterRosInterface_;
};

} /* namespace free_gait */
