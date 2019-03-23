/*
 * AdapterRos.cpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "free_gait_ros/AdapterRos.hpp"
namespace free_gait {

AdapterRos::AdapterRos(ros::NodeHandle& nodeHandle, const AdapterType type)
    : nodeHandle_(nodeHandle),
      tutor_loader_("pluginlib_tutorials", "polygon_base::RegularPolygon"),
      adapterLoader_("free_gait_ros", "free_gait::AdapterBase"),
      adapterRosInterfaceLoader_("free_gait_ros", "free_gait::AdapterRosInterfaceBase")
{
  // Load and initialize adapter.
  std::cout<<"Constructing AdapterRos..."<<std::endl;
  std::cout<<adapterLoader_.getBaseClassType()<<std::endl;
  std::cout<<adapterLoader_.getDeclaredClasses()[0]<<std::endl;
//  std::cout<<adapterLoader_.g<<std::endl;
  std::string adapterParameterName;
  std::string adapterRosInterfaceParameterName;
  if (type == AdapterType::Base) {
    adapterParameterName = "/free_gait/adapter_plugin/base";
    adapterRosInterfaceParameterName = "/free_gait/adapter_ros_interface_plugin/base";
  } else if (type == AdapterType::Preview){
    adapterParameterName = "/free_gait/adapter_plugin/preview";
    adapterRosInterfaceParameterName = "/free_gait/adapter_ros_interface_plugin/preview";
  } else if (type == AdapterType::Gazebo) {
    adapterParameterName = "/free_gait/adapter_plugin/gazebo";
    adapterRosInterfaceParameterName = "/free_gait/adapter_ros_interface_plugin/gazebo";
  };

//  pluginlib::ClassLoader<polygon_base::RegularPolygon> tutor_loader("pluginlib_tutorials", "polygon_base::RegularPolygon");
//  tutor_loader.createInstance("pluginlib_tutorials/regular_triangle");
//  tutor_.reset(tutor_loader_.createUnmanagedInstance("pluginlib_tutorials/regular_triangle"));
//  std::cout<<"Haved loaded tutorials"<<std::endl;
//  std::vector<std::string> libs_before = adapterRosInterfaceLoader_.getRegisteredLibraries();
  std::string adapterRosInterfacePluginName;
  nodeHandle.getParam(adapterRosInterfaceParameterName, adapterRosInterfacePluginName);
//  std::cout<<"===="<<adapterRosInterfaceLoader_.getClassLibraryPath(adapterRosInterfacePluginName)<<std::endl;
  adapterRosInterfaceLoader_.createInstance(adapterRosInterfacePluginName);
  adapterRosInterface_.reset(adapterRosInterfaceLoader_.createUnmanagedInstance(adapterRosInterfacePluginName));
//  std::vector<std::string> libs_after =adapterRosInterfaceLoader_.getRegisteredLibraries();
//  std::cout<<libs_after[0]<<std::endl;

  std::string adapterPluginName;
  nodeHandle.getParam(adapterParameterName, adapterPluginName);
//  std::cout<<"===="<<adapterLoader_.getClassLibraryPath(adapterPluginName)<<std::endl;
  adapter_.reset(adapterLoader_.createUnmanagedInstance(adapterPluginName));

//  std::string adapterRosInterfacePluginName;
//  nodeHandle.getParam("/free_gait/adapter_ros_interface_plugin", adapterRosInterfacePluginName);
//  adapterRosInterface_.reset(adapterRosInterfaceLoader_.createUnmanagedInstance(adapterRosInterfacePluginName));

  adapterRosInterface_->setNodeHandle(nodeHandle_);
  adapterRosInterface_->readRobotDescription();
  adapterRosInterface_->initializeAdapter(*adapter_);
}

AdapterRos::~AdapterRos()
{
}

bool AdapterRos::subscribeToRobotState(const std::string& robotStateTopic)
{
  // Get topic name parameter.
  std::string topic(robotStateTopic);
  if (topic.empty()) {
    if (nodeHandle_.hasParam("/free_gait/robot_state")) {
      nodeHandle_.getParam("/free_gait/robot_state", topic);
    } else {
      ROS_ERROR("Did not find ROS parameter for robot state topic '/free_gait/robot_state'.");
      return false;
    }
  }

  // Subscribe.
  return adapterRosInterface_->subscribeToRobotState(topic);
}

void AdapterRos::unsubscribeFromRobotState()
{
  adapterRosInterface_->unsubscribeFromRobotState();
}

const std::string AdapterRos::getRobotStateMessageType()
{
  return adapterRosInterface_->getRobotStateMessageType();
}

bool AdapterRos::isReady() const
{
    return adapterRosInterface_->isReady();
}

bool AdapterRos::updateAdapterWithState()
{
  return adapterRosInterface_->updateAdapterWithRobotState(*adapter_);
}

const AdapterBase& AdapterRos::getAdapter() const
{
  return *adapter_;
}

AdapterBase* AdapterRos::getAdapterPtr()
{
  return adapter_.get();
}

} /* namespace free_gait */
