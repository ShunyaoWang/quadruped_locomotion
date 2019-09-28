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
//      adapterRosInterfaceLoader_("free_gait_ros", "free_gait::AdapterRosInterfaceGazebo")
{

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

  std::string adapterRosInterfacePluginName;
  nodeHandle.getParam(adapterRosInterfaceParameterName, adapterRosInterfacePluginName);
  //value="free_gait_ros/AdapterRosInterfaceGazebo"

  /**
    Load class "free_gait_ros/AdapterRosInterfaceGazebo"
    */
  adapterRosInterfaceLoader_.createInstance(adapterRosInterfacePluginName);
  adapterRosInterface_.reset(adapterRosInterfaceLoader_.createUnmanagedInstance(adapterRosInterfacePluginName));


  std::string adapterPluginName;
  //free_gait_ros/AdapterGazebo
  nodeHandle.getParam(adapterParameterName, adapterPluginName);
  std::cout <<"adapterPluginName is " << adapterPluginName << std::endl;
//  std::cout<<"===="<<adapterLoader_.getClassLibraryPath(adapterPluginName)<<std::endl;
  adapter_.reset(adapterLoader_.createUnmanagedInstance(adapterPluginName));//AdapterGazebo.cpp, use the function in adaptergazebo


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
      nodeHandle_.getParam("/free_gait/robot_state", topic);//quadruped_simulation.launch /gazebo/robot_state;
    } else {
      ROS_ERROR("Did not find ROS parameter for robot state topic '/free_gait/robot_state'.");
      return false;
    }
  }

  // Subscribe.
  return adapterRosInterface_->subscribeToRobotState(topic);//adapterRosInterface subscribe robot state.
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
    /**
    std::unique_ptr<AdapterRosInterfaceBase> adapterRosInterface_; base, here is gazebo;
    adapter here is adaptergazebo;
    finished the initialized before.
    adapter subcribes
*/
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
