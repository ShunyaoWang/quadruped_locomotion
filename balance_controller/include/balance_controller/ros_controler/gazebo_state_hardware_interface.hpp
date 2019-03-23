/*
 *  gazebo_state_hardware_interface.hpp
 *  Descriotion: Gazebo Hardware Interface for simulation
 *
 *  Created on: Mar 19, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#pragma once
// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/robot_hw.h>
#include "hardware_interface/imu_sensor_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "balance_controller/ros_controler/robot_state_interface.hpp"
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo_msgs/ContactState.h>
#include <gazebo/sensors/sensors.hh>
//gazebo_ros_control
#include "gazebo_ros_control/robot_hw_sim.h"

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// URDF
#include <urdf/model.h>

//free_gait
#include "free_gait_msgs/RobotState.h"

namespace balance_controller {

class SimRobotStateHardwareInterface : public gazebo_ros_control::RobotHWSim//public hardware_interface::RobotHW, public hardware_interface::HardwareInterface
{
public:
//  SimRobotStateHardwareInterface(){};
  virtual bool initSim(
      const std::string& robot_namespace,
      ros::NodeHandle model_nh,
      gazebo::physics::ModelPtr parent_model,
      const urdf::Model *const urdf_model,
      std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void readSim(ros::Time time, ros::Duration period);

  virtual void writeSim(ros::Time time, ros::Duration period);

  virtual void eStopActive(const bool active);

  void readJoints();
  void writeJoints();
  void readStates();
protected:
  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID, STANCE_LEG};

  // Register the limits of the joint specified by joint_name and joint_handle. The limits are
  // retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
  // Return the joint's type, lower position limit, upper position limit, and effort limit.
  void registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle,
                           const ControlMethod ctrl_method,
                           const ros::NodeHandle& joint_limit_nh,
                           const urdf::Model *const urdf_model,
                           int *const joint_type, double *const lower_limit,
                           double *const upper_limit, double *const effort_limit);

  unsigned int n_dof_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;
  hardware_interface::ImuSensorInterface imu_interface_;
  hardware_interface::ImuSensorHandle::Data imu_data_;
  //!
  //! \brief robot_state_interface_, a robot state interface contain robot base state
  //! and joint states, see the robot_state_interface.hpp
  //!
  hardware_interface::RobotStateInterface robot_state_interface_;
  //!
  //! \brief robot_state_data_, robot_state data handle
  //!
  hardware_interface::RobotStateHandle::Data robot_state_data_;

  joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  std::vector<ControlMethod> joint_control_methods_;
  std::vector<control_toolbox::Pid> pid_controllers_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_position_command_;
  std::vector<double> last_joint_position_command_;
  std::vector<double> joint_velocity_command_;

  std::vector<gazebo::physics::JointPtr> sim_joints_;

//  gazebo::sensors::ContactSensorPtr contact_sensor_ptr;

//  gazebo::sensors::SensorPtr sensor_ptr;

  gazebo::physics::LinkPtr base_link_ptr_, lf_foot_link_ptr_, rf_foot_link_ptr_, rh_foot_link_ptr_, lh_foot_link_ptr_;

  std::string physics_type_;

  // e_stop_active_ is true if the emergency stop is active.
  bool e_stop_active_, last_e_stop_active_;

  double pos_read[12], pos_write[12], vel_read[12], vel_write[12], eff_read[12],eff_write[12];
  double position[3], orinetation[4], linear_vel[3], angular_vel[3];
  free_gait_msgs::RobotState actual_robot_state_;
};

typedef boost::shared_ptr<SimRobotStateHardwareInterface> SimRobotStateHardwareInterfacePtr;

};
