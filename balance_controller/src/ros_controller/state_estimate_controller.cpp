/*
 *  state_estimate_controller.cpp
 *  Descriotion:
 *
 *  Created on: Jul, 7, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#include "balance_controller/ros_controler/state_estimate_controller.hpp"

namespace balance_controller {

  StateEstimateController::StateEstimateController()
  {
    limbs_.push_back(free_gait::LimbEnum::LF_LEG);
    limbs_.push_back(free_gait::LimbEnum::RF_LEG);
    limbs_.push_back(free_gait::LimbEnum::LH_LEG);
    limbs_.push_back(free_gait::LimbEnum::RH_LEG);
    for(auto limb : limbs_)
      {
        real_contact_[limb] = false;
      }
  };

  StateEstimateController::~StateEstimateController()
  {};

  bool StateEstimateController::init(hardware_interface::RobotStateInterface* hardware,
                                     ros::NodeHandle& node_handle)
  {
    ROS_INFO("Init StateEstimateController");

//    urdf::Model urdf;
//    if (!urdf.initParam("/robot_description"))
//    {
//      ROS_ERROR("Failed to parse urdf file");
//      return false;
//    }

//    //! WSHY: get joint handle from robot state handle
//    std::string param_name = "joints";
//    if(!node_handle.getParam(param_name, joint_names))
//      {
//        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << node_handle.getNamespace() << ").");
//        return false;
//      }
//    n_joints = joint_names.size();
//    if(n_joints == 0){
//          ROS_ERROR_STREAM("List of joint names is empty.");
//          return false;
//        }

    //! WSHY: get robot state handle
    robot_state_handle = hardware->getHandle("base_controller");

    contact_sub_ = node_handle.subscribe<sim_assiants::FootContacts>("/bumper_sensor_filter_node/foot_contacts", 1, &StateEstimateController::footContactsCallback, this);

    imu_sub_ = node_handle.subscribe<sensor_msgs::Imu>("base_imu", 1, &StateEstimateController::IMUmsgCallback, this);
  }

  void StateEstimateController::update(const ros::Time& time, const ros::Duration& period)
  {
    ROS_INFO("State Estimate Update Once");
    //! WSHY: update joint state
    free_gait::JointPositions all_joint_positions;
    free_gait::JointVelocities all_joint_velocities;
    free_gait::JointEfforts all_joint_efforts;
    //! WSHY: get joint postions from robot state handle
    for(unsigned int i=0; i<12; i++)
      {
        all_joint_positions(i) = robot_state_handle.getJointPositionRead()[i];
        all_joint_velocities(i) = robot_state_handle.getJointVelocityRead()[i];
        all_joint_efforts(i) = robot_state_handle.getJointEffortRead()[i];
        ROS_INFO("Joint %d Position is : %f", i, all_joint_positions(i));
      }
    // TODO: call a state estimate method to calculate the pose estimate of robot.
    robot_state_handle.position_[0] = 0;
    robot_state_handle.position_[1] = 1;
    robot_state_handle.position_[2] = 2;
  }

  void StateEstimateController::starting(const ros::Time& time)
  {}

  void StateEstimateController::stopping(const ros::Time& time)
  {

  }

  void StateEstimateController::footContactsCallback(const sim_assiants::FootContactsConstPtr& foot_contacts)
  {
    unsigned int i = 0;
    for(auto contact : foot_contacts->foot_contacts)
      {
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
        real_contact_.at(limb) = contact.is_contact;
        i++;
      }
  }

  void StateEstimateController::IMUmsgCallback(const sensor_msgs::ImuConstPtr& imu_msg)
  {

  }

}
PLUGINLIB_EXPORT_CLASS(balance_controller::StateEstimateController, controller_interface::ControllerBase)
