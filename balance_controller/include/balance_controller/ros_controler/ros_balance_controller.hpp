/*
 *  ros_balance_controller.hpp
 *  Descriotion: ros_controller use VMC method to control base pose
 * TODO: access of Gait Pattern
 *
 *  Created on: Mar 18, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#pragma once

#include "controller_interface/controller.h"
#include "hardware_interface/joint_command_interface.h"
#include "balance_controller/contact_force_distribution/ContactForceDistribution.hpp"
#include "balance_controller/contact_force_distribution/ContactForceDistributionBase.hpp"
#include "balance_controller/motion_control/MotionControllerBase.hpp"
#include "balance_controller/motion_control/VirtualModelController.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "balance_controller/ros_controler/robot_state_interface.hpp"
#include "balance_controller/ros_controler/gazebo_state_hardware_interface.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "free_gait_msgs/RobotState.h"
#include "sim_assiants/FootContacts.h"

namespace balance_controller {
  class RosBalanceController : public controller_interface::Controller<hardware_interface::RobotStateInterface>
  {
  public:
    RosBalanceController();
    ~RosBalanceController();
    /**
     * @brief init
     * @param hardware
     * @param node_handle
     * @return
     */
    bool init(hardware_interface::RobotStateInterface* hardware,
              ros::NodeHandle& node_handle);
    void update(const ros::Time& time, const ros::Duration& period);
    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);
    /**
     * @brief joint_names
     */
    std::vector<std::string> joint_names;
    /**
     * @brief joints, a vector of joint handle, handle the joint of hardware
     * interface
     */
    std::vector<hardware_interface::JointHandle> joints;
//    std::vector<hardware_interface::RobotStateHandle> joints;
    /**
     * @brief robot_state_handle, handle robot state
     */
    hardware_interface::RobotStateHandle robot_state_handle;
    /**
     * @brief commands_buffer,TODO
     */
    realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer;
    unsigned int n_joints;
  private:
    /**
     * @brief base_command_sub_,subscribe base_command and contact information
     */
    ros::Subscriber base_command_sub_, contact_sub_;
    /**
     * @brief robot_state_, State class to save all the robot state ,and provide method of
     * kinemaics
     */
    std::shared_ptr<free_gait::State> robot_state_;

    std::vector<free_gait::LimbEnum> limbs_;
    std::vector<free_gait::BranchEnum> branches_;

    /**
     * @brief contact_distribution_ , pointer to contact force optimaziton
     */
    std::shared_ptr<ContactForceDistribution> contact_distribution_;
    /**
     * @brief virtual_model_controller_, pointer to virtual model controller
     */

    std::shared_ptr<VirtualModelController> virtual_model_controller_;
    /**
     * @brief baseCommandCallback, ros subscriber callback
     * @param robot_state
     */
    void baseCommandCallback(const free_gait_msgs::RobotStateConstPtr& robot_state);
    void footContactsCallback(const sim_assiants::FootContactsConstPtr& foot_contacts);
  };

}
