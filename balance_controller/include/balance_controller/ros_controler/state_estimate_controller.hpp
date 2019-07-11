/*
 *  state_estimate_controller.hpp
 *  Descriotion:
 *
 *  Created on: Jul, 7, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#pragma once

#include "controller_interface/controller.h"
#include "balance_controller/ros_controler/robot_state_interface.hpp"
#include <pluginlib/class_list_macros.hpp>

#include "free_gait_core/free_gait_core.hpp"
#include "sim_assiants/FootContacts.h"
#include "sensor_msgs/Imu.h"

#include "urdf/model.h"

namespace balance_controller {
  class StateEstimateController : public controller_interface::Controller<hardware_interface::RobotStateInterface>
  {
    typedef std::unordered_map<free_gait::LimbEnum, bool, EnumClassHash> LimbFlag;
  public:
    StateEstimateController();
    ~StateEstimateController();

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

    unsigned int n_joints;

  private:
    ros::Subscriber contact_sub_, imu_sub_;

    LimbFlag real_contact_;
    std::vector<free_gait::LimbEnum> limbs_;

    void footContactsCallback(const sim_assiants::FootContactsConstPtr& foot_contacts);
    void IMUmsgCallback(const sensor_msgs::ImuConstPtr& imu_msg);

  };

}
