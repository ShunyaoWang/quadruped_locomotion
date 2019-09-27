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
//#include "free_gait_msgs/RobotState.h"
#include "sim_assiants/FootContacts.h"
#include "sensor_msgs/Imu.h"

#include "urdf/model.h"
#include "legodom.h"
#include "kindr_ros/kindr_ros.hpp"
#include "state_switcher/StateSwitcher.hpp"
//#include "quadruped_odom/legodom.h"

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
//    hardware_interface::RobotStateHandle::Data robot_state_data_;
    unsigned int n_joints;

    Eigen::VectorXd OdomState;
    double real_time_factor;

  private:
    ros::Subscriber contact_sub_, imu_sub_;
    ros::Publisher robot_state_pub_;
    std::shared_ptr<free_gait::State> robot_state_ptr;
    std::shared_ptr<quadruped_odom::QuadrupedEstimation> LegOdom;

    LimbFlag real_contact_;
    std::vector<free_gait::LimbEnum> limbs_;
    std::vector<free_gait::BranchEnum> branches_;

    free_gait_msgs::RobotState robot_state_;
    void footContactsCallback(const sim_assiants::FootContactsConstPtr& foot_contacts);
    int delay_counts[4];
    bool use_gazebo_feedback, real_robot;
//    void IMUmsgCallback(const sensor_msgs::ImuConstPtr& imu_msg);

  };

}
