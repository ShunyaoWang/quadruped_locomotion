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

#include "single_leg_test/model_test_header.hpp"

#include <control_toolbox/pid.h>

#include <pluginlib/class_list_macros.hpp>

#include "free_gait_msgs/RobotState.h"
#include "sim_assiants/FootContacts.h"

#include "std_srvs/Empty.h"

#include "kindr_ros/kindr_ros.hpp"

#include "state_switcher/StateSwitcher.hpp"

#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Wrench.h"
#include "std_msgs/Time.h"

#include "Eigen/Dense"
#include "Eigen/LU"

namespace balance_controller {

  class RosBalanceController : public controller_interface::Controller<hardware_interface::RobotStateInterface>
  {
  typedef std::unordered_map<free_gait::LimbEnum, std::unique_ptr<StateSwitcher>, EnumClassHash> LimbState;
  typedef std::unordered_map<free_gait::LimbEnum, ros::Time, EnumClassHash> LimbDuration;
  typedef std::unordered_map<free_gait::LimbEnum, bool, EnumClassHash> LimbFlag;
  typedef std::unordered_map<free_gait::LimbEnum, double, EnumClassHash> LimbPhase;
  typedef std::unordered_map<free_gait::LimbEnum, free_gait::Vector, EnumClassHash> LimbVector;

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
    realtime_tools::RealtimeBuffer<Pose> command_pose_buffer;
    realtime_tools::RealtimeBuffer<LimbVector> command_foot_buffer, command_foot_vel_buffer;
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
    std::shared_ptr<free_gait::State> robot_state;
    Position base_desired_position;
    RotationQuaternion base_desired_rotation;
    LinearVelocity base_desired_linear_velocity;
    LocalAngularVelocity base_desired_angular_velocity;

    std::vector<free_gait::LimbEnum> limbs_;
    std::vector<free_gait::BranchEnum> branches_;

    LimbState limbs_state, limbs_desired_state, limbs_last_state;
    LimbFlag real_contact_, is_cartisian_motion_, is_footstep_, is_legmode_;
    LimbDuration t_sw0, t_st0;
    LimbFlag sw_flag, st_flag;
    LimbPhase sw_phase, st_phase;
    LimbVector foot_positions, foot_velocities, foot_accelerations, real_contact_force_;
    /**
     * @brief contact_distribution_ , pointer to contact force optimaziton
     */
    std::shared_ptr<ContactForceDistribution> contact_distribution_;
    /**
     * @brief virtual_model_controller_, pointer to virtual model controller
     */

    std::shared_ptr<VirtualModelController> virtual_model_controller_;

    std::shared_ptr<MyRobotSolver> single_leg_solver_;

    std::vector<control_toolbox::Pid> pid_controllers_;       /**< Internal PID controllers. */

    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;
    /**
     * @brief baseCommandCallback, ros subscriber callback
     * @param robot_state
     */
    void baseCommandCallback(const free_gait_msgs::RobotStateConstPtr& robot_state_msg);
    void footContactsCallback(const sim_assiants::FootContactsConstPtr& foot_contacts);

    void enforceJointLimits(double &command, unsigned int index);
    double computeTorqueFromPositionCommand(double command, int i, const ros::Duration& period);

    void contactStateMachine();
    bool jointTorquesLimit(free_gait::JointEffortsLeg& joint_torque, double max_torque);
    /**
     * @brief r_mutex_
     */
    boost::recursive_mutex r_mutex_;
    /**
     * @brief joint_command_pub_, for debug to monitor
     */
    ros::Publisher joint_command_pub_, base_command_pub_, base_actual_pub_, joint_actual_pub_,
    leg_state_pub_, contact_desired_pub_, leg_phase_pub_, desired_robot_state_pub_, actual_robot_state_pub_, motor_status_word_pub_, vmc_info_pub_, desired_vmc_info_pub_;
    std::vector<nav_msgs::Odometry> base_command_pose_, base_actual_pose_;
    std::vector<sensor_msgs::JointState> joint_command_, joint_actual_;
    std::vector<std_msgs::Int8MultiArray> leg_states_;
    std::vector<sim_assiants::FootContacts> foot_desired_contact_;
    std::vector<std_msgs::Float64MultiArray> leg_phases_;
    std::vector<free_gait_msgs::RobotState> desired_robot_state_, actual_robot_state_;
    std::vector<geometry_msgs::Wrench> vitual_force_torque_, desired_vitual_force_torque_;
    std::vector<std_msgs::Time> log_time_;
    std::vector<std_msgs::Int8MultiArray> motor_status_word_;
    ros::ServiceServer log_data_srv_;

    int log_length_, log_index_;
    bool logDataCapture(std_srvs::Empty::Request& req,
                        std_srvs::Empty::Response& res);
    LimbFlag update_surface_normal_flag, store_current_joint_state_flag_;
//    bool store_current_joint_state_flag_;
    LimbVector surface_normals;
//    std::vector<double> stored_limb_joint_position_;
    free_gait::JointPositions stored_limb_joint_position_;

    int delay_counts[4];

    bool real_robot, ignore_contact_sensor;
  };

}
