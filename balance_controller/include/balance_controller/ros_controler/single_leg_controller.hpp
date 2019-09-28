/*
 *  single_leg_controller.cpp
 *  Descriotion:
 *
 *  Created on: Jul, 7, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#pragma once

#include "controller_interface/controller.h"
#include "balance_controller/ros_controler/robot_state_interface.hpp"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"

#include "single_leg_test/model_test_header.hpp"
#include <pluginlib/class_list_macros.hpp>
#include "realtime_tools/realtime_buffer.h"
#include "kindr_ros/kindr_ros.hpp"

#include "free_gait_msgs/RobotState.h"
#include "sim_assiants/FootContacts.h"
#include "urdf/model.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "free_gait_msgs/SetLimbConfigure.h"

#include "geometry_msgs/Wrench.h"

#include "algorithm"
#include "new_quadruped_model_kp/joint_state.h"


namespace balance_controller {

  class SingleLegController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
  {
    typedef std::unordered_map<free_gait::LimbEnum, bool, EnumClassHash> LimbFlag;
    typedef std::unordered_map<free_gait::LimbEnum, free_gait::Vector, EnumClassHash> LimbVector;
  public:
    SingleLegController();
    ~SingleLegController();

    /**
     * @brief init
     * @param hardware
     * @param node_handle
     * @return
     */
    bool init(hardware_interface::EffortJointInterface* hardware,
              ros::NodeHandle& node_handle);
    void update(const ros::Time& time, const ros::Duration& period);
    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);
    /**
     * @brief joint_names
     */
    std::vector<std::string> joint_names;
    std::vector<std::string> leg_names;
    std::vector<std::string> control_methods;
    std::vector<free_gait::LimbEnum> leg_to_move;
    bool real_time_;
    /**
     * @brief joints, a vector of joint handle, handle the joint of hardware
     * interface
     */
    std::vector<hardware_interface::JointHandle> joints;//_positions;
//    std::vector<hardware_interface::JointHandle> joint_efforts;
    realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer;
    unsigned int n_joints;

    void staticTFPublisher();
    void robotStatePublisher();
    void fakePosePublisher();

  private:

    /**
     * @brief base_command_sub_,subscribe base_command and contact information
     */
    ros::Subscriber base_command_sub_, contact_sub_, contact_force_sub_;

    ros::ServiceClient switchControlMethodClient_;
    //! TF boardcaster
    tf::TransformBroadcaster tfBoardcaster_;
    tf::Transform odom2base, odom_to_footprint, footprint_to_base;
    tf::Quaternion q;

    //! ROS publisher
    ros::Publisher fakePosePub_, robot_state_pub_, joint_state_pub_;

    //! pose
    geometry_msgs::PoseWithCovarianceStamped fakePoseMsg_;
    geometry_msgs::Pose base_pose;

//    std::vector<Force> contactForces_;

    free_gait_msgs::RobotState robot_state_msg;
    /**
     * @brief robot_state_, State class to save all the robot state ,and provide method of
     * kinemaics
     */
    std::shared_ptr<free_gait::State> robot_state_;

    std::shared_ptr<MyRobotSolver> single_leg_solver_;

    std::vector<free_gait::LimbEnum> limbs_;
    std::vector<free_gait::BranchEnum> branches_;
    LimbFlag real_contact_, is_cartisian_motion_;
    LimbVector foot_positions, foot_velocities, foot_accelerations, contactForces_;

    Position end_desired_position;

    //! WSHY: update joint state
    free_gait::JointPositions all_joint_positions;
    free_gait::JointVelocities all_joint_velocities;
    free_gait::JointEfforts all_joint_efforts;

    /**
     * @brief baseCommandCallback, ros subscriber callback
     * @param robot_state
     */
    void baseCommandCallback(const free_gait_msgs::RobotStateConstPtr& robot_state_msg);
    void footContactsCallback(const sim_assiants::FootContactsConstPtr& foot_contacts);

    void forceCommandCallback(const geometry_msgs::WrenchConstPtr& force_command);
  };
}
