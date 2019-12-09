#pragma once
#include "controller_interface/controller.h"
#include "balance_controller/motion_control/MotionControllerBase.hpp"
#include "balance_controller/ros_controler/gazebo_state_hardware_interface.hpp"
#include "kindr_ros/kindr_ros.hpp"
#include "state_switcher/StateSwitcher.hpp"
#include "controller_manager/controller_manager.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "pluginlib/class_list_macros.hpp"
#include "free_gait_msgs/RobotState.h"
#include "std_srvs/Empty.h"
//#include "free_gait_msgs/FootInBase.h"
#include "sim_assiants/FootContacts.h"

#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Time.h"
#include "nav_msgs/Odometry.h"
#include "unordered_map"
#include "ros/ros.h"
#include "ros/time.h"
#include "Eigen/Dense"
#include "balance_controller/ros_controler/robot_state_interface.hpp"
#include "curves/ScalarCurveConfig.hpp"
#include "curves/PolynomialSplineContainer.hpp"
#include "curves/polynomial_splines_containers.hpp"
#include "curves/PolynomialSplineScalarCurve.hpp"
#include "cmath"
#include <std_msgs/Bool.h>

namespace balance_controller{
class configure_change_controller: public controller_interface::Controller<hardware_interface::RobotStateInterface>
{
    typedef std::unordered_map<free_gait::LimbEnum, free_gait::Vector, EnumClassHash> LimbVector;
    typedef std::unordered_map<free_gait::LimbEnum, bool, EnumClassHash> LimbFlag;
    typedef std::unordered_map<free_gait::LimbEnum, double, EnumClassHash> Limbphase;
    typedef std::unordered_map<free_gait::LimbEnum, std::unique_ptr<StateSwitcher>, EnumClassHash> LimbState;
    typedef std::unordered_map<free_gait::LimbEnum, ros::Time, EnumClassHash> LimbDuration;

//    typedef polynomial
    typedef typename curves::Time Time;

public:
    configure_change_controller();
    ~configure_change_controller();
    bool init(hardware_interface::RobotStateInterface *hardware, ros::NodeHandle& nodehandle);
    void update(const ros::Time& time, const ros::Duration& period);
    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);

private:
    enum leg_configuration_{x_configuration,left_configuration}leg_configuration_;
    void baseCommandCallback(const free_gait_msgs::RobotStateConstPtr& robot_state_msgs);    
    void change_to_X_configure_CB(const std_msgs::BoolConstPtr& X_Configure);
    void change_to_left_configure_CB(const std_msgs::BoolConstPtr& left_Configure);
    void change_to_right_configure_CB(const std_msgs::BoolConstPtr& right_Configure);
    void change_to_anti_x_configure_CB(const std_msgs::BoolConstPtr& anti_x_Configure);
    std::vector<std::string> joint_names_;
    std::vector<hardware_interface::JointHandle> joints_;
    hardware_interface::RobotStateHandle robot_state_handle_;
    unsigned int n_joints_;
    ros::Subscriber base_command_sub_;
    std::shared_ptr<free_gait::State> robot_state_;
    Position base_desired_position_;

    RotationQuaternion base_desired_rotation_;
    LinearVelocity base_desired_linear_velocity_;
    LocalAngularVelocity base_desired_angular_velocity_;

    std::vector<free_gait::LimbEnum> limbs_;
    std::vector<free_gait::BranchEnum> branches_;
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

    ros::Publisher joint_command_pub_, base_command_pub_, base_actual_pub_, desired_robot_state_pub_, actual_robot_state_pub_;
    std::vector<sensor_msgs::JointState> joint_command_, joint_actual_;
    std::vector<free_gait_msgs::RobotState> desired_robot_state_, actual_robot_state_;
    std::vector<nav_msgs::Odometry> base_command_pose_, base_actual_pose_;
    //    std::vector<std_msgs::Int8MultiArray> leg_states_;
    std::vector<sim_assiants::FootContacts> foot_desired_contact_;
//    std::vector<free_gait_msgs::FootInBase> foot_in_base_;
    std::vector<std_msgs::Time> log_time_;
    std::vector<std_msgs::Float64MultiArray> leg_phases_;


    realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer;

    LimbVector foot_positions, foot_velocities, foot_accelerations;
    LimbFlag is_cartisian_motion_, is_footstep_, is_legmode_;
    LimbFlag sw_flag_, st_flag_;
    Limbphase sw_phase_, st_phase_;
    LimbState limbs_state_, limbs_desired_state_, limbs_last_state_;
    LimbDuration t_sw0_, t_st0_;
    bool ignore_contact_sensor;
    quadruped_model::QuadrupedKinematics robot_kinetics;
    free_gait::Stance footholdsInSupport;

    ros::ServiceServer log_data_srv_;
    ros::Publisher leg_state_pub_, leg_phase_pub_, desird_robot_state_pub_, contact_desired_pub_;
    ros::Subscriber x_configure_sub_,left_configure_sub_, right_configure_sub_, anti_x_configure_sub_;
    int log_length_;
    free_gait::JointPositions all_joint_positions_;

    curves::PolynomialSplineScalarCurve<curves::PolynomialSplineContainerQuintic> trajectory_lf_, trajectory_rf_,trajectory_rh_,trajectory_lh_;
    std::vector<curves::ScalarCurveConfig::ValueType> values_lf_, values_rf_, values_rh_, values_lh_;
    curves::ScalarCurveConfig::ValueType values_joint_1, values_joint_2, values_joint_3, values_joint_4;
//    curves::PolynomialSplineQuinticScalarCurve::ValueType
//        ScalarCurveConfig::ValueType
    std::vector<Time> times_;

    bool logDataCapture(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    Time dt_;
    bool configure_time_start_flag_;
    double initial_position_x_;
    double l1, l2;
    //    std::vector<bool> leg_states_;
};

}//namespace
