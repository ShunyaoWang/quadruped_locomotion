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
  limbs_.push_back(free_gait::LimbEnum::RH_LEG);
  limbs_.push_back(free_gait::LimbEnum::LH_LEG);
  branches_.push_back(free_gait::BranchEnum::BASE);
  branches_.push_back(free_gait::BranchEnum::LF_LEG);
  branches_.push_back(free_gait::BranchEnum::RF_LEG);
  branches_.push_back(free_gait::BranchEnum::LH_LEG);
  branches_.push_back(free_gait::BranchEnum::RH_LEG);

  robot_state_ptr.reset(new free_gait::State);
  robot_state_ptr->initialize(limbs_, branches_);

  for(auto limb : limbs_)
    {
      real_contact_[limb] = true;
    }

};

StateEstimateController::~StateEstimateController()
{};

bool StateEstimateController::init(hardware_interface::RobotStateInterface* hardware,
                                     ros::NodeHandle& node_handle)
{
    ROS_INFO("Init StateEstimateController");
    // quadruped_odom::QuadrupedEstimation legodom(node_handle, robot_state_);
    //reset???
    LegOdom.reset(new quadruped_odom::QuadrupedEstimation(node_handle,robot_state_ptr));

//    urdf::Model urdf;
//    if (!urdf.initParam("/robot_description"))
//    {
//      ROS_ERROR("Failed to parse urdf file");
//      return false;
//    }

    if(!node_handle.getParam("/real_time_factor", real_time_factor))
      {
        ROS_ERROR("Can't find parameter of 'real_time_factor'");
        return false;
      }
    if(!node_handle.getParam("use_gazebo_feedback", use_gazebo_feedback))
      {
        ROS_ERROR("Can't find parameter of 'use_gazebo_feedback'");
        return false;
      }
    if(!node_handle.getParam("real_robot", real_robot))
      {
        ROS_ERROR("Can't find parameter of 'real_robot'");
        return false;
      }

    //! WSHY: get joint handle from robot state handle
    std::string param_name = "joints";
    if(!node_handle.getParam(param_name, joint_names))
      {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << node_handle.getNamespace() << ").");
        return false;
      }
    n_joints = joint_names.size();
    if(n_joints == 0){
          ROS_ERROR_STREAM("List of joint names is empty.");
          return false;
    }

    //! WSHY: get robot state handle
    robot_state_handle = hardware->getHandle("base_controller");
    for(int i = 0;i<4;i++)
      robot_state_handle.foot_contact_[i] = 1;

    //    imu_sub_ = node_handle.subscribe<sensor_msgs::Imu>("/imu", 1, &StateEstimateController::IMUmsgCallback, this);
//    contact_sub_ = node_handle.subscribe<sim_assiants::FootContacts>("/bumper_sensor_filter_node/foot_contacts", 1, &StateEstimateController::footContactsCallback, this);
    robot_state_pub_ = node_handle.advertise<free_gait_msgs::RobotState>("/legodom/robot_states", 1);

    robot_state_.lf_leg_joints.name.resize(3);
    robot_state_.lf_leg_joints.position.resize(3);
    robot_state_.lf_leg_joints.velocity.resize(3);
    robot_state_.lf_leg_joints.effort.resize(3);

    robot_state_.rf_leg_joints.name.resize(3);
    robot_state_.rf_leg_joints.position.resize(3);
    robot_state_.rf_leg_joints.velocity.resize(3);
    robot_state_.rf_leg_joints.effort.resize(3);

    robot_state_.lh_leg_joints.name.resize(3);
    robot_state_.lh_leg_joints.position.resize(3);
    robot_state_.lh_leg_joints.velocity.resize(3);
    robot_state_.lh_leg_joints.effort.resize(3);

    robot_state_.rh_leg_joints.name.resize(3);
    robot_state_.rh_leg_joints.position.resize(3);
    robot_state_.rh_leg_joints.velocity.resize(3);
    robot_state_.rh_leg_joints.effort.resize(3);

    geometry_msgs::Vector3Stamped vector_z;
    vector_z.vector.x = 0;
    vector_z.vector.y = 0;
    vector_z.vector.z = 1;
    robot_state_.lf_leg_mode.name = "LF_LEG";
    robot_state_.lf_leg_mode.surface_normal = vector_z;

//    robot_state_.rf_leg_mode.support_leg = foot_contacts->foot_contacts[1].is_contact;
    robot_state_.rf_leg_mode.name = "RF_LEG";
    robot_state_.rf_leg_mode.surface_normal = vector_z;

//    robot_state_.rh_leg_mode.support_leg = foot_contacts->foot_contacts[2].is_contact;
    robot_state_.rh_leg_mode.name = "RH_LEG";
    robot_state_.rh_leg_mode.surface_normal = vector_z;

//    robot_state_.lh_leg_mode.support_leg = foot_contacts->foot_contacts[3].is_contact;
    robot_state_.lh_leg_mode.name = "LH_LEG";
    robot_state_.lh_leg_mode.surface_normal = vector_z;


  }

void StateEstimateController::update(const ros::Time& time, const ros::Duration& period)
{
//    ROS_INFO("State Estimate Update Once");
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
//        ROS_INFO("Joint %d Position is : %f", i, all_joint_positions(i));
      }

// TODO: call a state estimate method to calculate the pose estimate of robot.
    for(int i=0;i<12;++i){
        LegOdom->joints_output.position[i] = all_joint_positions(i);
        LegOdom->joints_output.velocity[i] = all_joint_velocities(i);
    }

    robot_state_ptr->setCurrentLimbJoints(all_joint_positions);
    robot_state_ptr->setCurrentLimbJointVelocities(all_joint_velocities);
//    std::vector<bool> real_c
    std_msgs::Float64MultiArray foot_msg;
    foot_msg.data.resize(4);
    for(int i = 0;i<4;i++)
      {
        StateSwitcher::States foot_state = static_cast<StateSwitcher::States>(robot_state_handle.foot_contact_[i]);
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
//        if(foot_state == StateSwitcher::States::SwingEarlyTouchDown)
//          {
//            //delay 10 periods for make sure contact
//            delay_counts[i] ++;
//            if(delay_counts[i] == 10)
//              {
//                delay_counts[i] = 0;
//                real_contact_.at(limb) = true;
//              }
//          }
//        if(foot_state == StateSwitcher::States::SwingLatelyTouchDown)
//          {
//            //delay 10 periods for make sure contact
//            delay_counts[i] ++;
//            if(delay_counts[i] == 10)
//              {
//                delay_counts[i] = 0;
//                real_contact_.at(limb) = true;
//              }
//          }
        if(foot_state == StateSwitcher::States::StanceNormal)
          {
            real_contact_.at(limb) = true;
            foot_msg.data[i] = 1;
          }
        if(foot_state == StateSwitcher::States::SwingNormal)
          {
            real_contact_.at(limb) = false;
            foot_msg.data[i] = 0;
          }
//        std::cout<<"foot "<<i<<" contact state "<<robot_state_handle.foot_contact_[i]<<std::endl;

      }
      LegOdom->setFootState(foot_msg);
      robot_state_.lf_leg_mode.support_leg = real_contact_.at(free_gait::LimbEnum::LF_LEG);
      robot_state_.rf_leg_mode.support_leg = real_contact_.at(free_gait::LimbEnum::RF_LEG);
      robot_state_.rh_leg_mode.support_leg = real_contact_.at(free_gait::LimbEnum::RH_LEG);
      robot_state_.lh_leg_mode.support_leg = real_contact_.at(free_gait::LimbEnum::LH_LEG);

    //!!!频率控制，保证每秒钟处理的image不多于FREQ，这里将平率控制在10hz以内
//    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
//    {
//        PUB_THIS_FRAME = true;
//        // reset the frequency control
//        //重置
//        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
//        {
//            first_image_time = img_msg->header.stamp.toSec();
//            pub_count = 0;
//        }
//    }
//    else
//        PUB_THIS_FRAME = false;

    if(LegOdom->ProcessSensorData()){
//        ROS_WARN("Get updata state estimation...");
    }else{
//        ROS_ERROR("Process Error...");
    }

    OdomState = LegOdom->GetStopStateOdom();//PVQ
//    std::cout <<"OdomState: " << OdomState << std::endl;


    if(!use_gazebo_feedback || real_robot)
      {
        robot_state_handle.position_[0] = LegOdom->odom_position.x();
        robot_state_handle.position_[1] = LegOdom->odom_position.y();
        robot_state_handle.position_[2] = LegOdom->odom_position.z();
        robot_state_handle.linear_velocity_[0] = LegOdom->odom_vel.x();
        robot_state_handle.linear_velocity_[1] = LegOdom->odom_vel.y();
        robot_state_handle.linear_velocity_[2] = LegOdom->odom_vel.z();
        robot_state_handle.orientation_[0] = LegOdom->odom_orientation.w();//w,x,y,z
        robot_state_handle.orientation_[1] = LegOdom->odom_orientation.x();
        robot_state_handle.orientation_[2] = LegOdom->odom_orientation.y();
        robot_state_handle.orientation_[3] = LegOdom->odom_orientation.z();
      }

    if(real_robot)
      {
        robot_state_handle.angular_velocity_[0] = real_time_factor*LegOdom->imu_output.angular_velocity.x;
        robot_state_handle.angular_velocity_[1] = real_time_factor*LegOdom->imu_output.angular_velocity.y;
        robot_state_handle.angular_velocity_[2] = real_time_factor*LegOdom->imu_output.angular_velocity.z;
      }


//    robot_state_.lf_leg_joints.header =
    robot_state_.lf_leg_joints.name[0] = "front_left_1_joint";
    robot_state_.lf_leg_joints.position[0] = all_joint_positions(0);
    robot_state_.lf_leg_joints.velocity[0] = all_joint_velocities(0);
    robot_state_.lf_leg_joints.effort[0] = all_joint_efforts(0);
    robot_state_.lf_leg_joints.name[1] = "front_left_2_joint";
    robot_state_.lf_leg_joints.position[1] = all_joint_positions(1);
    robot_state_.lf_leg_joints.velocity[1] = all_joint_velocities(1);
    robot_state_.lf_leg_joints.effort[1] = all_joint_efforts(1);
    robot_state_.lf_leg_joints.name[2] = "front_left_3_joint";
    robot_state_.lf_leg_joints.position[2] = all_joint_positions(2);
    robot_state_.lf_leg_joints.velocity[2] = all_joint_velocities(2);
    robot_state_.lf_leg_joints.effort[2] = all_joint_efforts(2);

//    robot_state_.rf_leg_joints.header = joint_states->header;
    robot_state_.rf_leg_joints.name[0] = "front_right_1_joint";
    robot_state_.rf_leg_joints.position[0] = all_joint_positions(3);
    robot_state_.rf_leg_joints.velocity[0] = all_joint_velocities(3);
    robot_state_.rf_leg_joints.effort[0] = all_joint_efforts(3);
    robot_state_.rf_leg_joints.name[1] = "front_right_2_joint";
    robot_state_.rf_leg_joints.position[1] = all_joint_positions(4);
    robot_state_.rf_leg_joints.velocity[1] = all_joint_velocities(4);
    robot_state_.rf_leg_joints.effort[1] = all_joint_efforts(4);
    robot_state_.rf_leg_joints.name[2] = "front_right_3_joint";
    robot_state_.rf_leg_joints.position[2] = all_joint_positions(5);
    robot_state_.rf_leg_joints.velocity[2] = all_joint_velocities(5);
    robot_state_.rf_leg_joints.effort[2] = all_joint_efforts(5);

    //    robot_state_.rh_leg_joints.header = joint_states->header;
    robot_state_.rh_leg_joints.name[0] = "rear_right_1_joint";
    robot_state_.rh_leg_joints.position[0] = all_joint_positions(6);
    robot_state_.rh_leg_joints.velocity[0] = all_joint_velocities(6);
    robot_state_.rh_leg_joints.effort[0] = all_joint_efforts(6);
    robot_state_.rh_leg_joints.name[1] = "rear_right_2_joint";
    robot_state_.rh_leg_joints.position[1] = all_joint_positions(7);
    robot_state_.rh_leg_joints.velocity[1] = all_joint_velocities(7);
    robot_state_.rh_leg_joints.effort[1] = all_joint_efforts(7);
    robot_state_.rh_leg_joints.name[2] = "rear_right_3_joint";
    robot_state_.rh_leg_joints.position[2] = all_joint_positions(8);
    robot_state_.rh_leg_joints.velocity[2] = all_joint_velocities(8);
    robot_state_.rh_leg_joints.effort[2] = all_joint_efforts(8);

//    robot_state_.lh_leg_joints.header = joint_states->header;
    robot_state_.lh_leg_joints.name[0] = "rear_left_1_joint";
    robot_state_.lh_leg_joints.position[0] = all_joint_positions(9);
    robot_state_.lh_leg_joints.velocity[0] = all_joint_velocities(9);
    robot_state_.lh_leg_joints.effort[0] = all_joint_efforts(9);
    robot_state_.lh_leg_joints.name[1] = "rear_left_2_joint";
    robot_state_.lh_leg_joints.position[1] = all_joint_positions(10);
    robot_state_.lh_leg_joints.velocity[1] = all_joint_velocities(10);
    robot_state_.lh_leg_joints.effort[1] = all_joint_efforts(10);
    robot_state_.lh_leg_joints.name[2] = "rear_left_3_joint";
    robot_state_.lh_leg_joints.position[2] = all_joint_positions(11);
    robot_state_.lh_leg_joints.velocity[2] = all_joint_velocities(11);
    robot_state_.lh_leg_joints.effort[2] = all_joint_efforts(11);



    kindr_ros::convertToRosGeometryMsg(LegOdom->odom_position, robot_state_.base_pose.pose.pose.position);
    kindr_ros::convertToRosGeometryMsg(LegOdom->odom_orientation, robot_state_.base_pose.pose.pose.orientation);
    kindr_ros::convertToRosGeometryMsg(LegOdom->odom_vel, robot_state_.base_pose.twist.twist.linear);
    robot_state_.base_pose.twist.twist.angular = LegOdom->imu_output.angular_velocity;
    robot_state_.base_pose.child_frame_id = "/base_link";
    robot_state_.base_pose.header.frame_id = "/odom";

    robot_state_pub_.publish(robot_state_);
//    ROS_ERROR("robot_state_..........");

}

  void StateEstimateController::starting(const ros::Time& time)
  {
    ROS_INFO("State Estimate Start Once");
    for(int i = 0;i<4;i++)
      robot_state_handle.foot_contact_[i] = 1;

  }

  void StateEstimateController::stopping(const ros::Time& time)
  {
    ROS_INFO("State Estimate Stop Once");
    LegOdom->ResetParms();

  }

  void StateEstimateController::footContactsCallback(const sim_assiants::FootContactsConstPtr& foot_contacts)
  {
//    unsigned int i = 0;
//    for(auto contact : foot_contacts->foot_contacts)
//      {
//        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
//        real_contact_.at(limb) = contact.is_contact;
//        i++;
//      }


//    robot_state_.lf_leg_mode.support_leg = foot_contacts->foot_contacts[0].is_contact;
    robot_state_.lf_leg_mode.name = foot_contacts->foot_contacts[0].name;
    robot_state_.lf_leg_mode.surface_normal = foot_contacts->foot_contacts[0].surface_normal;

//    robot_state_.rf_leg_mode.support_leg = foot_contacts->foot_contacts[1].is_contact;
    robot_state_.rf_leg_mode.name = foot_contacts->foot_contacts[1].name;
    robot_state_.rf_leg_mode.surface_normal = foot_contacts->foot_contacts[1].surface_normal;

//    robot_state_.rh_leg_mode.support_leg = foot_contacts->foot_contacts[2].is_contact;
    robot_state_.rh_leg_mode.name = foot_contacts->foot_contacts[2].name;
    robot_state_.rh_leg_mode.surface_normal = foot_contacts->foot_contacts[2].surface_normal;

//    robot_state_.lh_leg_mode.support_leg = foot_contacts->foot_contacts[3].is_contact;
    robot_state_.lh_leg_mode.name = foot_contacts->foot_contacts[3].name;
    robot_state_.lh_leg_mode.surface_normal = foot_contacts->foot_contacts[3].surface_normal;
//    ROS_ERROR("footContactsCallback");
//    free_gait_msgs::RobotState rs;
//    robot_state_pub_.publish(rs);

  }

//  void StateEstimateController::IMUmsgCallback(const sensor_msgs::ImuConstPtr& imu_msg)
//  {

//  }

}
PLUGINLIB_EXPORT_CLASS(balance_controller::StateEstimateController, controller_interface::ControllerBase)
