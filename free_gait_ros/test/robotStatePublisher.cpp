/*
 *  filename.cpp
 *  Descriotion:
 *
 *  Created on: Aug 31, 2017
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "free_gait_msgs/RobotState.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "iostream"

free_gait_msgs::RobotState robot_state_;
void basePoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& fakePoseMsg_)
{
  robot_state_.base_pose.pose = fakePoseMsg_->pose;
  robot_state_.base_pose.child_frame_id = "/base_link";
  robot_state_.base_pose.header.frame_id = "/odom";
}

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{
  robot_state_.lf_leg_joints.header = joint_states->header;
  robot_state_.lf_leg_joints.name.push_back("front_left_1_joint");
  robot_state_.lf_leg_joints.position.push_back(joint_states->position[0]);
  robot_state_.lf_leg_joints.name.push_back("front_left_2_joint");
  robot_state_.lf_leg_joints.position.push_back(joint_states->position[1]);
  robot_state_.lf_leg_joints.name.push_back("front_left_3_joint");
  robot_state_.lf_leg_joints.position.push_back(joint_states->position[2]);

  robot_state_.rf_leg_joints.header = joint_states->header;
  robot_state_.rf_leg_joints.name.push_back("front_right_1_joint");
  robot_state_.rf_leg_joints.position.push_back(joint_states->position[3]);
  robot_state_.rf_leg_joints.name.push_back("front_right_2_joint");
  robot_state_.rf_leg_joints.position.push_back(joint_states->position[4]);
  robot_state_.rf_leg_joints.name.push_back("front_right_3_joint");
  robot_state_.rf_leg_joints.position.push_back(joint_states->position[5]);

  robot_state_.lh_leg_joints.header = joint_states->header;
  robot_state_.lh_leg_joints.name.push_back("rear_left_1_joint");
  robot_state_.lh_leg_joints.position.push_back(joint_states->position[6]);
  robot_state_.lh_leg_joints.name.push_back("rear_left_2_joint");
  robot_state_.lh_leg_joints.position.push_back(joint_states->position[7]);
  robot_state_.lh_leg_joints.name.push_back("rear_left_3_joint");
  robot_state_.lh_leg_joints.position.push_back(joint_states->position[8]);

  robot_state_.rh_leg_joints.header = joint_states->header;
  robot_state_.rh_leg_joints.name.push_back("rear_right_1_joint");
  robot_state_.rh_leg_joints.position.push_back(joint_states->position[9]);
  robot_state_.rh_leg_joints.name.push_back("rear_right_2_joint");
  robot_state_.rh_leg_joints.position.push_back(joint_states->position[10]);
  robot_state_.rh_leg_joints.name.push_back("rear_right_3_joint");
  robot_state_.rh_leg_joints.position.push_back(joint_states->position[11]);
  std::cout<<"Publish robot state with LF_LEG joint2 is : "<<robot_state_.lf_leg_joints.position[1]<<std::endl;

}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "robot_state_pub_node");
  ros::NodeHandle nodeHandle_("~");
  ros::Subscriber gazebo_joint_states_sub_ = nodeHandle_.subscribe("/joint_states", 1, jointStatesCallback);
  ros::Subscriber pose_sub = nodeHandle_.subscribe("/pose_pub_node/base_pose", 1, basePoseCallback);
  ros::Publisher robot_state_pub_ = nodeHandle_.advertise<free_gait_msgs::RobotState>("/gazebo/robot_states", 1);
  ros::Rate rate(100);
  while (ros::ok()) {
      robot_state_pub_.publish(robot_state_);
      std::cout<<"Publish robot state with position X is : "<<robot_state_.base_pose.pose.pose.position.x<<std::endl;
//      std::cout<<"Publish robot state with LF_LEG joint2 is : "<<robot_state_.lf_leg_joints.position[1]<<std::endl;
      ros::spinOnce();
      rate.sleep();
    }

  return 0;
}
