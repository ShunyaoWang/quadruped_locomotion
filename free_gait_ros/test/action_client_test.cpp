/*
 *  filename.cpp
 *  Descriotion:
 *
 *  Created on: Feb 25, 2018
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#include "free_gait_ros/free_gait_ros.hpp"
#include "free_gait_msgs/ExecuteStepsActionGoal.h"
#include "free_gait_msgs/RobotState.h"
#include "free_gait_core/free_gait_core.hpp"
#include "free_gait_msgs/Footstep.h"
#include "pluginlib/class_loader.h"

using namespace free_gait;

class ActionClientTest
{
public:
  ActionClientTest(const ros::NodeHandle& node_handle)//, FreeGaitActionClient& actionClient)
    : nodeHandle_(node_handle),
      is_done(false),
      is_active(false),
      is_updated(false)
//      actionClient_(nodeHandle_)
  {
    initialize();
    action_client_ptr.reset(new free_gait::FreeGaitActionClient(nodeHandle_));
//    free_gait::FreeGaitActionClient actionClient(nodeHandle_);
    base_pose_sub = nodeHandle_.subscribe("/gazebo/robot_states", 1 , &ActionClientTest::basePoseCallback, this);
//    actionClient.registerCallback(std::bind(&ActionClientTest::activeCallback, this),
//                                  std::bind(&ActionClientTest::feedbackCallback, this, std::placeholders::_1),
//                                  std::bind(&ActionClientTest::doneCallback, this, std::placeholders::_1, std::placeholders::_2));

    action_client_ptr->registerCallback(boost::bind(&ActionClientTest::activeCallback, this),
                                  boost::bind(&ActionClientTest::feedbackCallback, this, _1),
                                  boost::bind(&ActionClientTest::doneCallback, this, _1, _2));


  }

  ~ActionClientTest() {

  }

  void initialize()
  {
    surface_normal.vector.x = 0.0;
    surface_normal.vector.y = 0.0;
    surface_normal.vector.z = 1.0;
    height = 0.4;
    step_number = 0;
  }

  bool sendGoal(const free_gait_msgs::ExecuteStepsGoal& goal)
  {
    action_client_ptr->sendGoal(goal);
    return true;
  }

  void basePoseCallback(const free_gait_msgs::RobotStateConstPtr& robot_state)
  {
    geometry_msgs::PoseWithCovariance base_pose_in_world_;
    base_pose_in_world_.pose = robot_state->base_pose.pose.pose;
    base_pose = Pose(Position(base_pose_in_world_.pose.position.x,
                          base_pose_in_world_.pose.position.y,
                          base_pose_in_world_.pose.position.z),
                 RotationQuaternion(base_pose_in_world_.pose.orientation.w,
                                    base_pose_in_world_.pose.orientation.x,
                                    base_pose_in_world_.pose.orientation.y,
                                    base_pose_in_world_.pose.orientation.z));
//    ROS_INFO("base pose is updated");
//    std::cout<<base_pose<<std::endl;
    is_updated = true;

  };

  void getFootholdInWorld(const Position& foothold_in_base,
                          geometry_msgs::PointStamped& foodhold_in_wolrd)
  {
    Position lf_position = base_pose.getRotation().rotate(foothold_in_base) + base_pose.getPosition();
    foodhold_in_wolrd.point.x = lf_position(0);
    foodhold_in_wolrd.point.y = lf_position(1);
    foodhold_in_wolrd.point.z = lf_position(2);
    ROS_INFO("Generate One Foothold : ");
    std::cout<<lf_position<<std::endl;
    foodhold_in_wolrd.header.frame_id = "odom";
  }

  bool isUpdated()
  {
    return is_updated;
  }
  bool isDone()
  {
    return is_done;
  }
  bool isActive()
  {
    return is_active;
  }

  void activeCallback()
  {
    is_active = true;
    is_done = false;
    ROS_INFO("Goal just went active");
  }

  void feedbackCallback(const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback)
  {
    ROS_INFO("step time is : %f", feedback->phase);
  }

  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const free_gait_msgs::ExecuteStepsResult& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    is_done = true;
    is_active = false;
  }

  free_gait_msgs::Step generateStepForLeg(const std::string& leg_name, const geometry_msgs::PointStamped& foothold_in_world)
  {
    free_gait_msgs::Step step_msg;
    step_msg.id = std::to_string(step_number++);
    free_gait_msgs::BaseAuto base_auto_msg;
    base_auto_msg.height = height;
    base_auto_msg.average_angular_velocity = 0.5;
    base_auto_msg.average_linear_velocity = 0.5;
    base_auto_msg.ignore_timing_of_leg_motion = true;
    base_auto_msg.support_margin = 0.0;
    step_msg.base_auto.push_back(base_auto_msg);

    free_gait_msgs::Footstep footstep_msg;
    footstep_msg.name = leg_name;
    footstep_msg.target.point = foothold_in_world.point;
    footstep_msg.target.header.frame_id = "odom";
    footstep_msg.average_velocity = 0.15;
    footstep_msg.profile_height = 0.15;
    footstep_msg.profile_type = "triangle";
    footstep_msg.ignore_contact = false;
    footstep_msg.ignore_for_pose_adaptation = false;
    footstep_msg.surface_normal = surface_normal;
    step_msg.footstep.push_back(footstep_msg);

    return step_msg;
  }

private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber base_pose_sub;
  Pose base_pose;
  bool is_updated, is_done, is_active;
  std::unique_ptr<free_gait::FreeGaitActionClient> action_client_ptr;
  double height;
  int step_number;
  Position LF_nominal, RF_nominal, LH_nominal, RH_nominal;
  geometry_msgs::Vector3Stamped surface_normal;
  geometry_msgs::PointStamped lf_foot_holds, rf_foot_holds, lh_foot_holds, rh_foot_holds;

  LinearVelocity desired_linear_velocity_;
  LocalAngularVelocity desired_angular_velocity_;
  //  free_gait::FreeGaitActionClient actionClient_;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "action_client_test_node");
  ros::NodeHandle nodeHandle("~");
//  FreeGaitActionClient action_client(nodeHandle);
  ActionClientTest action_client_test(nodeHandle);//, action_client);
//  AdapterDummy adapter;
//  StepRosConverter convert(adapter);
//  free_gait::FreeGaitActionClient actionClient(nodeHandle);
//  ros::Subscriber base_pose_sub = nodeHandle.subscribe("/gazebo/robot_state", 1 , basePoseCallback);
//  actionClient.registerCallback(&free_gait::FreeGaitActionClient::activeCallback(),
//                                free)
  free_gait_msgs::ExecuteStepsGoal steps_goal;
  geometry_msgs::PointStamped lf_foot_holds, rf_foot_holds, lh_foot_holds, rh_foot_holds;
  Position LF_nominal, RF_nominal, LH_nominal, RH_nominal;
  double height = 0.4;
  LF_nominal = Position(0.4,0.275,-height);
  RF_nominal = Position(0.4,-0.275,-height);
  LH_nominal = Position(-0.4,0.275,-height);
  RH_nominal = Position(-0.4,-0.275,-height);
  int i = 0;
  ros::Rate rate(100);
  while (ros::ok()) {
      if(!action_client_test.isUpdated()){
          ROS_INFO("wait for update base pose");
        }else if(!action_client_test.isActive()){
//          free_gait_msgs::Step step_msg;
          //! LF_LEG step *********************************************
          action_client_test.getFootholdInWorld(LF_nominal, lf_foot_holds);
          steps_goal.steps.push_back(action_client_test.generateStepForLeg("LF_LEG", lf_foot_holds));
          //! RH_LEG step *********************************************
          action_client_test.getFootholdInWorld(RH_nominal, rh_foot_holds);
          steps_goal.steps.push_back(action_client_test.generateStepForLeg("RH_LEG", rh_foot_holds));
          //! RH_LEG step *********************************************
          action_client_test.getFootholdInWorld(RF_nominal, rf_foot_holds);
          steps_goal.steps.push_back(action_client_test.generateStepForLeg("RF_LEG", rf_foot_holds));
          //! RH_LEG step *********************************************
          action_client_test.getFootholdInWorld(LH_nominal, lh_foot_holds);
          steps_goal.steps.push_back(action_client_test.generateStepForLeg("LH_LEG", lh_foot_holds));



          action_client_test.sendGoal(steps_goal);

        }else{
//          ROS_INFO("Step action Active, Execution");
        }

//      ros::Duration(0.01).sleep();
      ros::spinOnce();
      rate.sleep();
    }


  return 0;
}
