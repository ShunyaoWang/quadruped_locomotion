/*
 *  action_server_test.cpp
 *  Descriotion:
 *
 *  Created on: Aug 31, 2017
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#include "free_gait_ros/free_gait_ros.hpp"
#include "free_gait_core/free_gait_core.hpp"
#include "pluginlib/class_loader.h"
#include "boost/bind.hpp"
#include "boost/thread.hpp"

#include "sensor_msgs/JointState.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "ros/advertise_service_options.h"
#include "free_gait_msgs/SetLimbConfigure.h"

#include "free_gait_ros/gait_generate_client.hpp"

using namespace free_gait;
using namespace std;
class ActionServerTest
{
public:
  ActionServerTest(ros::NodeHandle& nodehandle)
    : nodeHandle_(nodehandle),
      adapter_loader("free_gait_ros", "free_gait::AdapterBase"),
      use_gazebo(true),
      is_pause(false),
      is_stop(false),
      is_kinematics_control(true),
      is_start_gait(false),
      AdapterRos_(nodehandle, free_gait::AdapterRos::AdapterType::Gazebo),
      gait_generate_client_(nodehandle)
  {
    nodeHandle_.getParam("/use_gazebo", use_gazebo);
    nodeHandle_.getParam("/kinematics_control",is_kinematics_control);
    nodeHandle_.getParam("/free_gait/stop_execution_service", stop_service_name_);
    nodeHandle_.getParam("/free_gait/pause_execution_service", pause_service_name_);
//    nodeHandle_.getParam("/gait_generate/")
    if(use_gazebo){
      adapter.reset(AdapterRos_.getAdapterPtr());
      AdapterRos_.subscribeToRobotState();//moren parameters to /gazebo/robot_state, and update robot state.
    } else {
      adapter.reset(adapter_loader.createUnmanagedInstance("free_gait_ros/AdapterDummy"));
    };
    state.reset(new State());
    parameters.reset(new StepParameters());
    completer.reset(new StepCompleter(*parameters, *adapter));
    computer.reset(new StepComputer());

    AdapterRos_.updateAdapterWithState();//this state is /gazebo/robot_state
    //in fact, this is to sync state and /gazebo/robot_state
    //! WSHY: get the initial pose and set to the fisrt pose command
    state->setPositionWorldToBaseInWorldFrame(AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame());
    state->setOrientationBaseToWorld(AdapterRos_.getAdapter().getOrientationBaseToWorld());
    state->setLinearVelocityBaseInWorldFrame(AdapterRos_.getAdapter().getLinearVelocityBaseInWorldFrame());
    state->setAngularVelocityBaseInBaseFrame(AdapterRos_.getAdapter().getAngularVelocityBaseInBaseFrame());
    std::cout<<*state<<std::endl;

    executor.reset(new Executor(*completer, *computer, *adapter, *state));
    rosPublisher.reset(new StateRosPublisher(nodeHandle_, *adapter));//
    if(is_kinematics_control){//means what?
      rosPublisher->setTfPrefix("/ideal");
      }else if (!is_kinematics_control) {
      rosPublisher->setTfPrefix("/desired");
    }

    executor->initialize();

    AdapterRos_.updateAdapterWithState();//
//    AdvertiseServiceOptions pause_service_option = Advertise("a", boost::bind(&ActionServerTest::PauseServiceCallback, this, _1, _2), ros::VoidConstPtr())
    //receive the request, then use the funtion pauseservicecallback
    pause_service_server_ = nodeHandle_.advertiseService(pause_service_name_, &ActionServerTest::PauseServiceCallback, this);
    stop_service_server_ = nodeHandle_.advertiseService(stop_service_name_, &ActionServerTest::StopServiceCallback, this);
    gait_start_server_ = nodeHandle_.advertiseService("/gait_generate_switch", &ActionServerTest::GaitGenerateSwitchCallback, this);
    pace_start_server_ = nodeHandle_.advertiseService("/pace_switch", &ActionServerTest::PaceSwitchCallback, this);

    limb_configure_switch_server_ = nodeHandle_.advertiseService("/limb_configure", &ActionServerTest::SwitchLimbConfigureCallback, this);

    joint_state_pub_ = nodeHandle_.advertise<sensor_msgs::JointState>("all_joint_position", 1);//ros::publisher
    allJointStates_.position.resize(12);

    //free_gait_action server
    server_.reset(new FreeGaitActionServer(nodeHandle_, "/free_gait/action_server", *executor, *adapter));
    server_->initialize();
    server_->start();
//    server_->update();
    action_server_thread_ = boost::thread(boost::bind(&ActionServerTest::ActionServerThread, this));
    gait_generate_thread_ = boost::thread(boost::bind(&ActionServerTest::GaitGenerateThread, this));

  }
  ~ActionServerTest() {}

  void ActionServerThread()//(const FreeGaitActionServer& server)
  {
      ROS_INFO("In action server thread");
      parameters->footstepParameters.minimumDuration_ = 0.45;
      parameters->baseTargetParameters.minimumDuration = 0.45;
      double dt = 0.01;// change dt cause problem?
      double time = 0.0;
      ros::Rate rate(100);
      ros::Duration(0.5).sleep();
      cout<<"Current base position : "<<AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame()<<endl;
      AdapterRos_.updateAdapterWithState();
      executor->reset(); //! WSHY: adapter update has wrong
      cout<<adapter->getState()<<endl;
      cout<<"Current base position : "<<AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame()<<endl;
//      //! WSHY: get the initial pose and set to the fisrt pose command
      state->setPositionWorldToBaseInWorldFrame(AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame());
      state->setOrientationBaseToWorld(AdapterRos_.getAdapter().getOrientationBaseToWorld());
      state->setLinearVelocityBaseInWorldFrame(AdapterRos_.getAdapter().getLinearVelocityBaseInWorldFrame());
      state->setAngularVelocityBaseInBaseFrame(AdapterRos_.getAdapter().getAngularVelocityBaseInBaseFrame());

      for(int i=0;i<12;i++){
          allJointStates_.position[i] = adapter->getState().getJointPositionFeedback()(i);
        }
      if(is_kinematics_control)
        joint_state_pub_.publish(allJointStates_);
      std::cout<<AdapterRos_.getAdapter().getState()<<std::endl;
      rosPublisher->publish(AdapterRos_.getAdapter().getState());
      while (ros::ok()) {
          server_->update();
          if (!executor->getQueue().empty()&&!is_pause&&!is_stop) {
            boost::recursive_mutex::scoped_lock lock(r_mutex_);
            AdapterRos_.updateAdapterWithState();

            executor->advance(dt, false);
            if(is_start_gait){
                state->setLinearVelocityBaseInWorldFrame(desired_linear_velocity_);
                state->setAngularVelocityBaseInBaseFrame(desired_angular_velocity_);
              }
            time = time + dt;
            for(int i=0;i<12;i++){
                allJointStates_.position[i] = adapter->getState().getJointPositions()(i);
              }
            if(is_kinematics_control){
                //! WSHY: directly publish joint positions
              joint_state_pub_.publish(allJointStates_);
              }
//            cout<<"*************************************send joint command : "<<endl<<adapter->getState().getJointPositions()<<endl;
            if(!use_gazebo){
                //! WSHY: fake mode, publish fake state to rviz visualization
              rosPublisher->publish(adapter->getState());
              }
            if(!is_kinematics_control){
                //! WSHY: publish robot state to balance controller
              rosPublisher->publish(adapter->getState(), executor->getQueue());
              }
            lock.unlock();
            } else{
            }

          rate.sleep();
        }

  }

  void GaitGenerateThread()
  {
    ROS_INFO("In GaitGenerateThread thread");
    ros::Rate rate(100);
    double dt = 0.01;
    gait_generate_client_.initializeTrot(0.45, 0.45);
//    gait_generate_client_.initializePace(0.45, 3*0.5);
    while (ros::ok()) {
//        ROS_INFO("Gait Generate updated Once");
        if(is_start_gait){
            boost::recursive_mutex::scoped_lock lock(r_mutex_);
            AdapterRos_.updateAdapterWithState();//sync /gazebo/robot_state and state
            gait_generate_client_.copyRobotState(AdapterRos_.getAdapter().getState());//gait_generate_client_.state = /gazebo/robot_state
            gait_generate_client_.advance(dt);//judge the phase
            gait_generate_client_.generateFootHolds("foot_print");//generate footholds
            gait_generate_client_.updateBaseMotion(desired_linear_velocity_, desired_angular_velocity_);
//            ROS_WARN_STREAM("Desired Velocity :"<<desired_linear_velocity_<<endl);
            lock.unlock();
            gait_generate_client_.sendMotionGoal();
          }

        rate.sleep();
      }
  }

  bool StopServiceCallback(std_srvs::Trigger::Request& request,
                              std_srvs::Trigger::Response& response)
  {
    ROS_INFO("STOP Trigger");
    server_->setPreempted();
    is_stop = true;
    response.success = true;
    return true;
  }

  bool PauseServiceCallback(std_srvs::SetBool::Request& request,
                           std_srvs::SetBool::Response& response)
  {
    if(request.data == false){
        is_pause = false;
        is_stop = false;
        ROS_INFO("Start....");
      }
    if(request.data == true){
      is_pause = true;
      ROS_INFO("STOP....");
      }
    response.success = true;
    return true;
  }

  bool GaitGenerateSwitchCallback(std_srvs::SetBool::Request& request,
                                  std_srvs::SetBool::Response& response)
  {
    if(request.data == false){
        is_start_gait = false;
        ROS_INFO("STOP GAIT....");
      }
    if(request.data == true){
        is_start_gait = true;
        gait_generate_client_.initializeTrot(0.45,0.45);
//        gait_generate_client_.initializePace(0.45, 3*0.5);
      ROS_INFO("START GAIT....");
      }
    response.success = true;
    return true;
  }

  bool PaceSwitchCallback(std_srvs::SetBool::Request& request,
                                  std_srvs::SetBool::Response& response)
  {
    if(request.data == false){
        is_start_gait = false;
        ROS_INFO("STOP GAIT....");
      }
    if(request.data == true){
        is_start_gait = true;
        gait_generate_client_.initializePace(0.5,1.5);
//        gait_generate_client_.initializePace(0.45, 3*0.5);
      ROS_INFO("START GAIT....");
      }
    response.success = true;
    return true;
  }

  bool SwitchLimbConfigureCallback(free_gait_msgs::SetLimbConfigure::Request& request,
                                   free_gait_msgs::SetLimbConfigure::Response& response)
  {
    response.result = state->setLimbConfigure(request.configure);
    return true;
  }

private:
//  FreeGaitActionServer server_;
  std::unique_ptr<FreeGaitActionServer> server_;
  boost::thread action_server_thread_, gait_generate_thread_;
  ros::NodeHandle nodeHandle_;

  free_gait::AdapterRos AdapterRos_;

  std::unique_ptr<AdapterBase> adapter;
  pluginlib::ClassLoader<AdapterBase> adapter_loader;


  std::unique_ptr<State> state;
  std::unique_ptr<StepParameters> parameters;
  std::unique_ptr<StepCompleter> completer;
  std::unique_ptr<StepComputer> computer;
  std::unique_ptr<Executor> executor;
  std::unique_ptr<StateRosPublisher> rosPublisher;

  GaitGenerateClient gait_generate_client_;

  sensor_msgs::JointState allJointStates_;
  ros::Publisher joint_state_pub_;
  ros::ServiceServer pause_service_server_, stop_service_server_,
  gait_start_server_, limb_configure_switch_server_, pace_start_server_;
  std::string pause_service_name_, stop_service_name_;
  bool use_gazebo, is_pause, is_stop, is_kinematics_control, is_start_gait;
  /**
   * @brief r_mutex_
   */
  boost::recursive_mutex r_mutex_;
  LinearVelocity desired_linear_velocity_;
  LocalAngularVelocity desired_angular_velocity_;
};



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "action_server_test");
    ros::NodeHandle nh("~");
    ActionServerTest ac_test(nh);
    ros::spin();

    return 0;
}
