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
      AdapterRos_(nodehandle, free_gait::AdapterRos::AdapterType::Gazebo)
  {
    nodeHandle_.getParam("/use_gazebo", use_gazebo);
    nodeHandle_.getParam("/kinematics_control",is_kinematics_control);
    nodeHandle_.getParam("/free_gait/stop_execution_service", stop_service_name_);
    nodeHandle_.getParam("/free_gait/pause_execution_service", pause_service_name_);
    if(use_gazebo){
      adapter.reset(AdapterRos_.getAdapterPtr());
      AdapterRos_.subscribeToRobotState();
    } else {
      adapter.reset(adapter_loader.createUnmanagedInstance("free_gait_ros/AdapterDummy"));
    };
    state.reset(new State());
    parameters.reset(new StepParameters());
    completer.reset(new StepCompleter(*parameters, *adapter));
    computer.reset(new StepComputer());

    AdapterRos_.updateAdapterWithState();
    //! WSHY: get the initial pose and set to the fisrt pose command
    state->setPositionWorldToBaseInWorldFrame(AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame());
    state->setOrientationBaseToWorld(AdapterRos_.getAdapter().getOrientationBaseToWorld());
    state->setLinearVelocityBaseInWorldFrame(AdapterRos_.getAdapter().getLinearVelocityBaseInWorldFrame());
    state->setAngularVelocityBaseInBaseFrame(AdapterRos_.getAdapter().getAngularVelocityBaseInBaseFrame());
    std::cout<<*state<<std::endl;

    executor.reset(new Executor(*completer, *computer, *adapter, *state));
    rosPublisher.reset(new StateRosPublisher(nodeHandle_, *adapter));
    if(is_kinematics_control){
      rosPublisher->setTfPrefix("/ideal");
      }else if (!is_kinematics_control) {
      rosPublisher->setTfPrefix("/desired");
    }

    executor->initialize();

    AdapterRos_.updateAdapterWithState();
//    AdvertiseServiceOptions pause_service_option = Advertise("a", boost::bind(&ActionServerTest::PauseServiceCallback, this, _1, _2), ros::VoidConstPtr())
    pause_service_server_ = nodeHandle_.advertiseService(pause_service_name_, &ActionServerTest::PauseServiceCallback, this);
    stop_service_server_ = nodeHandle_.advertiseService(stop_service_name_, &ActionServerTest::StopServiceCallback, this);
    joint_state_pub_ = nodeHandle_.advertise<sensor_msgs::JointState>("all_joint_position", 1);
    allJointStates_.position.resize(12);

    server_.reset(new FreeGaitActionServer(nodeHandle_, "/free_gait/action_server", *executor, *adapter));
    server_->initialize();
    server_->start();
//    server_->update();
    action_server_thread_ = boost::thread(boost::bind(&ActionServerTest::ActionServerThread, this));

  }
  ~ActionServerTest() {}

  void ActionServerThread()//(const FreeGaitActionServer& server)
  {
      ROS_INFO("In action server thread");
      double dt = 0.01;// change dt cause problem?
      double time = 0.0;
      ros::Rate rate(100);
      ros::Duration(0.5).sleep();
      cout<<"Current base position : "<<AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame()<<endl;
      AdapterRos_.updateAdapterWithState();
      cout<<"Current base position : "<<AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame()<<endl;
//      //! WSHY: get the initial pose and set to the fisrt pose command
      state->setPositionWorldToBaseInWorldFrame(AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame());
      state->setOrientationBaseToWorld(AdapterRos_.getAdapter().getOrientationBaseToWorld());
      state->setLinearVelocityBaseInWorldFrame(AdapterRos_.getAdapter().getLinearVelocityBaseInWorldFrame());
      state->setAngularVelocityBaseInBaseFrame(AdapterRos_.getAdapter().getAngularVelocityBaseInBaseFrame());

      for(int i=0;i<12;i++){
          allJointStates_.position[i] = adapter->getState().getJointPositionsToReach()(i);
        }
      if(is_kinematics_control)
        joint_state_pub_.publish(allJointStates_);
      std::cout<<AdapterRos_.getAdapter().getState()<<std::endl;
      rosPublisher->publish(AdapterRos_.getAdapter().getState());
      while (ros::ok()) {
          server_->update();
//          AdapterRos_.updateAdapterWithState();
//          cout<<"Current base position : "<<AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame()<<endl;

//          TODO(Shunyao): How to Update?
//          cout<<"Current base position : "<<AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame()<<endl;
//          cout<<"joint position: "<<AdapterRos_.getAdapter().getAllJointPositions()<<endl;
          if (!executor->getQueue().empty()&&!is_pause&&!is_stop) {
            AdapterRos_.updateAdapterWithState();
            executor->advance(dt, true);
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
              rosPublisher->publish(adapter->getState());
              }
            } else{ // directly publish current state
//              state->setPositionWorldToBaseInWorldFrame(AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame());
//              state->setOrientationBaseToWorld(AdapterRos_.getAdapter().getOrientationBaseToWorld());
//              state->setLinearVelocityBaseInWorldFrame(AdapterRos_.getAdapter().getLinearVelocityBaseInWorldFrame());
//              state->setAngularVelocityBaseInBaseFrame(AdapterRos_.getAdapter().getAngularVelocityBaseInBaseFrame());
//              if(!is_kinematics_control){
//                  //! WSHY: publish robot state to balance controller
//                rosPublisher->publish(*state);
//                }
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
private:
//  FreeGaitActionServer server_;
  std::unique_ptr<FreeGaitActionServer> server_;
  boost::thread action_server_thread_;
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

  sensor_msgs::JointState allJointStates_;
  ros::Publisher joint_state_pub_;
  ros::ServiceServer pause_service_server_, stop_service_server_;
  std::string pause_service_name_, stop_service_name_;
  bool use_gazebo, is_pause, is_stop, is_kinematics_control;
};



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "action_server_test");
    ros::NodeHandle nh("~");
    ros::ServiceServer pause_service_server, stop_service_server;
    ActionServerTest ac_test(nh);
//    pause_service_server = nh.advertiseService("/free_gait/pause_execution_service", &ActionServerTest::PauseServiceCallback, &ac_test);

    ros::spin();
//    pluginlib::ClassLoader<AdapterBase> adapter_loader("free_gait_ros", "free_gait::AdapterBase");
//    std::unique_ptr<AdapterBase> adapter;
//    adapter.reset(adapter_loader.createUnmanagedInstance("free_gait_ros/AdapterDummy"));


//    double dt = 0.001;
//    double time = 0.0;
////    ros::Time t_start = ros::Time::now();
//    ros::Rate rate(1000);
//    ros::spin();

//        while (!executor.getQueue().empty()) {
//          executor.advance(dt, true);
//          time = time + dt;
//          rosPublisher.publish(adapter->getState());

//        }

//    ros::Time t_end = ros::Time::now();
//    cout<<"end time : "<<time<<endl;
//    cout<<"Real time costs is : "<<t_end-t_start<<endl;
    return 0;
}
