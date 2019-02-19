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

using namespace free_gait;
using namespace std;
class ActionServerTest
{
public:
  ActionServerTest(ros::NodeHandle& nodehandle)
    : nodeHandle_(nodehandle),
      adapter_loader("free_gait_ros", "free_gait::AdapterBase")
  {
    adapter.reset(adapter_loader.createUnmanagedInstance("free_gait_ros/AdapterDummy"));
    state.reset(new State());
    parameters.reset(new StepParameters());
    completer.reset(new StepCompleter(*parameters, *adapter));
    computer.reset(new StepComputer());
    executor.reset(new Executor(*completer, *computer, *adapter, *state));
    rosPublisher.reset(new StateRosPublisher(nodeHandle_, *adapter));
    executor->initialize();

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
      double dt = 0.001;
      double time = 0.0;
      ros::Rate rate(100);
      while (ros::ok()) {
          server_->update();
//          TODO(Shunyao): How to Update?
          if (!executor->getQueue().empty()) {
            executor->advance(dt, true);
            time = time + dt;
            rosPublisher->publish(adapter->getState());
          }
          rate.sleep();
        }

  }
private:
//  FreeGaitActionServer server_;
  std::unique_ptr<FreeGaitActionServer> server_;
  boost::thread action_server_thread_;
  ros::NodeHandle nodeHandle_;
  std::unique_ptr<AdapterBase> adapter;
  pluginlib::ClassLoader<AdapterBase> adapter_loader;
  std::unique_ptr<State> state;
  std::unique_ptr<StepParameters> parameters;
  std::unique_ptr<StepCompleter> completer;
  std::unique_ptr<StepComputer> computer;
  std::unique_ptr<Executor> executor;
  std::unique_ptr<StateRosPublisher> rosPublisher;
};



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "action_server_test");
    ros::NodeHandle nh("~");
    ActionServerTest ac_test(nh);
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
