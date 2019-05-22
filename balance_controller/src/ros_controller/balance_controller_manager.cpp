/*
 *  balance_controller_manager.cpp
 *  Descriotion:
 *
 *  Created on: May,20, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#include "ros/ros.h"
#include "controller_manager/controller_manager.h"
#include "pluginlib/class_loader.h"
#include "ros_ethercat_driver/hardware_interface/ros_ethercat_hardware_interface.hpp"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>
#include <atomic>
#include <signal.h>

std::atomic<bool> quit(false);//signal flag

void got_signal(int)
{
 quit = true;
}

class BalanceControllerManager
{
public:
  BalanceControllerManager(const ros::NodeHandle& nh)
    : nh_(nh)
  {
    EtherCAT_HW_.reset(new ros_ethercat_driver::RobotStateEtherCATHardwareInterface);
    EtherCAT_HW_->init(nh_, nh_);
    controller_manager_.reset(new controller_manager::ControllerManager(EtherCAT_HW_.get()));

    ros::TimerOptions control_timer_options(ros::Duration(0.0025),
                                            boost::bind(&BalanceControllerManager::controlLoop, this, _1),
                                            &update_queue_, false, false);
    control_timer_ = nh_.createTimer(control_timer_options);
//    control_timer_.start();

//    control_loop_thread_ = boost::thread(boost::bind(&BalanceControllerManager::controlLoop, this));

    timer_thread_ = boost::thread(boost::bind(&BalanceControllerManager::timerThread, this));
    control_timer_.start();
  }
  ~BalanceControllerManager()
  {

    EtherCAT_HW_->~RobotStateEtherCATHardwareInterface();
    ROS_INFO("Deconstruct BalanceControllerManager");
  }

  void controlLoop(const ros::TimerEvent&)
  {
//    ROS_INFO("Loop once");
    ros::Time time = ros::Time::now();
    ros::Duration peroid(0.0025);
    EtherCAT_HW_->eStopActive(false);
    EtherCAT_HW_->read(time, peroid);
    controller_manager_->update(time, peroid);
    EtherCAT_HW_->write(time, peroid);
  }

  void controlLoop()
  {
    ros::Time time = ros::Time::now();
    ros::Duration peroid(0.01);
    ros::Rate rate(100);
    while (ros::ok()) {
//        ROS_INFO("Loop once");
        time = ros::Time::now();


        EtherCAT_HW_->read(time, peroid);
        controller_manager_->update(time, peroid);
        EtherCAT_HW_->write(time, peroid);
        rate.sleep();
      }

  }

  void timerThread()
  {
    static const double timeout = 0.0025;
    while (nh_.ok()) {
        update_queue_.callAvailable(ros::WallDuration(timeout));
      }
  }

private:
  ros::NodeHandle nh_;
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  std::shared_ptr<ros_ethercat_driver::RobotStateEtherCATHardwareInterface> EtherCAT_HW_;
  ros::CallbackQueue update_queue_;
  ros::Timer control_timer_;

  boost::thread control_loop_thread_, timer_thread_;

};

//void controlLoop(controller_manager::ControllerManager& cm,
//                 ros_ethercat_driver::RobotStateEtherCATHardwareInterface& EtherCAT_HW)
//{
//      ROS_INFO("Loop once");
//      ros::Time time = ros::Time::now();
//      ros::Duration peroid(1/400);

//      EtherCAT_HW.read(time, peroid);
//      cm.update(time, peroid);
//      EtherCAT_HW.write(time, peroid);
//}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "balance_controller_manager");
  ros::NodeHandle nh, nh_("~");
  BalanceControllerManager balanceControllerManager(nh_);

  struct sigaction sa;
  memset(&sa, 0, sizeof (sa));
  sa.sa_handler = got_signal;
  sigaction(SIGINT, &sa, NULL);

  // Spin
  ros::AsyncSpinner spinner(1); // Use n threads
  spinner.start();
//  ros::waitForShutdown();
  while (ros::ok()) {
      if(quit) break;
    }

  ROS_INFO("balance_controller_manager is shutdown");

//  ros_ethercat_driver::RobotStateEtherCATHardwareInterface EtherCATHardWare;
//  EtherCATHardWare.init(nh_, nh_);
//  ROS_INFO("Finish HW init");
//  controller_manager::ControllerManager controller_manager(&EtherCATHardWare, nh);
//  ROS_INFO("Start controller manager");

//  ros::CallbackQueue update_queue;
//  ros::AsyncSpinner update_spinner(1, &update_queue);

//  ros::Timer control_timer;

//  ros::TimerOptions control_timer_options(ros::Duration(1/400),
//                                  boost::bind(controlLoop, boost::ref(controller_manager), boost::ref(EtherCATHardWare)),
//                                          &update_queue);
//  control_timer = nh.createTimer(control_timer_options);
//  ROS_INFO("Start Ros Spinner");
//  update_spinner.start();
  //  ros::Rate rate(400);
//  ros::Duration peroid = ros::Duration(0.025);
//  ros::Time time = ros::Time::now();
//  while (ros::ok()) {
//      time = ros::Time::now();

//      EtherCAT_HW.read(time, peroid);
//      cm.update(time, peroid);
//      EtherCAT_HW.write(time, peroid);

//      rate.sleep();
//      peroid = rate.cycleTime();
//    }
  return 0;
}
