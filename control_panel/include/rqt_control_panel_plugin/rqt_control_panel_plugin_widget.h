#ifndef CONTROL_PANEL_PLUGIN_WIDGET
#define CONTROL_PANEL_PLUGIN_WIDGET

#include <QWidget>
#include "iostream"
#include "ros/ros.h"

#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "free_gait_msgs/SetLimbConfigure.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"

#include "boost/thread.hpp"

typedef enum ControlMethod{
  BALANCE,
  JOINT_POSITION,
  JOINT_VELOCITY,
  JOINT_EFFORT,
}ControlMethod;

namespace Ui {
  class rqt_control_panel_plugin_widget;
}

class rqt_control_panel_plugin_widget : public QWidget
{
  Q_OBJECT

public:
  explicit rqt_control_panel_plugin_widget(const ros::NodeHandle& nodehandle, QWidget *parent = nullptr);
  ~rqt_control_panel_plugin_widget();

private slots:
  void on_Controllers_currentChanged(int index);

  void on_trotButton_clicked();

  void on_stopBotton_clicked();

  void on_setJointPositionButton_clicked();

  void on_stopJointVelocityButton_clicked();

  void on_stopPositionButton_clicked();

  void on_stopJointEffortButton_clicked();

  void on_setJointVelocityButton_clicked();

  void on_setJointEffortButton_clicked();

  void on_setVelocityBotton_clicked();

  void on_paceButton_clicked();

  void on_startSingleLegContoller_clicked();

  void on_stopSingleLegController_clicked();

private:
  Ui::rqt_control_panel_plugin_widget *ui;
  ros::NodeHandle nodehandle_;

  boost::thread updateCommandThread_;

  ros::ServiceClient switchControlMethodClient_;
  ros::ServiceClient switchControllerClient_;
  ros::ServiceClient listControllerClient_;
  ros::ServiceClient trotSwitchClient_;
  ros::ServiceClient paceSwitchClient_;

  ros::Publisher eStopPublisher_;
  ros::Publisher jointPositionCommandPublisher_;
  ros::Publisher jointVelocityCommandPublisher_;
  ros::Publisher jointEffortCommandPublisher_;
  ros::Publisher baseVelocityCommandPublisher_;

  ros::Subscriber jointStateSubscriber_;

  ControlMethod control_method_;
  sensor_msgs::JointState joint_states_;

  void displayOutputInfos(const std::string &color,
                                                  const QString &context);

  void updateCommandLoop();

  bool updateJointState();

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
};

#endif // CONTROL_PANEL_PLUGIN_WIDGET
