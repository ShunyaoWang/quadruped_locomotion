#include <rqt_control_panel_plugin/rqt_control_panel_plugin_widget.h>
#include "ui_rqt_control_panel_plugin_widget.h"

rqt_control_panel_plugin_widget::rqt_control_panel_plugin_widget(const ros::NodeHandle& nodehandle, QWidget *parent) :
  QWidget(parent),
  ui(new Ui::rqt_control_panel_plugin_widget),
  nodehandle_(nodehandle)
{
  ui->setupUi(this);

  ui->Controllers->setTabText(0, "Balance");
  ui->Controllers->setTabText(1, "Joint Position");
  ui->Controllers->setTabText(2, "Joint Velocity");
  ui->Controllers->setTabText(3, "Joint Effort");
  ui->Controllers->setTabText(4, "Single Leg Control");

  switchControllerClient_ = nodehandle_.serviceClient<controller_manager_msgs::SwitchController>
      ("/controller_manager/switch_controller", false);
  listControllerClient_ = nodehandle_.serviceClient<controller_manager_msgs::ListControllers>
      ("/controller_manager/list_controllers", false);

  switchControlMethodClient_ = nodehandle_.serviceClient<free_gait_msgs::SetLimbConfigure>
      ("/set_control_method", false);

  trotSwitchClient_ = nodehandle_.serviceClient<std_srvs::SetBool>("/gait_generate_switch", false);

  paceSwitchClient_ = nodehandle_.serviceClient<std_srvs::SetBool>("/pace_switch",false);

  eStopPublisher_ = nodehandle_.advertise<std_msgs::Bool>("/e_stop", 1);

  baseVelocityCommandPublisher_ = nodehandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  jointPositionCommandPublisher_ = nodehandle_.advertise<std_msgs::Float64MultiArray>("/all_joints_position_group_controller/command", 1);

  jointVelocityCommandPublisher_ = nodehandle_.advertise<std_msgs::Float64MultiArray>("/all_joints_velocity_group_controller/command", 1);

  jointEffortCommandPublisher_ = nodehandle_.advertise<std_msgs::Float64MultiArray>("/all_joints_effort_group_controller/command", 1);

  jointStateSubscriber_ = nodehandle_.subscribe("/joint_states", 1, &rqt_control_panel_plugin_widget::jointStateCallback, this);

}

rqt_control_panel_plugin_widget::~rqt_control_panel_plugin_widget()
{
  delete ui;
}

void rqt_control_panel_plugin_widget::updateCommandLoop()
{

}

void rqt_control_panel_plugin_widget::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
  joint_states_.name = joint_state->name;
  joint_states_.header = joint_state->header;
  joint_states_.position = joint_state->position;
  joint_states_.velocity = joint_state->velocity;
  joint_states_.effort = joint_state->effort;

}

bool rqt_control_panel_plugin_widget::updateJointState()
{
  int tab_index = ui->Controllers->currentIndex();
  switch (tab_index) {
    case 1: //joint position
      {
        ui->lf_joint_positon_1->setValue(joint_states_.position[0]);
        ui->lf_joint_positon_2->setValue(joint_states_.position[1]);
        ui->lf_joint_positon_3->setValue(joint_states_.position[2]);

        ui->rf_joint_positon_1->setValue(joint_states_.position[3]);
        ui->rf_joint_positon_2->setValue(joint_states_.position[4]);
        ui->rf_joint_positon_3->setValue(joint_states_.position[5]);

        ui->rh_joint_positon_1->setValue(joint_states_.position[6]);
        ui->rh_joint_positon_2->setValue(joint_states_.position[7]);
        ui->rh_joint_positon_3->setValue(joint_states_.position[8]);

        ui->lh_joint_positon_1->setValue(joint_states_.position[9]);
        ui->lh_joint_positon_2->setValue(joint_states_.position[10]);
        ui->lh_joint_positon_3->setValue(joint_states_.position[11]);
        break;
      }
    case 2: //joint velocity
      {
        ui->lf_joint_velocity_1->setValue(joint_states_.velocity[0]);
        ui->lf_joint_velocity_2->setValue(joint_states_.velocity[1]);
        ui->lf_joint_velocity_3->setValue(joint_states_.velocity[2]);

        ui->rf_joint_velocity_1->setValue(joint_states_.velocity[3]);
        ui->rf_joint_velocity_2->setValue(joint_states_.velocity[4]);
        ui->rf_joint_velocity_3->setValue(joint_states_.velocity[5]);

        ui->rh_joint_velocity_1->setValue(joint_states_.velocity[6]);
        ui->rh_joint_velocity_2->setValue(joint_states_.velocity[7]);
        ui->rh_joint_velocity_3->setValue(joint_states_.velocity[8]);

        ui->lh_joint_velocity_1->setValue(joint_states_.velocity[9]);
        ui->lh_joint_velocity_2->setValue(joint_states_.velocity[10]);
        ui->lh_joint_velocity_3->setValue(joint_states_.velocity[11]);
        break;
      }
    case 3: //joint effort
      {
        ui->lf_joint_effort_1->setValue(joint_states_.effort[0]);
        ui->lf_joint_effort_2->setValue(joint_states_.effort[1]);
        ui->lf_joint_effort_3->setValue(joint_states_.effort[2]);

        ui->rf_joint_effort_1->setValue(joint_states_.effort[3]);
        ui->rf_joint_effort_2->setValue(joint_states_.effort[4]);
        ui->rf_joint_effort_3->setValue(joint_states_.effort[5]);

        ui->rh_joint_effort_1->setValue(joint_states_.effort[6]);
        ui->rh_joint_effort_2->setValue(joint_states_.effort[7]);
        ui->rh_joint_effort_3->setValue(joint_states_.effort[8]);

        ui->lh_joint_effort_1->setValue(joint_states_.effort[9]);
        ui->lh_joint_effort_2->setValue(joint_states_.effort[10]);
        ui->lh_joint_effort_3->setValue(joint_states_.effort[11]);
        break;
      }
    }
}

void rqt_control_panel_plugin_widget::on_Controllers_currentChanged(int index)
{
  controller_manager_msgs::ListControllers list_controllers;
  listControllerClient_.call(list_controllers.request, list_controllers.response);
//  std::cout<<list_controllers.response.controller[0].name<<std::endl;
  std::string current_controller;
  for(auto& controller:list_controllers.response.controller)
    {
      if(controller.name == "all_joints_position_group_controller" && controller.state =="running")
        {
          current_controller = "all_joints_position_group_controller";
          break;
        }
      if(controller.name == "all_joints_velocity_group_controller" && controller.state =="running")
        {
          current_controller = "all_joints_velocity_group_controller";
          break;
        }
      if(controller.name == "all_joints_effort_group_controller" && controller.state =="running")
        {
          current_controller = "all_joints_effort_group_controller";
          break;
        }
      if(controller.name == "all_joints_position_effort_group_controller" && controller.state =="running")
        {
          current_controller = "all_joints_position_effort_group_controller";
          break;
        }
      if(controller.name == "base_balance_controller" && controller.state =="running")
        {
          current_controller = "base_balance_controller";
          break;
        }
      if(controller.name == "single_leg_controller" && controller.state =="running")
        {
          current_controller = "single_leg_controller";
          break;
        }

    }
  controller_manager_msgs::SwitchController controller_switch;
  free_gait_msgs::SetLimbConfigure control_method;
  std_msgs::Bool e_stop_msg;
  e_stop_msg.data = true;

  QString tab_name = ui->Controllers->tabText(index);
  std::cout<<current_controller<<std::endl;
  if(tab_name == "Balance")
    {
      e_stop_msg.data = false;
      eStopPublisher_.publish(e_stop_msg);
      std::cout<<"Balance"<<std::endl;
      controller_switch.request.start_controllers.push_back("base_balance_controller");//the name to start
      controller_switch.request.stop_controllers.push_back(current_controller.c_str());//the name to stop
      controller_switch.request.strictness = controller_switch.request.STRICT;
      /**
         the strictness (BEST_EFFORT or STRICT)
        #    * STRICT means that switching will fail if anything goes wrong (an invalid
        #      controller name, a controller that failed to start, etc. )
        #    * BEST_EFFORT means that even when something goes wrong with on controller,
        #      the service will still try to start/stop the remaining controllers

        */
      switchControllerClient_.call(controller_switch.request, controller_switch.response);

      control_method.request.configure = tab_name.toStdString();//Qstring to std::string, balance means what?
      switchControlMethodClient_.call(control_method.request, control_method.response);//SwitchControllerMethod, balance

      if(controller_switch.response.ok && control_method.response.result)
        {
          control_method_ = BALANCE;// the whole body controller method
          displayOutputInfos("green", "Switch to Balance Controller");
        }

    }else if (tab_name == "Joint Position") {
      std::cout<<"Joint Position"<<std::endl;
      eStopPublisher_.publish(e_stop_msg);
      controller_switch.request.start_controllers.push_back("all_joints_position_group_controller");
      controller_switch.request.stop_controllers.push_back(current_controller.c_str());
      controller_switch.request.strictness = controller_switch.request.STRICT;
      switchControllerClient_.call(controller_switch.request, controller_switch.response);

      control_method.request.configure = tab_name.toStdString();
      switchControlMethodClient_.call(control_method.request, control_method.response);

      if(controller_switch.response.ok && control_method.response.result)
        {
          control_method_ = JOINT_POSITION;
          displayOutputInfos("green", "Switch to Joint Position Controller");
        }
      updateJointState();
    }else if (tab_name == "Joint Velocity") {
      std::cout<<"Joint Velocity"<<std::endl;
      eStopPublisher_.publish(e_stop_msg);
      controller_switch.request.start_controllers.push_back("all_joints_velocity_group_controller");
      controller_switch.request.stop_controllers.push_back(current_controller.c_str());
      controller_switch.request.strictness = controller_switch.request.STRICT;
      switchControllerClient_.call(controller_switch.request, controller_switch.response);

      control_method.request.configure = tab_name.toStdString();
      switchControlMethodClient_.call(control_method.request, control_method.response);

      if(controller_switch.response.ok && control_method.response.result)
        {
          control_method_ = JOINT_VELOCITY;
          displayOutputInfos("green", "Switch to Joint Velocity Controller");
        }
      updateJointState();
    }else if (tab_name == "Joint Effort"){
      std::cout<<"Joint Effort"<<std::endl;
      eStopPublisher_.publish(e_stop_msg);
      controller_switch.request.start_controllers.push_back("all_joints_effort_group_controller");
      controller_switch.request.stop_controllers.push_back(current_controller.c_str());
      controller_switch.request.strictness = controller_switch.request.STRICT;
      switchControllerClient_.call(controller_switch.request, controller_switch.response);

      control_method.request.configure = tab_name.toStdString();
      switchControlMethodClient_.call(control_method.request, control_method.response);

      if(controller_switch.response.ok && control_method.response.result)
        {
          control_method_ = JOINT_EFFORT;
          displayOutputInfos("green", "Switch to Joint Effort Controller");
        }
      updateJointState();
    }else if (tab_name == "Single Leg Control") {
      std::cout<<"Single Leg Controller"<<std::endl;
      eStopPublisher_.publish(e_stop_msg);
      controller_switch.request.start_controllers.push_back("single_leg_controller");
      controller_switch.request.stop_controllers.push_back(current_controller.c_str());
      controller_switch.request.strictness = controller_switch.request.STRICT;
      switchControllerClient_.call(controller_switch.request, controller_switch.response);

      control_method.request.configure = "Joint Effort";
      switchControlMethodClient_.call(control_method.request, control_method.response);

      if(controller_switch.response.ok && control_method.response.result)
        {
          control_method_ = JOINT_EFFORT;
          displayOutputInfos("green", "Switch to Single Leg Controller");
        }
    }
}

void rqt_control_panel_plugin_widget::displayOutputInfos(const std::string &color,
                                                const QString &context)
{

  if(color == "red")
    {
      ui->InfoOutputs->setTextColor(QColor(255,0,0));
      ui->InfoOutputs->insertPlainText(QString("ERROR : ") + context + QString("\n"));
    }
  if(color == "green")
    {
      ui->InfoOutputs->setTextColor(QColor(0,255,0));
      ui->InfoOutputs->insertPlainText(QString("INFO : ") + context + QString("\n"));
    }
  if(color == "yellow")
    {
      ui->InfoOutputs->setTextColor(QColor(255,255,0));
      ui->InfoOutputs->insertPlainText(QString("WARN : ") + context + QString("\n"));
    }

//  ui->info_display->insertPlainText(context + QString("\n"));
}


void rqt_control_panel_plugin_widget::on_trotButton_clicked()
{
  std_srvs::SetBool trot_switch;
  trot_switch.request.data = true;
  trotSwitchClient_.call(trot_switch.request, trot_switch.response);
  if(trot_switch.response.success)
    {
      displayOutputInfos("green", "Starting Troting......");
    }
}

void rqt_control_panel_plugin_widget::on_paceButton_clicked()
{
  std_srvs::SetBool pace_switch;
  pace_switch.request.data = true;
  paceSwitchClient_.call(pace_switch.request, pace_switch.response);
  if(pace_switch.response.success)
    {
      displayOutputInfos("green", "Starting Pacing......");
    }
}

void rqt_control_panel_plugin_widget::on_stopBotton_clicked()
{
  std_srvs::SetBool gait_stop_switch;
  gait_stop_switch.request.data = false;
  trotSwitchClient_.call(gait_stop_switch.request, gait_stop_switch.response);
  paceSwitchClient_.call(gait_stop_switch.request, gait_stop_switch.response);
  if(gait_stop_switch.response.success)
    {
      displayOutputInfos("yellow", "Stop Troting");
    }
}

void rqt_control_panel_plugin_widget::on_setJointPositionButton_clicked()
{
  std_msgs::Bool e_stop_msg;
  e_stop_msg.data = false;

  std_msgs::Float64MultiArray joint_position_command;
  joint_position_command.data.resize(12);

  joint_position_command.data[0] = ui->lf_joint_positon_1->value();
  joint_position_command.data[1] = ui->lf_joint_positon_2->value();
  joint_position_command.data[2] = ui->lf_joint_positon_3->value();

  joint_position_command.data[3] = ui->rf_joint_positon_1->value();
  joint_position_command.data[4] = ui->rf_joint_positon_2->value();
  joint_position_command.data[5] = ui->rf_joint_positon_3->value();

  joint_position_command.data[6] = ui->rh_joint_positon_1->value();
  joint_position_command.data[7] = ui->rh_joint_positon_2->value();
  joint_position_command.data[8] = ui->rh_joint_positon_3->value();

  joint_position_command.data[9] = ui->lh_joint_positon_1->value();
  joint_position_command.data[10] = ui->lh_joint_positon_2->value();
  joint_position_command.data[11] = ui->lh_joint_positon_3->value();

  jointPositionCommandPublisher_.publish(joint_position_command);

  eStopPublisher_.publish(e_stop_msg);

}

void rqt_control_panel_plugin_widget::on_stopJointVelocityButton_clicked()
{
  std_msgs::Bool e_stop_msg;
  e_stop_msg.data = true;
  eStopPublisher_.publish(e_stop_msg);
}

void rqt_control_panel_plugin_widget::on_stopPositionButton_clicked()
{
  std_msgs::Bool e_stop_msg;
  e_stop_msg.data = true;
  eStopPublisher_.publish(e_stop_msg);
}

void rqt_control_panel_plugin_widget::on_stopJointEffortButton_clicked()
{
  std_msgs::Bool e_stop_msg;
  e_stop_msg.data = true;
  eStopPublisher_.publish(e_stop_msg);
}

void rqt_control_panel_plugin_widget::on_setJointVelocityButton_clicked()
{
  std_msgs::Bool e_stop_msg;
  e_stop_msg.data = false;

  std_msgs::Float64MultiArray joint_velocity_command;
  joint_velocity_command.data.resize(12);

  joint_velocity_command.data[0] = ui->lf_joint_velocity_1->value();
  joint_velocity_command.data[1] = ui->lf_joint_velocity_2->value();
  joint_velocity_command.data[2] = ui->lf_joint_velocity_3->value();

  joint_velocity_command.data[3] = ui->rf_joint_velocity_1->value();
  joint_velocity_command.data[4] = ui->rf_joint_velocity_2->value();
  joint_velocity_command.data[5] = ui->rf_joint_velocity_3->value();

  joint_velocity_command.data[6] = ui->rh_joint_velocity_1->value();
  joint_velocity_command.data[7] = ui->rh_joint_velocity_2->value();
  joint_velocity_command.data[8] = ui->rh_joint_velocity_3->value();

  joint_velocity_command.data[9] = ui->lh_joint_velocity_1->value();
  joint_velocity_command.data[10] = ui->lh_joint_velocity_2->value();
  joint_velocity_command.data[11] = ui->lh_joint_velocity_3->value();

  jointVelocityCommandPublisher_.publish(joint_velocity_command);

  eStopPublisher_.publish(e_stop_msg);

}

void rqt_control_panel_plugin_widget::on_setJointEffortButton_clicked()
{
  std_msgs::Bool e_stop_msg;
  e_stop_msg.data = false;

  std_msgs::Float64MultiArray joint_effort_command;
  joint_effort_command.data.resize(12);

  joint_effort_command.data[0] = ui->lf_joint_effort_1->value();
  joint_effort_command.data[1] = ui->lf_joint_effort_2->value();
  joint_effort_command.data[2] = ui->lf_joint_effort_3->value();

  joint_effort_command.data[3] = ui->rf_joint_effort_1->value();
  joint_effort_command.data[4] = ui->rf_joint_effort_2->value();
  joint_effort_command.data[5] = ui->rf_joint_effort_3->value();

  joint_effort_command.data[6] = ui->rh_joint_effort_1->value();
  joint_effort_command.data[7] = ui->rh_joint_effort_2->value();
  joint_effort_command.data[8] = ui->rh_joint_effort_3->value();

  joint_effort_command.data[9] = ui->lh_joint_effort_1->value();
  joint_effort_command.data[10] = ui->lh_joint_effort_2->value();
  joint_effort_command.data[11] = ui->lh_joint_effort_3->value();

  jointEffortCommandPublisher_.publish(joint_effort_command);

  eStopPublisher_.publish(e_stop_msg);
}

void rqt_control_panel_plugin_widget::on_setVelocityBotton_clicked()
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = ui->desired_vx->value();
  cmd_vel.linear.y = ui->desired_vy->value();
  cmd_vel.angular.z = ui->desired_wz->value();

  baseVelocityCommandPublisher_.publish(cmd_vel);

}


void rqt_control_panel_plugin_widget::on_startSingleLegContoller_clicked()
{
  std_msgs::Bool e_stop_msg;
  e_stop_msg.data = false;

  eStopPublisher_.publish(e_stop_msg);
}

void rqt_control_panel_plugin_widget::on_stopSingleLegController_clicked()
{
  std_msgs::Bool e_stop_msg;
  e_stop_msg.data = true;

  eStopPublisher_.publish(e_stop_msg);
}

void rqt_control_panel_plugin_widget::on_resetJointPostionButton_clicked()
{
  ui->lf_joint_positon_1->setValue(0);
  ui->lf_joint_positon_2->setValue(1.2);
  ui->lf_joint_positon_3->setValue(-2.4);

  ui->rf_joint_positon_1->setValue(0);
  ui->rf_joint_positon_2->setValue(-1.2);
  ui->rf_joint_positon_3->setValue(2.4);

  ui->rh_joint_positon_1->setValue(0);
  ui->rh_joint_positon_2->setValue(1.2);
  ui->rh_joint_positon_3->setValue(-2.4);

  ui->lh_joint_positon_1->setValue(0);
  ui->lh_joint_positon_2->setValue(-1.2);
  ui->lh_joint_positon_3->setValue(2.4);


}

void rqt_control_panel_plugin_widget::on_resetJointVelocityButton_clicked()
{
    ui->lf_joint_velocity_1->setValue(0);
    ui->lf_joint_velocity_2->setValue(0);
    ui->lf_joint_velocity_3->setValue(0);

    ui->rf_joint_velocity_1->setValue(0);
    ui->rf_joint_velocity_2->setValue(0);
    ui->rf_joint_velocity_3->setValue(0);

    ui->rh_joint_velocity_1->setValue(0);
    ui->rh_joint_velocity_2->setValue(0);
    ui->rh_joint_velocity_3->setValue(0);

    ui->lh_joint_velocity_1->setValue(0);
    ui->lh_joint_velocity_2->setValue(0);
    ui->lh_joint_velocity_3->setValue(0);
}

void rqt_control_panel_plugin_widget::on_resetJointEffortButton_clicked()
{
  ui->lf_joint_effort_1->setValue(0);
  ui->lf_joint_effort_2->setValue(0);
  ui->lf_joint_effort_3->setValue(0);

  ui->rf_joint_effort_1->setValue(0);
  ui->rf_joint_effort_2->setValue(0);
  ui->rf_joint_effort_3->setValue(0);

  ui->rh_joint_effort_1->setValue(0);
  ui->rh_joint_effort_2->setValue(0);
  ui->rh_joint_effort_3->setValue(0);

  ui->lh_joint_effort_1->setValue(0);
  ui->lh_joint_effort_2->setValue(0);
  ui->lh_joint_effort_3->setValue(0);
}
