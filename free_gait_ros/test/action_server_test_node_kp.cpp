#include "free_gait_core/free_gait_core.hpp"
#include "free_gait_ros/free_gait_ros.hpp"
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

class ActionServerTestkp
{
public:
    ActionServerTestkp(ros::NodeHandle& nodehandle):
        nodehandle_(nodehandle),
        adapter_loader("free_gait_ros", "free_gait::adapterBase"),
        use_gazebo(true),
        is_pause(false),
        is_stop(false),
        is_kinematics_control(true),
        is_start_gait(false),
        AdapterRos_(nodehandle, free_gait::AdapterRos::AdapterType::Gazebo),
        gait_generate_client_(nodehandle)
    {
        nodehandle_.getParam("/use_gazebo", use_gazebo);
        nodehandle_.getParam("/kinematics_control", is_kinematics_control);
        nodehandle_.getParam("/free_gait/stop_execution_service", stop_service_name_);
        nodehandle_.getParam("/free_gait/pause_execution_service", pause_service_name_);

        if(use_gazebo)
        {
            adapter.reset(AdapterRos_.getAdapterPtr());
            AdapterRos_.subscribeToRobotState();
        } else {
            adapter.reset(adapter_loader.createUnmanagedInstance("free_gait_ros/AdapterDummy"));
        }
        state.reset(new State());
        parameters.reset(new StepParameters());
        completer.reset(new StepCompleter(*parameters, *adapter));
        computer.reset(new StepComputer());

        AdapterRos_.updateAdapterWithState();

        state->setPositionWorldToBaseInWorldFrame(AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame());
        state->setOrientationBaseToWorld(AdapterRos_.getAdapter().getOrientationBaseToWorld());
        state->setLinearVelocityBaseInWorldFrame(AdapterRos_.getAdapter().getLinearVelocityBaseInWorldFrame());
        state->setAngularVelocityBaseInBaseFrame(AdapterRos_.getAdapter().getAngularVelocityBaseInBaseFrame());
        std::cout << *state << std::endl;

        executor.reset(new Executor(*completer, *computer, *adapter, *state));
        rospublisher.reset(new StateRosPublisher(nodehandle_, *adapter));

        if(is_kinematics_control)
        {
            rospublisher->setTfPrefix("/ideal");
        }else if (!is_kinematics_control){
            rospublisher->setTfPrefix("/desired");
        }

        executor->initialize();

        AdapterRos_.updateAdapterWithState();

        pause_service_server_ = nodehandle_.advertiseService(pause_service_name_, &ActionServerTestkp::PauseServiceCallback, this);
        stop_service_server_ = nodehandle_.advertiseService(stop_service_name_, &ActionServerTestkp::StopServiceCallback, this);
        gait_start_server_ = nodehandle_.advertiseService("/gait_generate_switch", &ActionServerTestkp::GaitGenerateSwitchCallback, this);
        pace_start_server_ = nodehandle_.advertiseService("/pace_switch", &ActionServerTestkp::PaceSwitchCallback, this);

        limb_configure_switch_server_ = nodehandle_.advertiseService("/limb_configure", &ActionServerTestkp::SwitchLimbConfigureCallback, this);

        joint_state_pub_ = nodehandle_.advertise<sensor_msgs::JointState>("/all_joint_position", 1);
        allJointStates_.position.resize(12);

        server_.reset(new FreeGaitActionServer(nodehandle_, "/free_gait/action_server", *executor, *adapter));
        server_->initialize();
        server_->start();

        action_server_thread_ = boost::thread(boost::bind(&ActionServerTestkp::ActionServerThread, this));
        gait_generate_thread = boost::thread(boost::bind(&ActionServerTestkp::gait_generate_thread, this));

    }

    ~ActionServerTestkp(){}

    void ActionServerThread()
    {
        ROS_INFO("In action server thread");
        parameters->footstepParameters.minimumDuration_ = 0.45;
        parameters->baseTargetParameters.minimumDuration = 0.45;
        double dt = 0.01;
        double time = 0.0;
        ros::Rate rate(100);
        ros::Duration(0.5).sleep();
        cout << "current base position : " << AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame() << endl;
        AdapterRos_.updateAdapterWithState();
        executor->reset();
        cout << adapter->getState() << endl;
        cout << "current base position : " << AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame() << endl;

        state->setPositionWorldToBaseInWorldFrame(AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame());
        state->setOrientationBaseToWorld(AdapterRos_.getAdapter().getOrientationBaseToWorld());
        state->setLinearVelocityBaseInWorldFrame(AdapterRos_.getAdapter().getLinearVelocityBaseInWorldFrame());
        state->setAngularVelocityBaseInBaseFrame(AdapterRos_.getAdapter().getAngularVelocityBaseInBaseFrame());

        for (int i = 0; i < 12; i++) {
            allJointStates_.position[i] = adapter->getState().getJointPositionFeedback()(i);
        }

        if(is_kinematics_control)
            joint_state_pub_.publish(allJointStates_);
        cout << AdapterRos_.getAdapter().getState() << endl;
        rospublisher->publish(AdapterRos_.getAdapter().getState());

        while (ros::ok()) {
            server_->update();
            if (!executor->getQueue().empty() && !is_pause && !is_stop){
                boost::recursive_mutex::scoped_lock lock(r_mutex_);
                AdapterRos_.updateAdapterWithState();

                executor->advance(dt, false);

                if(is_start_gait){
                    state->setLinearVelocityBaseInWorldFrame(desired_linear_velocity_);
                    state->setAngularVelocityBaseInBaseFrame(desired_angular_velocity_);
                }
                time = time + dt;
                for (unsigned int i = 0; i < 12; i++) {
                    allJointStates_.position[i] = adapter->getState().getJointPositions()(i);
                }
                if(is_kinematics_control){
                    joint_state_pub_.publish(allJointStates_);
                }
                if(!use_gazebo)
                {
                    rospublisher->publish(adapter->getState());
                }
                if(!is_kinematics_control)
                {
                    rospublisher->publish(adapter->getState(),executor->getQueue());
                }
                lock.unlock();
            }else {
                cout << "nothing out put" << endl;
                    }
            rate.sleep();
        }
    }

    void Gaitgeneratethread()
    {
        ROS_INFO("the gaitgeneratethread thread");
        ros::Rate rate(100);
        double dt = 0.01;
        gait_generate_client_.initializeTrot(0.45, 0.45);
        while (ros::ok()) {
            if(is_start_gait){
                boost::recursive_mutex::scoped_lock lock(r_mutex_);
                AdapterRos_.updateAdapterWithState();
                gait_generate_client_.copyRobotState(AdapterRos_.getAdapter().getState());
                gait_generate_client_.advance(dt);
                gait_generate_client_.generateFootHolds("foot_print");
                gait_generate_client_.updateBaseMotion(desired_linear_velocity_,desired_angular_velocity_);
                lock.unlock();
                gait_generate_client_.sendMotionGoal();
            }
            rate.sleep();
        }
    }
    bool StopServiceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
    {
        ROS_INFO("STOP Triggher");
        server_->setPreempted();
        is_stop = true;
        response.success = true;
        return  true;
    }

    bool PauseServiceCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
    {
        if(request.data == false)
        {
            is_pause = false;
            is_stop = false;
            ROS_INFO("Start...");
        }
        if(request.data == true){
            is_pause = true;
            ROS_INFO("Stop...");
        }
        response.success = true;
        return true;
    }

    bool GaitGenerateSwitchCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
    {
        if(request.data ==false)
        {
            is_start_gait = false;
            ROS_INFO("Stop gait...");
        }
        if(request.data == true)
        {
            is_start_gait = true;
            gait_generate_client_.initializeTrot(0.45, 0.45);
            ROS_INFO("Start gait...");
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
    std::unique_ptr<FreeGaitActionServer> server_;
    boost::thread action_server_thread_, gait_generate_thread;
    ros::NodeHandle nodehandle_;

    free_gait::AdapterRos AdapterRos_;

    std::unique_ptr<AdapterBase> adapter;
    pluginlib::ClassLoader<AdapterBase> adapter_loader;

    std::unique_ptr<State> state;
    std::unique_ptr<StepParameters> parameters;
    std::unique_ptr<StepCompleter> completer;
    std::unique_ptr<StepComputer> computer;
    std::unique_ptr<Executor> executor;
    std::unique_ptr<StateRosPublisher> rospublisher;

    GaitGenerateClient gait_generate_client_;

    sensor_msgs::JointState allJointStates_;
    ros::Publisher joint_state_pub_;
    ros::ServiceServer pause_service_server_, stop_service_server_,
        gait_start_server_, limb_configure_switch_server_, pace_start_server_;
    std::string pause_service_name_, stop_service_name_;
    bool use_gazebo, is_pause, is_stop, is_kinematics_control, is_start_gait;

    boost::recursive_mutex r_mutex_;
    LinearVelocity desired_linear_velocity_;
    LocalAngularVelocity desired_angular_velocity_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "action_server_test");
    ros::NodeHandle nh("~");
    ActionServerTestkp ac_test(nh);

    ros::spin();
    return 0;
}

