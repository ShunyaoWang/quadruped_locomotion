#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/free_gait_core.hpp"
#include "free_gait_core/leg_motion/Footstep.hpp"
#include "free_gait_msgs/Footstep.h"
#include "free_gait_ros/free_gait_ros.hpp"
#include "pluginlib/class_loader.h"
#include "free_gait_core/executor/AdapterBase.hpp"
#include "ros/node_handle.h"

using namespace free_gait;
using namespace std;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "step_test_kp");
    ros::NodeHandle nh("~");
    cout << "node 1" << endl;
    AdapterRos adaptorRos_(nh, free_gait::AdapterRos::AdapterType::Preview);
    cout << "node 2" << endl;
    pluginlib::ClassLoader<AdapterBase> adapter_loader("free_gait_ros", "free_gait::AdapterBase");
    std::unique_ptr<AdapterBase> adapter;
    adapter.reset(adapter_loader.createUnmanagedInstance("free_gait_ros/AdapterDummy"));//adapter use the adapterdummy

    JointPositionsLeg joints(0, 1, -3);
    Position end = adapter->getPositionBaseToFootInBaseFrame(LimbEnum::LF_LEG, joints);//why this is
//    string basefreamedid_ = adapter->getBaseFrameId();

//    JointPositionsLeg legjoints = adapter->getJointPositionsForLimb(LimbEnum::LF_LEG);
//    JointPositionsLeg legjoints_2 = adapter->getJointPositionsForLimb(LimbEnum::LH_LEG);
//    JointPositionsLeg legjoints_3 = adapter->getJointPositionsForLimb(LimbEnum::RF_LEG);
//    JointPositionsLeg legjoints_4 = adapter->getJointPositionsForLimb(LimbEnum::RH_LEG);

//    std::cout << "the leg joints is" << legjoints <<endl <<legjoints_2 << endl << legjoints_3 << endl
//        << legjoints_4 << std::endl;

//    std::cout << "the limbs are " << adapter->getLimbs()[0] << adapter->getLimbs()[1]
//              << adapter->getLimbs()[2] << std::endl;

//    std::cout << "the end is " << end << std::endl;
//    adapter->getLimbJointPositionsFromPositionBaseToFootInBaseFrame(end, LimbEnum::LF_LEG, joints);
//    std::cout << "the joint is " << joints << std::endl;

    State state;
    StepParameters parameters;
    StepCompleter completer(parameters, *adapter);
    StepComputer computer;
    Executor executor(completer, computer, *adapter, state);
    BatchExecutor batch_executor(executor);
    executor.initialize();
    StepRosConverter converter(*adapter);
    StateRosPublisher rosPublisher(nh, *adapter);

    std::vector<Step> steps;
    Step step;
    step.setId("00");
    free_gait_msgs::BaseAuto baseauto_msg;
    baseauto_msg.height = 0.4;
    baseauto_msg.average_linear_velocity = 0.0;
    baseauto_msg.average_angular_velocity = 0.0;
    baseauto_msg.support_margin = 0.0;
    baseauto_msg.ignore_timing_of_leg_motion = false;
    BaseAuto baseauto;
    converter.fromMessage(baseauto_msg, baseauto);
    step.addBaseMotion(baseauto);
    steps.push_back(step);

    std::cout << "step 00 build" << std::endl;

    Step step1;
    step1.setId("01");
    free_gait_msgs::BaseTarget base_target_msg;
    geometry_msgs::PointStamped target;
    geometry_msgs::Vector3Stamped surface_normal;
    base_target_msg.target.header.frame_id = "odom";
    base_target_msg.target.pose.position.x = 0.1;
    base_target_msg.target.pose.position.y = 0;
    base_target_msg.target.pose.position.z = 0.4;
    base_target_msg.target.pose.orientation.w = 1;
    base_target_msg.target.pose.orientation.x = 0;
    base_target_msg.target.pose.orientation.y = 0;
    base_target_msg.target.pose.orientation.z = 0;
    base_target_msg.average_angular_velocity = 0.1;
    base_target_msg.average_linear_velocity = 0.1;
    base_target_msg.ignore_timing_of_leg_motion = false;
    BaseTarget baseTarget;
    converter.fromMessage(base_target_msg, baseTarget);
    step1.addBaseMotion(baseTarget);
    steps.push_back(step1);

    std::cout << "step 01 build" << std::endl;

    Step step2;
    step2.setId("02");
    free_gait_msgs::Footstep footstep_msg;
    footstep_msg.target.point.x = 0.5;
    footstep_msg.target.point.y = 0.25;
    footstep_msg.target.point.z = 0.0;

    footstep_msg.target.header.frame_id = "odom";
    footstep_msg.surface_normal.vector.x = 0.0;
    footstep_msg.surface_normal.vector.y = 0.0;
    footstep_msg.surface_normal.vector.z = 1.0;

    footstep_msg.name = "LF_LEG";
    footstep_msg.average_velocity = 0.15;
    footstep_msg.profile_height = 0.15;
    footstep_msg.profile_type = "triangle";
    footstep_msg.ignore_contact = false;//???
    footstep_msg.ignore_for_pose_adaptation = false;//???

    Footstep footstep(LimbEnum::LF_LEG);
    converter.fromMessage(footstep_msg, footstep);
    step2.addLegMotion(footstep);

    baseauto_msg.height = 0.4;
    baseauto_msg.average_angular_velocity = 0.0;
    baseauto_msg.average_linear_velocity = 0.0;
    baseauto_msg.support_margin = 0.0;
    baseauto_msg.ignore_timing_of_leg_motion = false;
    converter.fromMessage(baseauto_msg, baseauto);
    step2.addBaseMotion(baseauto);

    steps.push_back(step2);

    std::cout << "step 02 build" << std::endl;

    Step step3;
    step3.setId("03");
    target.point.x = -0.2;
    target.point.y = -0.25;
    target.point.z = 0.0;
    target.header.frame_id = "odom";
    surface_normal.vector.x = 0.0;
    surface_normal.vector.y = 0.0;
    surface_normal.vector.z = 1.0;
    //  footstep.updateStartPosition(start);
    footstep_msg.name = "RH_LEG";
    footstep_msg.target = target;
    footstep_msg.average_velocity = 0.15;
    footstep_msg.profile_height = 0.15;
    footstep_msg.profile_type = "triangle";
    footstep_msg.ignore_contact = false;
    footstep_msg.ignore_for_pose_adaptation = false;
    footstep_msg.surface_normal = surface_normal;
    Footstep footstep2(LimbEnum::RH_LEG);
    converter.fromMessage(footstep_msg, footstep2);
    step3.addLegMotion(footstep2);
    // Basemotion
    //  free_gait_msgs::BaseAuto baseauto_msg;
    baseauto_msg.height = 0.4;
    baseauto_msg.average_angular_velocity = 0.0;
    baseauto_msg.average_linear_velocity = 0.0;
    baseauto_msg.support_margin = 0.0;
    baseauto_msg.ignore_timing_of_leg_motion = false;
    //  BaseAuto baseauto;
    converter.fromMessage(baseauto_msg, baseauto);
    step3.addBaseMotion(baseauto);
    steps.push_back(step3);

    std::cout << "step 03 build" << std::endl;

    free_gait_msgs::BaseTrajectory base_trajectory_msg;

    BaseTrajectory baseTrajectory;
    converter.fromMessage(base_trajectory_msg, baseTrajectory);

    executor.getQueue().add(steps);
    executor.setPreemptionType(Executor::PreemptionType::PREEMPT_IMMEDIATE);
    double dt = 0.001;
    double time = 0.0;
    ros::Time t_start = ros::Time::now();
    ros::Rate rate(100);

    std::cout << "executor is ready" << std::endl;

    std::cout << executor.getQueue().size() << std::endl;

    while (!executor.getQueue().empty() && ros::ok()) {
        std::cout << "what " << std::endl;
        executor.advance(dt, true);
        time = time + dt;
        std::cout << "time is " << time << std::endl;
        rosPublisher.publish(adapter->getState());
        rate.sleep();
    }

    ros::Time t_end = ros::Time::now();
    std::cout << "end time is " << time << std::endl;
    std::cout << " Real time cost is " << t_end - t_start << std::endl;
    return 0;
}
