#include <ros/ros.h>
#include <iostream>
#include <legodom.h>

using namespace std;
using namespace quadruped_odom;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "legodom_test");
    ros::NodeHandle nh_("w");

    std::shared_ptr<free_gait::State> robot_state;
    std::vector<free_gait::LimbEnum> limbs;
    std::vector<free_gait::BranchEnum> branches_;
    limbs.push_back(free_gait::LimbEnum::LF_LEG);
    limbs.push_back(free_gait::LimbEnum::RF_LEG);
    limbs.push_back(free_gait::LimbEnum::RH_LEG);
    limbs.push_back(free_gait::LimbEnum::LH_LEG);
    branches_.push_back(free_gait::BranchEnum::BASE);
    branches_.push_back(free_gait::BranchEnum::LF_LEG);
    branches_.push_back(free_gait::BranchEnum::RF_LEG);
    branches_.push_back(free_gait::BranchEnum::LH_LEG);
    branches_.push_back(free_gait::BranchEnum::RH_LEG);

    robot_state.reset(new free_gait::State);
    robot_state->initialize(limbs, branches_);

    QuadrupedEstimation legodom_test(nh_, robot_state);


    //sensor
    sensor_msgs::Imu imu_data;
    imu_data.orientation.w = 1;
    imu_data.orientation.x = 0;
    imu_data.orientation.y = 0;
    imu_data.orientation.z = 0;
//    Eigen::Quaterniond imu_q;
//    imu_q << imu_data.orientation;


    std_msgs::Float64MultiArray foot_out;
    foot_out.data.resize(4);
    foot_out.data[0] = 0;
    foot_out.data[1] = 1;
    foot_out.data[2] = 0;
    foot_out.data[3] = 1;
    free_gait::JointPositions joints_all, jointsvel;
    joints_all  <<  0,0,0,
                    0,0,0,
                    0,0,0,
                    0,0,0,
//    joints_all  <<  -0.5,0.86,1.1,
//                    -0.5,-0.86,1.7,
//                    -0.5,-0.86,1.7,
//                    -0.5,0.86,1.1;
//    joints_all  << -0.52, 0.87, 4.56, -0.52, -0.87, 1.7, -0.52, 0.87, 4.56, -0.52, -0.87, 1.7;
    jointsvel << 0,0,1,
                 0,0,1,
                 0,0,1,
                 0,0,1;
    robot_state->setCurrentLimbJoints(joints_all);
    robot_state->setCurrentLimbJointVelocities(jointsvel);

    nav_msgs::Odometry gazebo_output ;
    gazebo_output.pose.pose.position.x = 1;
    gazebo_output.pose.pose.position.y = 2;
    gazebo_output.pose.pose.position.z = 3;
    gazebo_output.pose.pose.orientation.w = 1;
    gazebo_output.pose.pose.orientation.x = 0.1;
    gazebo_output.pose.pose.orientation.y = 0.1;
    gazebo_output.pose.pose.orientation.z = 0.1;

    legodom_test.ProcessSensorData(nh_,imu_data,foot_out,joints_all,gazebo_output);

//    legodom_test.TFINIT(gazebo_output,imu_data,joints_all);//check

//    legodom_test.FootstateJudge(foot_out);//check

//    legodom_test.GetFootPoseInBase();//check
//    legodom_test.GetFootPoseInBase(joints_all);//check


//    legodom_test.QuadrupedPosition();

//    legodom_test.GetPositionAddEveryStep();



//    legodom_test.QuadrupedVel();//check
//    legodom_test.GetLinearVelFromKin();//check
//    legodom_test.GetLinearVelFromJointvel();//check

//      legodom_test.QuadrupedOrientation();
//    legodom_test.GetQFromImu(imu_data);//check
//    legodom_test.GetQFromKin();//check


//    legodom_test.GetStopStateOdom();//check


    ROS_INFO("test finish");

    return 0;
}

