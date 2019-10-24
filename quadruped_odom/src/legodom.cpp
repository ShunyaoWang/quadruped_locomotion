#include "legodom.h"

namespace quadruped_odom {

QuadrupedEstimation::QuadrupedEstimation(const ros::NodeHandle& _nodehandle,
                                         std::shared_ptr<free_gait::State> robot_state)
    : nodeHandle_(_nodehandle),
      robot_state_(robot_state)
{

    //everytime -- everystep -- frominit-- everytime4foot
    nodeHandle_.param("cal_position_way", _cal_position_way, std::string("everytime4foot"));
    //dpdtvel -- jacobvel
    nodeHandle_.param("cal_vel_way", _cal_vel_way, std::string("jacobvel"));
    nodeHandle_.param("orientation_way", _orientation_way, std::string("imu_way"));
    nodeHandle_.param("vel_set_T", _vel_set_T,  double(10.0) );
    nodeHandle_.param("odom_dt", _odom_dt, double(0.02) );
    nodeHandle_.param("/cal_fator_x", _cal_fator_x, double(1.2));
    nodeHandle_.param("/cal_fator_y", _cal_fator_y, double(1.2));
    nodeHandle_.param("/cal_fator_z", _cal_fator_z, double(0.9));
    nodeHandle_.param("/real_time_factor", real_time_factor, double(0.2));
    nodeHandle_.param("/legodom/use_gazebo_feedback", gazebo_flag, bool(true));
    nodeHandle_.param("/imu_topic_name", imu_topic_name_, std::string("/imu"));
    //Initialization1
    InitParam();
    init_time = ros::Time::now();
    //sub topic
    imu_sub = nodeHandle_.subscribe<sensor_msgs::Imu>(imu_topic_name_, 10, &QuadrupedEstimation::imuCb, this);
//    foot_state_sub = nodeHandle_.subscribe<std_msgs::Float64MultiArray>("/gazebo/foot_contact_state", 10, &QuadrupedEstimation::footstateCb, this);

    //    joints_sub = nodeHandle_.subscribe<sensor_msgs::JointState>("/joint_states", 10, &QuadrupedEstimation::jointsCb, this);
//    gazebo_sub = nodeHandle_.subscribe("/gazebo/odom", 10, &QuadrupedEstimation::gazeboCb, this);
    if(gazebo_flag)
      {
        modelStatesSub_ = nodeHandle_.subscribe("/gazebo/model_states", 2, &QuadrupedEstimation::modelStatesCallback, this);
        gazebo_pub = nodeHandle_.advertise<nav_msgs::Odometry>("/gazebo/odom",10);
      }

    //pub topic
    legodom_odom_pub = nodeHandle_.advertise<nav_msgs::Odometry>("/legodom", 10);
    legodom_map_pub = nodeHandle_.advertise<nav_msgs::Odometry>("/legodom_map", 10);
    legodom_error_pub = nodeHandle_.advertise<nav_msgs::Odometry>("/legodom_error", 10);
    legodom_init_pub = nodeHandle_.advertise<nav_msgs::Odometry>("/legodom_init", 10);
    legPose_pub = nodeHandle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/legodom/base_pose", 10);

    imuvel_pub = nodeHandle_.advertise<geometry_msgs::Twist>("/imu_vel",10);


}

QuadrupedEstimation::QuadrupedEstimation(const ros::NodeHandle& _nodehandle)
    :nodeHandle_(_nodehandle)
{
    nodeHandle_.param("cal_position_way", _cal_position_way, std::string("everyTime"));
    nodeHandle_.param("cal_vel_way", _cal_vel_way, std::string("dpdtvel"));
    nodeHandle_.param("orientation_way", _orientation_way, std::string("imu_way"));
    nodeHandle_.param("vel_set_T", _vel_set_T,  double(10.0) );
    nodeHandle_.param("odom_dt", _odom_dt, double(0.02) );
    nodeHandle_.param("cal_fator_x", _cal_fator_x, double(1.25));
    nodeHandle_.param("cal_fator_y", _cal_fator_y, double(1.25));
    nodeHandle_.param("cal_fator_z", _cal_fator_z, double(0.9));
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

    robot_state_.reset(new free_gait::State);
    robot_state_->initialize(limbs, branches_);

    //Initialization
    InitParam();

    //sub topic
    imu_sub = nodeHandle_.subscribe<sensor_msgs::Imu>("/imu", 100, &QuadrupedEstimation::imuCb, this);
    foot_state_sub = nodeHandle_.subscribe<std_msgs::Float64MultiArray>("/gazebo/foot_contact_state", 10, &QuadrupedEstimation::footstateCb, this);
    joints_sub = nodeHandle_.subscribe<sensor_msgs::JointState>("/joint_states", 10, &QuadrupedEstimation::jointsCb, this);
    gazebo_sub = nodeHandle_.subscribe("/gazebo/odom", 10, &QuadrupedEstimation::gazeboCb, this);
    //joints(position,velocity,effort)
    //footContacts(support_leg,surface_normal)
    //gazebo-pose+twist + tf(odom--->basefootprint,basefootprint--->base_link)
//    robot_state_sub = nodeHandle_.subscribe<free_gait_msgs::RobotState>("/gazebo/robot_states", 10, &QuadrupedEstimation::robotstateCb, this);

    //pub topic
    legodom_odom_pub = nodeHandle_.advertise<nav_msgs::Odometry>("/legodom", 10);
    legodom_map_pub = nodeHandle_.advertise<nav_msgs::Odometry>("/legodom_map", 10);
    legodom_error_pub = nodeHandle_.advertise<nav_msgs::Odometry>("/legodom_error", 10);
    legodom_init_pub = nodeHandle_.advertise<nav_msgs::Odometry>("/legodom_init", 10);


}

QuadrupedEstimation::~QuadrupedEstimation(){

}
void QuadrupedEstimation::RobotStateLoad(){


}

void QuadrupedEstimation::imuCb(const sensor_msgs::Imu::ConstPtr& imu_msg){

//    ROS_WARN("get imu_msg");
    imu_output = *imu_msg;
    imu_cb_flag = true;

}

void QuadrupedEstimation::jointsCb(const sensor_msgs::JointState::ConstPtr& joint_msg){

//    for(int i=0; i< 16; ++i){
//        joint_out.data[i] = joint_msg->position[i];
//    }
//    for(int j=0; j< 16; ++j){
//        joint_out.data[j+16] = joint_msg->velocity[j];
//    }

//    ROS_WARN("get joints_msg");
//    joints_output = *joint_msg;
//    joints_cb_flag = true;

//    for(int i=0;i< 12;++i){
//        joints_all(i) = joints_output.position[i];
//        jointsvel_all(i)=  joints_output.velocity[i];
//    }
//    cout << "joints_all: "<< endl << joints_all <<
//            "jointsvel_all"<< endl << jointsvel_all << endl;

//    robot_state_->setCurrentLimbJoints(joints_all);
//    robot_state_->setCurrentLimbJointVelocities(jointsvel_all);

}
void QuadrupedEstimation::setFootState(const std_msgs::Float64MultiArray& foot_msg){

//    ROS_WARN("get foot_msg");
    foot_output = foot_msg;
    foot_cb_flag = true;
}

void QuadrupedEstimation::footstateCb(const std_msgs::Float64MultiArray::ConstPtr& foot_msg){

//    ROS_WARN("get foot_msg");
    foot_output = *foot_msg;
    foot_cb_flag = true;
}

void QuadrupedEstimation::gazeboCb(const nav_msgs::Odometry::ConstPtr& gazebo_msg){

//    ROS_WARN("get gazebo_msg");
    gazebo_output = *gazebo_msg;



}

void QuadrupedEstimation::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &modelStatesMsg){
//    ROS_INFO("Recieved a model states");
    geometry_msgs::Pose base_pose = modelStatesMsg->pose[10];
    geometry_msgs::Twist base_twist =modelStatesMsg->twist[10];
    gazebo_output.header.stamp = ros::Time::now();
    gazebo_output.pose.pose = base_pose;
    gazebo_output.twist.twist = base_twist;
    gazebo_output.header.frame_id = "/gazebo_odom";
    gazebo_output.child_frame_id = "/gazebo_baselink";
    gazebo_pub.publish(gazebo_output);


}

void QuadrupedEstimation::robotstateCb(const free_gait_msgs::RobotState::ConstPtr& robotstate_msg)
{
//    joints_all = robotstate_msg->lf_leg_joints.position[0],robotstate_msg->lf_leg_joints.position[1],robotstate_msg->lf_leg_joints.position[2],
//                 robotstate_msg->rf_leg_joints.position[0],robotstate_msg->rf_leg_joints.position[1],robotstate_msg->rf_leg_joints.position[2],
//                 robotstate_msg->lh_leg_joints.position[0],robotstate_msg->lh_leg_joints.position[1],robotstate_msg->lh_leg_joints.position[2],
//                 robotstate_msg->rh_leg_joints.position[0],robotstate_msg->rh_leg_joints.position[1],robotstate_msg->rh_leg_joints.position[2];
//    robot_state->setAllJointPositions(joints_all);

//    jointsvel_all = robotstate_msg->lf_leg_joints.velocity[0],robotstate_msg->lf_leg_joints.velocity[1],robotstate_msg->lf_leg_joints.velocity[2],
//                 robotstate_msg->rf_leg_joints.velocity[0],robotstate_msg->rf_leg_joints.velocity[1],robotstate_msg->rf_leg_joints.velocity[2],
//                 robotstate_msg->lh_leg_joints.velocity[0],robotstate_msg->lh_leg_joints.velocity[1],robotstate_msg->lh_leg_joints.velocity[2],
//                 robotstate_msg->rh_leg_joints.velocity[0],robotstate_msg->rh_leg_joints.velocity[1],robotstate_msg->rh_leg_joints.position[2];
//    robot_state->setAllJointVelocities(jointsvel_all);
    //to odom
//    gazebo_output = robotstate_msg->base_pose;

//    gazebo_out.data[0] = robotstate_msg->base_pose.pose.pose.orientation.x;
//    gazebo_out.data[1] = robotstate_msg->base_pose.pose.pose.orientation.y;
//    gazebo_out.data[2] = robotstate_msg->base_pose.pose.pose.orientation.z;
//    gazebo_out.data[3] = robotstate_msg->base_pose.pose.pose.orientation.w;
//    gazebo_out.data[4] = robotstate_msg->base_pose.pose.pose.position.x;
//    gazebo_out.data[5] = robotstate_msg->base_pose.pose.pose.position.y;
//    gazebo_out.data[6] = robotstate_msg->base_pose.pose.pose.position.z ;
//    gazebo_out.data[8] = robotstate_msg->base_pose.twist.twist.angular.x;
//    gazebo_out.data[9] = robotstate_msg->base_pose.twist.twist.angular.y;
//    gazebo_out.data[10] = robotstate_msg->base_pose.twist.twist.angular.z;
//    gazebo_out.data[11] = robotstate_msg->base_pose.twist.twist.linear.x;
//    gazebo_out.data[12] = robotstate_msg->base_pose.twist.twist.linear.y;
//    gazebo_out.data[13] = robotstate_msg->base_pose.twist.twist.linear.z;
//    gazebo_out.data[14] = robotstate_msg->base_pose.header.stamp.toSec();


}


void QuadrupedEstimation::InitParam(){
//    ROS_ERROR("............Initializing Params.............");

    //data form topic
//    imu_out.data.resize(11);
//    joint_out.data.resize(32);
//    foot_out.data.resize(8);
//    gazebo_out.data.resize(15);
//    gazebo_out.data[3] = 1;//

    odom_positioninit << 0,0,0;
    odom_vel << 0,0,0;
    odom2init_orientation.w() = 1;
    odom2init_orientation.x() = 0;
    odom2init_orientation.y() = 0;
    odom2init_orientation.z() = 0;
    imu_output.orientation.w = 1;
    imu_output.orientation.x = 0;
    imu_output.orientation.y = 0;
    imu_output.orientation.z = 0;
    vel_kin_T_ = 0;
    velfromkin_ = 0;
    leg1_T = 0;
    leg2_T = 0;
    leg3_T = 0;
    leg4_T = 0;
    init_ww =1;
    cycle_T = 0;
    foot_state_T =0;
    foot_flag =0;
    X_everystep << 0.1,0,0;
    first_foot_flag =0;
    gazebo_output.pose.pose.orientation.w =1;
    gazebo_T =0;
    odom_position_T =0;
    P_init_time =0;
    //falge
    imu_cb_flag = false;
    joints_cb_flag = false;
    foot_cb_flag = false;
    position_joints_flag = false;
    tfcal_flag = true;
    topic_flag = true;
    P_init_flag =true;
    no_data_input_flag = true;
    first_check_p = true;
    velget_flag = true;
    gazebo_flag = false;

    //resize
    T_DH.resize(4);
    P_DH.resize(4);
    P_BW.resize(4);
    YPRfromQ.reserve(4);
    rotation_matrix_DH.resize(4);
    R_Q.resize(4);
    Q_kindr.resize(4);
    V_Jacob.resize(4);
    joint_vel_.resize(4);
    legodom_error_cal.data.resize(6);
    foot_output.data.resize(4);
    jointsleg_.resize(4);
    tmp_init_data.resize(7);

//    joints_all << 0,1.57,-3.14,0,1.57,-3.14,0,1.57,-3.14,0,1.57,-3.14;
    joints_output.position.resize(12);
    joints_output.position.at(0) = 0;
    joints_output.position.at(1) = 1.57;
    joints_output.position.at(2) = -3.14;
    joints_output.position.at(3) = 0;
    joints_output.position.at(4) = 1.57;
    joints_output.position.at(5) = -3.14;
    joints_output.position.at(6) = 0;
    joints_output.position.at(7) = 1.57;
    joints_output.position.at(8) = -3.14;
    joints_output.position.at(9) = 0;
    joints_output.position.at(10) = 1.57;
    joints_output.position.at(11) = -3.14;

    joints_output.velocity.resize(12);
//    for(int i=0;i<12;++i){
//        jointsvel_all(i) = 0;
//        joints_output.velocity.at(i) = 0;
//    }
}

void QuadrupedEstimation::InitParam(const ros::NodeHandle& nh) {
//    ROS_WARN("Initializing Params........");

//    nh.param("cal_position_way", _cal_position_way, std::string("everyTime"));
//    nh.param("cal_vel_way", _cal_vel_way, std::string("dpdtvel"));
//    nh.param("orientation_way", _orientation_way, std::string("imu_way"));
//    nh.param("vel_set_T", _vel_set_T,  double(10.0) );
//    nh.param("odom_dt", _odom_dt, double(0.02) );

    //data form topic
//    imu_out.data.resize(11);
//    joint_out.data.resize(32);
//    foot_out.data.resize(8);
//    gazebo_out.data.resize(15);
//    gazebo_out.data[3] = 1;//
    odom_positioninit << 0,0,0;
    odom2init_orientation.w() = 1;
    odom2init_orientation.x() = 0;
    odom2init_orientation.y() = 0;
    odom2init_orientation.z() = 0;
    imu_output.orientation.w = 1;
    imu_output.orientation.x = 0;
    imu_output.orientation.y = 0;
    imu_output.orientation.z = 0;
    gazebo_output.pose.pose.orientation.w =1;
    gazebo_T =0;
    odom_position_T =0;
    tmp_init_data[3] =1;
    vel_kin_T_ = 0;
    velfromkin_ = 0;
    leg1_T = 0;
    leg2_T = 0;
    leg3_T = 0;
    leg4_T = 0;
    P_init_time =0;
    //falge
    imu_cb_flag = false;
    joints_cb_flag = false;
    foot_cb_flag = false;
    tfcal_flag = true;
    topic_flag = true;
    first_check_p = true;
    //for position flag
    P_init_flag =true;
    no_data_input_flag = true;
    velget_flag = true;


    //resize
    T_DH.resize(4);
    P_DH.resize(4);
    P_BW.resize(4);
    YPRfromQ.reserve(4);
    rotation_matrix_DH.resize(4);
    R_Q.resize(4);
    Q_kindr.resize(4);
    V_Jacob.resize(4);
    joint_vel_.resize(4);
    legodom_error_cal.data.resize(6);
    foot_output.data.resize(4);
    jointsleg_.resize(4);
    tmp_init_data.resize(7);

    cycle_T = 0;
    foot_state_T =0;
    foot_flag =0;
    X_everystep << 0.1,0,0;
//    first_foot_flag =0;
//    joints_all << 0,1.57,-3.14,0,1.57,-3.14,0,1.57,-3.14,0,1.57,-3.14;
    joints_output.position.resize(12);
    joints_output.position.at(0) = 0;
    joints_output.position.at(1) = 1.57;
    joints_output.position.at(2) = -3.14;
    joints_output.position.at(3) = 0;
    joints_output.position.at(4) = 1.57;
    joints_output.position.at(5) = -3.14;
    joints_output.position.at(6) = 0;
    joints_output.position.at(7) = 1.57;
    joints_output.position.at(8) = -3.14;
    joints_output.position.at(9) = 0;
    joints_output.position.at(10) = 1.57;
    joints_output.position.at(11) = -3.14;

    joints_output.velocity.resize(12);
//    for(int i=0;i<12;++i){
//        jointsvel_all(i) = 0;
//        joints_output.velocity.at(i) = 0;
//    }
}


void QuadrupedEstimation::ResetParms(){
//    ROS_WARN("ResetParms...");

    init_x = gazebo_output.pose.pose.position.x;
    init_y = gazebo_output.pose.pose.position.y;
    init_z = 0;
    init_wx = gazebo_output.pose.pose.orientation.x;
    init_wy = gazebo_output.pose.pose.orientation.y;
    init_wz = gazebo_output.pose.pose.orientation.z;
    init_ww = gazebo_output.pose.pose.orientation.w;
    odom_position(0) =0;
    odom_position(1) =0;
    odom_position(2) =0;
    odom_orientation.x() =0;
    odom_orientation.y() =0;
    odom_orientation.z() =0;
    odom_orientation.w() =1;
    odom_vel(0) =0;
    odom_vel(1) =0;
    odom_vel(2) =0;
    odom_position_tmp << 0,0,0;
    InitParam();
//    gazebo_output.pose.pose.orientation.w =1;
//    imu_output.orientation.x = 0;
//    imu_output.orientation.y = 0;
//    imu_output.orientation.z = 0;
//    imu_output.orientation.w = 1;
//    foot_output.data.resize(4);
//    joints_all << 0,1.57,-3.14,0,1.57,-3.14,0,1.57,-3.14,0,1.57,-3.14;
//    for(int i=0;i<12;++i){
//        jointsvel_all(i) = 0;
//    }
//    joints_output.position.resize(12);
//    joints_output.position.at(0) = 0;
//    joints_output.position.at(1) = 1.57;
//    joints_output.position.at(2) = -3.14;
//    joints_output.position.at(3) = 0;
//    joints_output.position.at(4) = 1.57;
//    joints_output.position.at(5) = -3.14;
//    joints_output.position.at(6) = 0;
//    joints_output.position.at(7) = 1.57;
//    joints_output.position.at(8) = -3.14;
//    joints_output.position.at(9) = 0;
//    joints_output.position.at(10) = 1.57;
//    joints_output.position.at(11) = -3.14;
//    if(imu_output.orientation.w == 1 && joints_output.position.data() == 0 &&(topic_flag)){
//        ROS_WARN("Resetparms-no topic msg");
//        gazebo_output.pose.pose.orientation.w =1;
//        imu_output.orientation.x = 0;
//        imu_output.orientation.y = 0;
//        imu_output.orientation.z = 0;
//        imu_output.orientation.w = 1;
//        foot_output.data.resize(4);
//        for(int i=0; i< 16; ++i){
//            joints_output.position[i] = 0.0;
//        }
//    }
//    else{

//        ROS_WARN("Resetparms-get msg form sensor");
//        topic_flag = false;

//        if( gazebo_change[0] ==0 && gazebo_change[1] == 0 && gazebo_change[2]== 0){
//            ROS_ERROR("gazebo_pose_stop_time...");
//            ++gazebo_pose_stop_time;
//            if(gazebo_pose_stop_time > 5){
//                ROS_ERROR("gazebo_pose_stop_time > 5");
//                topic_flag =true;
//                gazebo_pose_stop_time = 0;

//                gazebo_output.pose.pose.orientation.w =1;
//                imu_output.orientation.x = 0;
//                imu_output.orientation.y = 0;
//                imu_output.orientation.z = 0;
//                imu_output.orientation.w = 1;
//                foot_output.data.resize(4);

//            }
//        }else{
//            cout <<"Resetparms---true" <<endl;
//        }
//    }

    ROS_WARN("Resetparms-finish");

}

bool QuadrupedEstimation::ProcessSensorData() {
    TicToc t_data_solve;
//    ROS_ERROR("ProcessSensorData begin");

    //get odom--->world
    TFINIT();//check

    ////////////////////
    //foot judge
    FootstateJudge(foot_output);//check

//    foot_pose_ in base
    GetFootPoseInBase();//check

    // global orientation+noise
    QuadrupedOrientation();//check

    //global-position
    QuadrupedPosition();//check
    //global-position
    QuadrupedVel();//check



    //odom-output
    LegOdomOut();//check

    //tf-output
    LegTFOut();//check

    //////
//    ResetParms();

//    ROS_WARN("ProcessSensorData finish");
//    cout << "ProcessSensorData_solve cost: " << t_data_solve.toc() << " ms" << endl;
    return true;
}

bool QuadrupedEstimation::ProcessSensorData(ros::NodeHandle& _nodehandle,
                                            sensor_msgs::Imu& imu_in,
                                            std_msgs::Float64MultiArray& foot_in,
                                            free_gait::JointPositions& jointsin,
                                            nav_msgs::Odometry& gazeboin) {
    TicToc t_data_solve;

//    ROS_WARN("ProcessSensorData begin");

    //Initialization
    InitParam(_nodehandle);

    //get odom--->world
    TFINIT(gazeboin,imu_in,jointsin);

    ////////////////////
    //foot judge
    FootstateJudge(foot_in);

//    foot_pose_ in base
    GetFootPoseInBase(jointsin);

    //global-position
    QuadrupedPosition();
    //global-position
    QuadrupedVel();
    // global orientation+noise
    QuadrupedOrientation(imu_in);

    // toppic
    LegPubTopic(_nodehandle);

    //odom-output
    LegOdomOut();

    //tf-output
    LegTFOut();

    //////
//    ResetParms();
//    ROS_WARN("ProcessSensorData finish");
//    cout << "ProcessSensorData_solve cost: " << t_data_solve.toc() << " ms" << endl;

    return true;

}



void QuadrupedEstimation::TFINIT() {
//    ROS_WARN("TFINIT !!!");

//    if( gazebo_output.pose.pose.position.x == 0 && gazebo_output.pose.pose.orientation.w ==1 && gazebo_output.twist.twist.linear.x ==0 ){
    if( !gazebo_flag ){

//        ROS_WARN("TFINIT-no gazebo msg");
        init_x = 0;
        init_y = 0;
        init_z = 0.25;
        init_wx = 0;
        init_wy = 0;
        init_wz = 0;
        init_ww = 1;
    }
    else if( (tfcal_flag)){
//    else if((gazebo_output.pose.pose.position.z > 0.1 ) && (tfcal_flag)){
//        ROS_ERROR("TFINIT-prepare!!!");
        init_x = gazebo_output.pose.pose.position.x;
        init_y = gazebo_output.pose.pose.position.y;
        init_z = gazebo_output.pose.pose.position.z;
        init_wx = gazebo_output.pose.pose.orientation.x;
        init_wy = gazebo_output.pose.pose.orientation.y;
        init_wz = gazebo_output.pose.pose.orientation.z;
        init_ww = gazebo_output.pose.pose.orientation.w;
//        tmp_init_data << init_x,init_y,init_z,init_wx,init_wy,init_wz,init_ww;
        tmp_init_data[0] = init_x;
        tmp_init_data[1] = init_y;
        tmp_init_data[2] = init_z;
        tmp_init_data[3] = init_wx;
        tmp_init_data[4] = init_wy;
        tmp_init_data[5] = init_wz;
        tmp_init_data[6] = init_ww;

        tfcal_flag = false;
        velget_flag = false;
    }
    else{
//        ROS_WARN("TFINIT-use first data");
        init_x = tmp_init_data[0];
        init_y = tmp_init_data[1];
        init_z = tmp_init_data[2];
        init_wx = tmp_init_data[3];
        init_wy = tmp_init_data[4];
        init_wz = tmp_init_data[5];
        init_ww = tmp_init_data[6];


    }
    odom_positioninit << init_x, init_y, init_z;
    odom2init_orientation.x() =  init_wx;
    odom2init_orientation.y() =  init_wy;
    odom2init_orientation.z() =  init_wz;
    odom2init_orientation.w() =  init_ww;
//    Eigen::Vector3d odom2map_YPR;
//    odom2map_YPR = QuaterniondToRPY(odom2init_orientation);

//    cout << "init_x: " << init_x <<
//            "  init_y: " << init_y <<
//            "  init_z: " << init_z <<endl<<
//            "init_ww: "<< init_ww <<
//            "  init_wx: "<< init_wx <<
//            "  init_wy: "<< init_wy <<
//            "  init_wz: "<< init_wz<<endl ;
//            "init_yaw: " << odom2map_YPR(0) <<
//            "  init_pitch: " << odom2map_YPR(1) <<
//            "  init_roll: " << odom2map_YPR(2) <<endl;
}

void QuadrupedEstimation::TFINIT(const nav_msgs::Odometry& gazebo_msg,sensor_msgs::Imu& imu_0, free_gait::JointPositions& jointsall) {
    ROS_WARN("TFINIT !!!");

    if( gazebo_msg.pose.pose.position.x == 0 && gazebo_msg.pose.pose.orientation.w ==1 && gazebo_msg.twist.twist.linear.x ==0 ){
        ROS_WARN("TFINIT-just stand,init---no gazebo msg");
        init_x = 0;
        init_y = 0;
        init_z = 0;
        init_wx = 0;
        init_wy = 0;
        init_wz = 0;
        init_ww = 1;
    }
    else if((imu_0.orientation.w != 0) && (jointsall(10)!=0) && (jointsall(1)!=0) && (jointsall(2)!=0)&&(tfcal_flag)){
        ROS_WARN("TFINIT-move!!!");
        init_x = gazebo_msg.pose.pose.position.x;
        init_y = gazebo_msg.pose.pose.position.y;
        init_z = gazebo_msg.pose.pose.position.z;
        init_wx = gazebo_msg.pose.pose.orientation.x;
        init_wy = gazebo_msg.pose.pose.orientation.y;
        init_wz = gazebo_msg.pose.pose.orientation.z;
        init_ww = gazebo_msg.pose.pose.orientation.w;

        tmp_init_data[0] = init_x;
        tmp_init_data[1] = init_y;
        tmp_init_data[2] = init_z;
        tmp_init_data[3] = init_wx;
        tmp_init_data[4] = init_wy;
        tmp_init_data[5] = init_wz;
        tmp_init_data[6] = init_ww;
        tfcal_flag = false;
        velget_flag = false;

    }
    else{
        ROS_WARN("TFINIT-use first data");
        init_x = tmp_init_data[4];
        init_y = tmp_init_data[5];
        init_z = tmp_init_data[6];
        init_wx = tmp_init_data[0];
        init_wy = tmp_init_data[1];
        init_wz = tmp_init_data[2];
        init_ww = tmp_init_data[3];

    }
    odom_positioninit << init_x, init_y, init_z;
    odom2init_orientation.x() =  init_wx;
    odom2init_orientation.y() =  init_wy;
    odom2init_orientation.z() =  init_wz;
    odom2init_orientation.w() =  init_ww;

//    Eigen::Vector3d odom2map_YPR;
//    odom2map_YPR = QuaterniondToRPY(odom2init_orientation);

//    cout << "init_x: " << init_x <<
//            "  init_y: " << init_y <<
//            "  init_z: " << init_z <<endl<<
//            "init_ww: "<< init_ww <<
//            "  init_wx: "<< init_wx <<
//            "  init_wy: "<< init_wy <<
//            "  init_wz: "<< init_wz<<endl <<
//            "init_yaw: " << odom2map_YPR(0) <<
//            "  init_pitch: " << odom2map_YPR(1) <<
//            "  init_roll: " << odom2map_YPR(2) <<endl;
}

void  QuadrupedEstimation::LegOdomOut(){
//    ROS_WARN("LegOdomOut !!!");
    //***************************//

    //get odom-baselink
    tf::Transform map_odom = tf::Transform(tf::Quaternion(init_wx,init_wy,init_wz,init_ww),
                                               tf::Vector3(init_x,init_y,0.0));
    tf::Transform legmap_baselink = tf::Transform(tf::Quaternion(odom_orientation.x(),odom_orientation.y(),odom_orientation.z(),odom_orientation.w()),
                                               tf::Vector3(odom_position(0),odom_position(1),odom_position(2)));
    legodom_tf = map_odom.inverse() * legmap_baselink;

//******************map--->baselink******************************//
    boost::array<double, 36> odom_pose_covariance ={
                               {1e-6, 0, 0, 0, 0, 0,
                                0, 1e-6, 0, 0, 0, 0,
                                0, 0, 1e-6, 0, 0, 0,
                                0, 0, 0, 1e9, 0, 0,
                                0, 0, 0, 0, 1e9, 0,
                                0, 0, 0, 0, 0, 1e-9}};
    boost::array<double, 36> odom_twist_covariance ={
                               {1e-6, 0, 0, 0, 0, 0,
                                0, 1e-6, 0, 0, 0, 0,
                                0, 0, 1e-6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e-3}};

    boost::array<double, 36> odom_pose_covariance2 ={
                               {1e-9, 0, 0, 0, 0, 0,
                                0, 1e-9, 1e-9, 0, 0, 0,
                                0, 0, 1e-9, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e9}};
    boost::array<double, 36> odom_twist_covariance2 ={
                               {1e-9, 0, 0, 0, 0, 0,
                                0, 1e-9, 1e-9, 0, 0, 0,
                                0, 0, 1e-9, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e9}};
    legodom_map_.header.stamp = ros::Time::now();
    legodom_map_.header.frame_id ="map";
    legodom_map_.child_frame_id = "leg_baselink";
    //P---V----Q
    legodom_map_.pose.pose.position.x = odom_position(0);
    legodom_map_.pose.pose.position.y = odom_position(1);
    legodom_map_.pose.pose.position.z = odom_position(2);
    legodom_map_.pose.pose.orientation.x =  odom_orientation.x();
    legodom_map_.pose.pose.orientation.y =  odom_orientation.y();
    legodom_map_.pose.pose.orientation.z =  odom_orientation.z();
    legodom_map_.pose.pose.orientation.w =  odom_orientation.w();
    legodom_map_.twist.twist.linear.x = odom_vel(0);
    legodom_map_.twist.twist.linear.y = odom_vel(1);
    legodom_map_.twist.twist.linear.z = odom_vel(2);
    if( odom_vel(0) ==0 && odom_vel(1) ==0 && odom_vel(2) ==0){
        legodom_map_.pose.covariance = odom_pose_covariance;
        legodom_map_.twist.covariance = odom_twist_covariance;
    }else{
        legodom_map_.pose.covariance = odom_pose_covariance2;
        legodom_map_.twist.covariance = odom_twist_covariance2;
    }
    legodom_map_pub.publish(legodom_map_);

//    cout << "legodom_map_----x: " << odom_position(0) <<endl <<
//            "legodom_map_----y: " << odom_position(1) <<endl <<
//            "legodom_map_----z: " << odom_position(2) <<endl <<
////            "legodom_map_---yaw:" << odom_YPR(0) <<endl <<
////            "legodom_map_---pitch: " << odom_YPR(1) <<endl <<
////            "legodom_map_---roll: " << odom_YPR(2) <<endl <<
//            "legodom_map_---Vx: " << odom_vel(0) <<endl <<
//            "legodom_map_---Vy: " << odom_vel(1) <<endl <<
//            "legodom_map_---Vz: " << odom_vel(2) <<endl ;

//    cout << "gazebo----x: " << gazebo_output.pose.pose.position.x <<endl <<
//            "gazebo----y: " << gazebo_output.pose.pose.position.y <<endl <<
//            "gazebo----z: " << gazebo_output.pose.pose.position.z <<endl <<
//            "gazebo---Vx: " << gazebo_output.twist.twist.linear.x <<endl <<
//            "gazebo---Vy: " << gazebo_output.twist.twist.linear.y <<endl <<
//            "gazebo---Vz: " << gazebo_output.twist.twist.linear.z <<endl ;
//*********************odom--->baselink***************************//
//    Pose_stamped_.header = legodom_map_.header;
    Pose_stamped_.pose = legodom_map_.pose;
    boost::array<double, 36> pose_covariance ={
                               {1e-6, 0, 0, 0, 0, 0,
                                0, 1e-6, 0, 0, 0, 0,
                                0, 0, 1e-6, 0, 0, 0,
                                0, 0, 0, 1e-9, 0, 0,
                                0, 0, 0, 0, 1e-9, 0,
                                0, 0, 0, 0, 0, 1e-9}};
    Pose_stamped_.pose.covariance = pose_covariance;
    legPose_pub.publish(Pose_stamped_);

    Pose odom_pose;
    kindr_ros::convertFromRosGeometryMsg(legodom_map_.pose.pose, odom_pose);
    robot_state_->setPoseBaseToWorld(odom_pose);
//*********************odom--->baselink***************************//
    leg_odom.header.stamp = ros::Time::now();
    leg_odom.header.frame_id ="legodom";
    leg_odom.child_frame_id = "leg_baselink";
    leg_odom.pose.pose.position.x = legodom_tf.getOrigin().x();
    leg_odom.pose.pose.position.y = legodom_tf.getOrigin().y();
    leg_odom.pose.pose.position.z = legodom_tf.getOrigin().z();
    leg_odom.pose.pose.orientation.x =  legodom_tf.getRotation().x();
    leg_odom.pose.pose.orientation.y =  legodom_tf.getRotation().y();
    leg_odom.pose.pose.orientation.z =  legodom_tf.getRotation().z();
    leg_odom.pose.pose.orientation.w =  legodom_tf.getRotation().w();

    odom_vel_inodom_vec = GetVelInOdom(odom_vel);
    leg_odom.twist.twist.linear.x = odom_vel_inodom_vec(0);
    leg_odom.twist.twist.linear.y = odom_vel_inodom_vec(1);
    leg_odom.twist.twist.linear.z = odom_vel_inodom_vec(2);
    legodom_odom_pub.publish(leg_odom);


//    Eigen::Vector3d base2odom_YPR;
//    Eigen::Quaterniond base2odom_orientaion;
//    base2odom_orientaion.x() = legodom_tf.getRotation().x();
//    base2odom_orientaion.y() = legodom_tf.getRotation().y();
//    base2odom_orientaion.z() = legodom_tf.getRotation().z();
//    base2odom_orientaion.w() = legodom_tf.getRotation().w();
//    base2odom_YPR = QuaterniondToRPY(base2odom_orientaion);
//    cout << "legodom_odom_----x: " << legodom_tf.getOrigin().x() <<endl <<
//            "legodom_odom_----y: " << legodom_tf.getOrigin().y() <<endl <<
//            "legodom_odom_----z: " << legodom_tf.getOrigin().z() <<endl <<
//            "legodom_odom_---yaw:" << base2odom_YPR(0) <<endl <<
//            "legodom_odom_---pitch: " << base2odom_YPR(1) <<endl <<
//            "legodom_odom_---roll: " << base2odom_YPR(2) <<endl <<
//            "legodom_odom_---Vx: " << odom_vel_inodom(0) <<endl <<
//            "legodom_odom_---Vy: " << odom_vel_inodom(1) <<endl <<
//            "legodom_odom_---Vz: " << odom_vel_inodom(2) <<endl ;

//*********************odom--->init***************************//
    legodom_init.header.stamp = ros::Time::now();
    legodom_init.header.frame_id = "map";
    legodom_init.child_frame_id = "legodom";
    legodom_init.pose.pose.position.x = init_x;
    legodom_init.pose.pose.position.x = init_y;
    legodom_init.pose.pose.position.x = 0;
    legodom_init.pose.pose.orientation.x = init_wx;
    legodom_init.pose.pose.orientation.y = init_wy;
    legodom_init.pose.pose.orientation.z = init_wz;
    legodom_init.pose.pose.orientation.w = init_ww;
    legodom_init.twist.twist.linear.x = 0;
    legodom_init.twist.twist.linear.y = 0;
    legodom_init.twist.twist.linear.z = 0;
    legodom_init.twist.twist.angular.x = 0;
    legodom_init.twist.twist.angular.y = 0;
    legodom_init.twist.twist.angular.z = 0;
    legodom_init_pub.publish(legodom_init);

//*********************mapodom--->error***************************//
    legodom_error.pose.pose.position.x = gazebo_output.pose.pose.position.x - odom_position(0);
    legodom_error.pose.pose.position.y = gazebo_output.pose.pose.position.y - odom_position(1);
    legodom_error.pose.pose.position.z = gazebo_output.pose.pose.position.z - odom_position(2);
    legodom_error.twist.twist.linear.x = gazebo_output.twist.twist.linear.x - odom_vel(0);
    legodom_error.twist.twist.linear.y = gazebo_output.twist.twist.linear.y - odom_vel(1);
    legodom_error.twist.twist.linear.z = gazebo_output.twist.twist.linear.z - odom_vel(2);
    legodom_error.twist.twist.angular.x = 0;
    legodom_error.twist.twist.angular.y = 0;
    legodom_error.twist.twist.angular.z = 0;
    legodom_error_pub.publish(legodom_error);
//    cout << "mapodom_error----x: " << legodom_error.pose.pose.position.x <<endl <<
//            "mapodom_error----y: " << legodom_error.pose.pose.position.y <<endl <<
//            "mapodom_error----z: " << legodom_error.pose.pose.position.z <<endl <<
//            "mapodom_error---Vx: " << legodom_error.twist.twist.linear.x <<endl <<
//            "mapodom_error---Vy: " << legodom_error.twist.twist.linear.y <<endl <<
//            "mapodom_error---Vz: " << legodom_error.twist.twist.linear.z <<endl ;

//*********************error_test***************************//
//    gazebo_queue_tmp << gazebo_output.pose.pose.position.x,gazebo_output.pose.pose.position.y,gazebo_output.pose.pose.position.z;
//    if(gazebo_T < 2){
//        gazebo_tmp.push(gazebo_queue_tmp);
//        ++gazebo_T;
//    }else{
//        gazebo_tmp.pop();
//        gazebo_tmp.push(gazebo_queue_tmp);
//    }
//    gazebo_queue = gazebo_tmp.front();
//    gazebo_change = gazebo_queue_tmp - gazebo_queue;
    //every time (gazebo_odom(k) - gazebo_odom(k-1))
//    legodom_error_cal.data[0] = gazebo_change(0);
//    legodom_error_cal.data[1] = gazebo_change(1);
//    legodom_error_cal.data[2] = gazebo_change(2);
//    cout << "gazebo_T: "<< gazebo_T<< endl;
//    cout << "gazebo_queue_tmp----x: " << gazebo_queue_tmp(0) <<endl <<
//            "gazebo_queue_tmp----y: " << gazebo_queue_tmp(1) <<endl <<
//            "gazebo_queue_tmp----z: " << gazebo_queue_tmp(2) <<endl ;
//    cout << "gazebo_queue----x: " << gazebo_queue(0) <<endl <<
//            "gazebo_queue----y: " << gazebo_queue(1) <<endl <<
//            "gazebo_queue----z: " << gazebo_queue(2) <<endl ;
//    cout << "everytime_gazeboerror----x: " << gazebo_change(0) <<endl <<
//            "everytime_gazeboerror----y: " << gazebo_change(1) <<endl <<
//            "everytime_gazeboerror----z: " << gazebo_change(2) <<endl ;

    //***************************//
//    if(odom_position_T < 2){
//        odom_p_tmp.push(odom_position);
//        ++odom_position_T;
//    }else{
//        odom_p_tmp.pop();
//        odom_p_tmp.push(odom_position);
//    }
//    last_odom_position = odom_p_tmp.front();
//    odom_position_error = odom_position - last_odom_position;
//    //every time (legodom_odom(k) - legodom_odom(k-1))
//    legodom_error_cal.data[3] = odom_position_error(0);
//    legodom_error_cal.data[4] = odom_position_error(1);
//    legodom_error_cal.data[5] = odom_position_error(2);
//    cout << "odom_position_T: "<< odom_position_T<< endl;
//    cout << "everytime_odomerror---x: " << odom_position_error(0) <<endl <<
//            "everytime_odomerror---y: " << odom_position_error(1) <<endl <<
//            "everytime_odomerror---z: " << odom_position_error(2) <<endl ;

}

void QuadrupedEstimation::LegTFOut(){
//    ROS_WARN("LegTFOut !!!");
//*********************base2map_tf***************************//
    ros::Time sim_time(imu_output.header.stamp);//ros::Time::now().toSec() - init_time.toSec());
    if(!gazebo_flag)
      sim_time = ros::Time::now();
    //    base2map_tf.header.frame_id = "map";
//    base2map_tf.child_frame_id = "base_link";
//    base2map_tf.header.stamp = sim_time;//ros::Time::now(). - init_time;
//    base2map_tf.transform.translation.x = odom_position(0);
//    base2map_tf.transform.translation.y = odom_position(1);
//    base2map_tf.transform.translation.z = odom_position(2);
//    base2map_tf.transform.rotation.x = odom_orientation.x();
//    base2map_tf.transform.rotation.y = odom_orientation.y();
//    base2map_tf.transform.rotation.z = odom_orientation.z();
//    base2map_tf.transform.rotation.w = odom_orientation.w();
//    base2map_broadcaster.sendTransform(base2map_tf);

//*********************base2odom_tf***************************//
    base2odom_tf.header.frame_id = "odom";
    base2odom_tf.child_frame_id = "base_link";
    base2odom_tf.header.stamp = sim_time;//ros::Time::now() - init_time;
    base2odom_tf.transform.translation.x = legodom_tf.getOrigin().x();
    base2odom_tf.transform.translation.y = legodom_tf.getOrigin().y();
    base2odom_tf.transform.translation.z = legodom_tf.getOrigin().z();
    base2odom_tf.transform.rotation.x = legodom_tf.getRotation().x();
    base2odom_tf.transform.rotation.y = legodom_tf.getRotation().y();
    base2odom_tf.transform.rotation.z = legodom_tf.getRotation().z();
    base2odom_tf.transform.rotation.w = legodom_tf.getRotation().w();
    base2map_broadcaster.sendTransform(base2odom_tf);



    //*********************odom2map_tf***************************//
//    odom2map_tf.header.frame_id = "map";
//    odom2map_tf.child_frame_id = "odom";
//    odom2map_tf.header.stamp = sim_time;//ros::Time::now() - init_time;
//    odom2map_tf.transform.translation.x = init_x;
//    odom2map_tf.transform.translation.y = init_y;
//    odom2map_tf.transform.translation.z = 0;
//    odom2map_tf.transform.rotation.x = init_wx;
//    odom2map_tf.transform.rotation.y = init_wy;
//    odom2map_tf.transform.rotation.z = init_wz;
//    odom2map_tf.transform.rotation.w = init_ww;
//    base2map_broadcaster.sendTransform(odom2map_tf);

//    ROS_WARN("LegTFOut........... !!!");
//    tf::Quaternion q;
//    q.setW(legodom_tf.getRotation().w());
//    q.setX(legodom_tf.getRotation().x());
//    q.setY(legodom_tf.getRotation().y());
//    q.setZ(legodom_tf.getRotation().z());
//    double yaw, pitch, roll;
//    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
//    odom_to_footprint.setRotation(tf::createQuaternionFromYaw(yaw));
//    odom_to_footprint.setOrigin(tf::Vector3(legodom_tf.getOrigin().x(),legodom_tf.getOrigin().y(),0));
//    tfBoardcaster_.sendTransform(tf::StampedTransform(odom_to_footprint, sim_time, "odom", "foot_print"));

//    footprint_to_base.setRotation(tf::createQuaternionFromRPY(roll, pitch, 0.0));
//    footprint_to_base.setOrigin(tf::Vector3(0,0,legodom_tf.getOrigin().z()));
//    tfBoardcaster_.sendTransform(tf::StampedTransform(footprint_to_base, sim_time, "foot_print","base_link"));

    tf::Quaternion q;
    q.setW(odom_orientation.w());
    q.setX(odom_orientation.x());
    q.setY(odom_orientation.y());
    q.setZ(odom_orientation.z());
    double yaw, pitch, roll;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    odom_to_footprint.setRotation(tf::createQuaternionFromYaw(yaw));
    odom_to_footprint.setOrigin(tf::Vector3(odom_position(0),odom_position(1),0));
    tfBoardcaster_.sendTransform(tf::StampedTransform(odom_to_footprint, sim_time, "odom", "foot_print"));

    footprint_to_base.setRotation(tf::createQuaternionFromRPY(roll, pitch, 0.0));
    footprint_to_base.setOrigin(tf::Vector3(0,0,odom_position(2)));
    tfBoardcaster_.sendTransform(tf::StampedTransform(footprint_to_base, sim_time, "foot_print","base_link"));
}

void QuadrupedEstimation::LegPubTopic(ros::NodeHandle& _nn){
    ROS_WARN("LegPubTopic !!!");
    //pub topic
    legodom_odom_pub = _nn.advertise<nav_msgs::Odometry>("/legodom", 10);
    legodom_map_pub = _nn.advertise<nav_msgs::Odometry>("/legodom_map", 10);
    legodom_error_pub = _nn.advertise<nav_msgs::Odometry>("/legodom_error", 10);
    legodom_init_pub = _nn.advertise<nav_msgs::Odometry>("/legodom_init", 10);


}


void QuadrupedEstimation::QuadrupedPosition() {
//    ROS_WARN("QuadrupedPosition...");
    //get parms!
    GetPositionParms();

//    cout << "_cal_position_way: " << _cal_position_way << endl;
    //get 3 way to cal position
    if(_cal_position_way == "everytime") {
        GetPositionAddEveryTime();
    }
    else if(_cal_position_way == "everystep") {
        GetPositionAddEveryStep();
    }
    else if(_cal_position_way == "frominit") {
        GetPositionAddFromInit();
    }
    else if(_cal_position_way == "footstep") {
        //just for test, no use for later
        GetPositionAddFootStep();
    }
    else if(_cal_position_way == "everytime4foot") {
        GetPositionAddEveryTimeInAllFoot();
    }

    //test and check
//    ROS_WARN("legposition: ");
//    cout<< "leg1_first - P_BW[0]:  " << P_leg1 - P_BW[0] << endl <<
//           "leg2_first - P_BW[1]:  " << P_leg2 - P_BW[1] << endl <<
//           "leg3_first - P_BW[2]:  " << P_leg3 - P_BW[2] << endl <<
//           "leg4_first - P_BW[3]:  " << P_leg4 - P_BW[3] << endl;
//    ROS_WARN("compare:::::::::::::::::::::::");
//    cout<< "leg1_first :  " << P_leg1 << endl <<
//           "leg2_first :  " << P_leg2 << endl <<
//           "leg3_first :  " << P_leg3 << endl <<
//           "leg4_first :  " << P_leg4 << endl;
//    ROS_WARN("compare");
//    cout<< "P_BW[0]:  " << P_BW[0] << endl <<
//           "P_BW[1]:  " << P_BW[1] << endl <<
//           "P_BW[2]:  " << P_BW[2] << endl <<
//           "P_BW[3]:  " << P_BW[3] << endl;
//    cout << "sum cycle times: " << cycle_step <<endl << "cycle_T: " << cycle_T <<endl;
//    cout<< "odom_position:  " << odom_position <<endl;

}


void QuadrupedEstimation::GetPositionAddEveryTimeInAllFoot(){
    ////**********GetPositionAddEveryTimeInAllFoot************////
//    ROS_INFO("GetPositionAddEveryTimeInAllFoot");
    if(P_init_flag){
        if(foot_flag == 1111 || foot_flag == 10000){
            P_leg1 = P_BW[0];
            P_leg2 = P_BW[1];
            P_leg3 = P_BW[2];
            P_leg4 = P_BW[3];
            odom_position(0) = ( (P_leg1(0)-leg1_first(0))+(P_leg2(0)-leg2_first(0))+(P_leg3(0)-leg3_first(0))+(P_leg4(0)-leg4_first(0)))/4 ;
            odom_position(1) = ( (P_leg1(1)-leg1_first(1))+(P_leg2(1)-leg2_first(1))+(P_leg3(1)-leg3_first(1))+(P_leg4(1)-leg4_first(1)))/4 ;
            odom_position(2) = -( P_leg1(2)+P_leg2(2)+P_leg3(2)+P_leg4(2))/4;
        }else{
//            ROS_ERROR("get wrong init state:!");
            P_init_flag = false;
        }

        if(P_init_time > 10){
            P_init_flag = false;
//            ROS_ERROR("In Init stage---turn cal stage");
        }else{
            ++P_init_time;
//            ROS_INFO("In Init stage---last for 10 time");
        }

    }
    else{
//            cout << "foot_flag: " << foot_flag << endl;
            if(first_foot_flag == 1111 && first_check_p){
//                ROS_ERROR("first inter flag: 1111---get init data msg");
                odom_position = odom_positioninit + Position(0,0,0.0);
                first_check_p = false;
            }

            P_leg1_ = P_BW[0];
            P_leg2_ = P_BW[1];
            P_leg3_ = P_BW[2];
            P_leg4_ = P_BW[3];
            double number_of_contact = foot_output.data[0] + foot_output.data[1] + foot_output.data[2] + foot_output.data[3];
            if(number_of_contact>=2)
              {
                odom_position_tmp(0) = -_cal_fator_x*( foot_output.data[0]*(P_leg1_(0) - leg1_first(0)) + foot_output.data[1]*(P_leg2_(0) - leg2_first(0)) + foot_output.data[2]*(P_leg3_(0) - leg3_first(0))
                    +foot_output.data[3]* (P_leg4_(0) - leg4_first(0) ))/number_of_contact;
                odom_position_tmp(1) = -_cal_fator_y*( foot_output.data[0]*(P_leg1_(1) - leg1_first(1)) + foot_output.data[1]*(P_leg2_(1) - leg2_first(1)) + foot_output.data[2]*(P_leg3_(1) - leg3_first(1))
                    +foot_output.data[3]* (P_leg4_(1) - leg4_first(1) ))/number_of_contact;
//                odom_position_tmp(2) = -( foot_output.data[0]*(P_leg1_(2)) + foot_output.data[1]*(P_leg2_(2)) + foot_output.data[2]*(P_leg3_(2))
//                    +foot_output.data[3]* (P_leg4_(2)))/number_of_contact + 0.03;
                odom_position_tmp(2) = -_cal_fator_z*( foot_output.data[0]*(P_leg1_(2) - leg1_first(2)) + foot_output.data[1]*(P_leg2_(2)- leg2_first(2)) + foot_output.data[2]*(P_leg3_(2) - leg3_first(2))
                    +foot_output.data[3]* (P_leg4_(2) - leg4_first(2)))/number_of_contact;// + 0.03;

                odom_position(0) += odom_position_tmp(0);
                odom_position(1) += odom_position_tmp(1);
                odom_position(2) += odom_position_tmp(2);
              }
            //! WSHY: When there are unexpected large joint vel in the end of last stance, it will cause wrong position
            //! estimate, consider using average stance vel
//            odom_position(0) += odom_position_tmp(0);
//            odom_position(1) += odom_position_tmp(1);
//            odom_position(2) += odom_position_tmp(2);
        }

}

void QuadrupedEstimation::GetPositionAddEveryTime(){
    ////**********EveryTime************////
    ROS_INFO("GetPositionAddEveryTime");
    if(P_init_flag){
        if(foot_flag == 1111 || foot_flag == 10000){
            P_leg1 = P_BW[0];
            P_leg2 = P_BW[1];
            P_leg3 = P_BW[2];
            P_leg4 = P_BW[3];
            odom_position(0) = ( (P_leg1(0)-leg1_first(0))+(P_leg2(0)-leg2_first(0))+(P_leg3(0)-leg3_first(0))+(P_leg4(0)-leg4_first(0)))/4 ;
            odom_position(1) = ( (P_leg1(1)-leg1_first(1))+(P_leg2(1)-leg2_first(1))+(P_leg3(1)-leg3_first(1))+(P_leg4(1)-leg4_first(1)))/4 ;
            odom_position(2) = -( P_leg1(2)+P_leg2(2)+P_leg3(2)+P_leg4(2))/4;
        }else{
            ROS_ERROR("get wrong init state:!");
            P_init_flag = false;
        }

        if(P_init_time > 10){
            P_init_flag = false;
            ROS_ERROR("In Init stage---turn cal stage");
        }else{
            ++P_init_time;
            ROS_INFO("In Init stage---last for 10 time");
        }

    }
    else{
        if(foot_flag == 10000)
        {
            //test
            if((foot_flag == first_foot_flag) && (no_data_input_flag)){
                ROS_INFO("no data---init flag: 0000");
            }
            else if((foot_flag != first_foot_flag ) && (no_data_input_flag) ){
                ROS_INFO("last get data---now no data(first): 0000");
                no_data_input_flag = false;
            }
            else if((foot_flag == first_foot_flag) && (!no_data_input_flag)){
                ROS_INFO("before get data---again no data(again+++): 0000");
            }
            else if((foot_flag != first_foot_flag) && (!no_data_input_flag)){
                ROS_INFO("before get data---again no data(second+++): 0000");
            }
            odom_position = odom_position;
        }
        else if(foot_flag == 1111)
        {

            if(first_foot_flag == 1111 && first_check_p){
                ROS_ERROR("first inter flag: 1111---get init data msg");
                odom_position = odom_positioninit;
                first_check_p = false;
            }

            else if(first_foot_flag != 1111 && first_foot_flag != 10000){
                ROS_WARN("again inter flag: 1111");
                P_leg1 = P_BW[0];
                P_leg2 = P_BW[1];
                P_leg3 = P_BW[2];
                P_leg4 = P_BW[3];
                odom_position_tmp = -( (P_BW[0]-P_leg1)+(P_BW[1]-P_leg2)+(P_BW[2]-P_leg3)+(P_BW[3]-P_leg4) )/4 ;
                odom_position += odom_position_tmp;
            }
            else{
//                if(leg1_first(2) > -0.46){
//                if(foot_cb_flag){
//                    ROS_WARN("previous data is 0!!!");
//                    return;
//                }else{
//                    ROS_INFO("inter flag: 1111");
//                    P_leg1 = leg1_first;
//                    P_leg2 = leg2_first;
//                    P_leg3 = leg3_first;
//                    P_leg4 = leg4_first;
//                    odom_position_tmp = -( (P_BW[0]-P_leg1)+(P_BW[1]-P_leg2)+(P_BW[2]-P_leg3)+(P_BW[3]-P_leg4) )/4 ;
//                    odom_position += odom_position_tmp;
//                }

                ROS_INFO("inter flag: 1111");
                P_leg1 = leg1_first;
                P_leg2 = leg2_first;
                P_leg3 = leg3_first;
                P_leg4 = leg4_first;
                odom_position_tmp = -( (P_BW[0]-P_leg1)+(P_BW[1]-P_leg2)+(P_BW[2]-P_leg3)+(P_BW[3]-P_leg4) )/4 ;
                odom_position += odom_position_tmp;

            }

        }
        else if(foot_flag == 1010)
        {
//            if( (first_foot_flag != foot_flag) && first_foot_flag == 1111){
//                ROS_INFO("first inter flag: 1010");
//                cycle_T = cycle_T + 1;
//                P_leg1 = leg1_first;
//                P_leg3 = leg3_first;
//                P_leg2 = leg2_first;
//                P_leg4 = leg4_first;
//                odom_position_tmp = -( (P_BW[0]-P_leg1)+(P_BW[2]-P_leg3))/2 ;
//            }
//            else if( (first_foot_flag != foot_flag) && first_foot_flag != 1111 ){
//                ROS_INFO("again step inter flag: 1010");
//                cycle_T = cycle_T + 1;
//                odom_position_tmp << 0,0,0;
//            }else{
//                ROS_INFO("inter flag: 1010");
//                P_leg1 = leg1_first;
//                P_leg3 = leg3_first;
//                P_leg2 = leg2_first;
//                P_leg4 = leg4_first;
//                odom_position_tmp = -( (P_BW[0]-P_leg1)+(P_BW[2]-P_leg3))/2 ;
//            }
////            P_leg1 = leg1_first;
////            P_leg3 = leg3_first;
////            odom_position_tmp = -( (P_BW[0].vector()-P_leg1)+(P_BW[2].vector()-P_leg3))/2 ;
//            odom_position += odom_position_tmp;
//        }

        if( (first_foot_flag != foot_flag) && first_foot_flag == 1111){
            ROS_INFO("first inter flag: 1010");
            cycle_T = cycle_T + 1;
            P_leg1_ = P_BW[0];
            P_leg2_ = P_BW[1];
            P_leg3_ = P_BW[2];
            P_leg4_ = P_BW[3];
            odom_position_tmp(0) = -_cal_fator_x*( (P_leg1_(0) - leg1_first(0))+ (P_leg3_(0) - leg3_first(0)))/2;
            odom_position_tmp(1) = -_cal_fator_y*( (P_leg1_(1) - leg1_first(1))+ (P_leg3_(1) - leg3_first(1)))/2;
            odom_position_tmp(2) = -( P_leg1_(2) + P_leg3_(2) )/2;
            odom_position(0) += odom_position_tmp(0);
            odom_position(1) += odom_position_tmp(1);
            odom_position(2) = odom_position_tmp(2);
        }
        else if( (first_foot_flag != foot_flag) && first_foot_flag != 1111 ){
            ROS_INFO("again step inter flag: 1010");
            cycle_T = cycle_T + 1;
            P_leg1_ = P_BW[0];
            P_leg2_ = P_BW[1];
            P_leg3_ = P_BW[2];
            P_leg4_ = P_BW[3];
            odom_position_tmp(0) = 0;
            odom_position_tmp(1) = 0;
            odom_position_tmp(2) = -( P_leg1_(2) + P_leg3_(2) )/2;
            odom_position(0) += odom_position_tmp(0);
            odom_position(1) += odom_position_tmp(1);
            odom_position(2) = odom_position_tmp(2);
        }else{
            ROS_INFO("inter flag: 1010");
            P_leg1_ = P_BW[0];
            P_leg2_ = P_BW[1];
            P_leg3_ = P_BW[2];
            P_leg4_ = P_BW[3];
            odom_position_tmp(0) = -_cal_fator_x*( (P_leg1_(0) - leg1_first(0))+ (P_leg3_(0) - leg3_first(0)))/2;
            odom_position_tmp(1) = -_cal_fator_y*( (P_leg1_(1) - leg1_first(1))+ (P_leg3_(1) - leg3_first(1)))/2;
            odom_position_tmp(2) = -( P_leg1_(2) + P_leg3_(2) )/2;
//            odom_position_tmp(2) = odom_position_tmp(2);
            odom_position(0) += odom_position_tmp(0);
            odom_position(1) += odom_position_tmp(1);
            odom_position(2) = odom_position_tmp(2);
        }

    }


        else if(foot_flag == 10101)
        {
//            P_leg1 = leg1_first;
//            P_leg3 = leg3_first;
//            P_leg2 = leg2_first;
//            P_leg4 = leg4_first;
//            if( (first_foot_flag != foot_flag) && first_foot_flag == 1111){
//                ROS_INFO("first inter flag: 10101");
//                cycle_T = cycle_T + 1;
//                odom_position_tmp = -( (P_BW[1]-P_leg2)+(P_BW[3]-P_leg4))/2 ;
//            }
//            else if( (first_foot_flag != foot_flag) && first_foot_flag != 1111 ){
//                ROS_INFO("again step inter flag: 10101");
//                cycle_T = cycle_T + 1;
//                odom_position_tmp << 0,0,0;
//            }else{
//                ROS_INFO("inter flag: 10101");
//                odom_position_tmp = -( (P_BW[1]-P_leg2)+(P_BW[3]-P_leg4))/2 ;
//            }
////            P_leg2 = leg2_first;
////            P_leg4 = leg4_first;
////            odom_position_tmp = -( (P_BW[1].vector()-P_leg2)+(P_BW[3].vector()-P_leg4))/2 ;
//            odom_position += odom_position_tmp;


            if( (first_foot_flag != foot_flag) && first_foot_flag == 1111){
                ROS_INFO("first inter flag: 10101");
                cycle_T = cycle_T + 1;
                P_leg1_ = P_BW[0];
                P_leg2_ = P_BW[1];
                P_leg3_ = P_BW[2];
                P_leg4_ = P_BW[3];
//                odom_position_tmp = -( (P_BW[1]-P_leg2)+(P_BW[3]-P_leg4))/2 ;
                odom_position_tmp(0) = -_cal_fator_x*( (P_leg2_(0) - leg2_first(0))+ (P_leg4_(0) - leg4_first(0)))/2;
                odom_position_tmp(1) = -_cal_fator_y*( (P_leg2_(1) - leg2_first(1))+ (P_leg4_(1) - leg4_first(1)))/2;
                odom_position_tmp(2) = -( P_leg2_(2) + P_leg4_(2) )/2;
                odom_position(0) += odom_position_tmp(0);
                odom_position(1) += odom_position_tmp(1);
                odom_position(2) = odom_position_tmp(2);
            }
            else if( (first_foot_flag != foot_flag) && first_foot_flag != 1111 ){
                ROS_INFO("again step inter flag: 10101");
                cycle_T = cycle_T + 1;
                P_leg1_ = P_BW[0];
                P_leg2_ = P_BW[1];
                P_leg3_ = P_BW[2];
                P_leg4_ = P_BW[3];
                odom_position_tmp(0) = 0;
                odom_position_tmp(1) = 0;
                odom_position_tmp(2) = -( P_leg2_(2) + P_leg4_(2) )/2;
                odom_position(0) += odom_position_tmp(0);
                odom_position(1) += odom_position_tmp(1);
                odom_position(2) = odom_position_tmp(2);

            }else{
                ROS_INFO("inter flag: 10101");
                P_leg1_ = P_BW[0];
                P_leg2_ = P_BW[1];
                P_leg3_ = P_BW[2];
                P_leg4_ = P_BW[3];
                odom_position_tmp(0) = -_cal_fator_x*( (P_leg2_(0) - leg2_first(0))+ (P_leg4_(0) - leg4_first(0)))/2;
                odom_position_tmp(1) = -_cal_fator_y*( (P_leg2_(1) - leg2_first(1))+ (P_leg4_(1) - leg4_first(1)))/2;
                odom_position_tmp(2) = -( P_leg2_(2) + P_leg4_(2) )/2;
//                odom_position_tmp(2) = odom_position_tmp(2);
                odom_position(0) += odom_position_tmp(0);
                odom_position(1) += odom_position_tmp(1);
                odom_position(2) = odom_position_tmp(2);
            }

        }

        //cal
        if(cycle_T == 0){
            cycle_step = 0;
        }else{
            cycle_step = (cycle_T-1) /2 +1;
        }
    }
}


void QuadrupedEstimation::GetPositionAddEveryStep(){
    ////**********EveryStep************////
    ROS_INFO("GetPositionAddEveryStep");
    if(P_init_flag){
        if(foot_flag == 1111 || foot_flag == 10000){
            P_leg1 = P_BW[0];
            P_leg2 = P_BW[1];
            P_leg3 = P_BW[2];
            P_leg4 = P_BW[3];
            odom_position(0) = ( (P_leg1(0)-leg1_first(0))+(P_leg2(0)-leg2_first(0))+(P_leg3(0)-leg3_first(0))+(P_leg4(0)-leg4_first(0)))/4 ;
            odom_position(1) = ( (P_leg1(1)-leg1_first(1))+(P_leg2(1)-leg2_first(1))+(P_leg3(1)-leg3_first(1))+(P_leg4(1)-leg4_first(1)))/4 ;
            odom_position(2) = -( P_leg1(2)+P_leg2(2)+P_leg3(2)+P_leg4(2))/4;
        }else{
            ROS_ERROR("get wrong init state:!");
            P_init_flag = false;
        }

        if(P_init_time > 10){
            P_init_flag = false;
            ROS_ERROR("In Init stage---turn cal stage");
        }else{
            ++P_init_time;
            ROS_INFO("In Init stage---last for 10 time");
        }
    }
    else{
        if(foot_flag == 10000)
        {
            //test
            if((foot_flag == first_foot_flag) && (no_data_input_flag)){
                ROS_INFO("no data---init flag: 0000");
            }
            else if((foot_flag != first_foot_flag ) && (no_data_input_flag) ){
                ROS_INFO("last get data---now no data(first): 0000");
                no_data_input_flag = false;
            }
            else if((foot_flag == first_foot_flag) && (!no_data_input_flag)){
                ROS_INFO("before get data---again no data(again+++): 0000");
            }
            else if((foot_flag != first_foot_flag) && (!no_data_input_flag)){
                ROS_INFO("before get data---again no data(second+++): 0000");
            }
            odom_position = odom_position;
        }
        else if(foot_flag == 1111)
        {
            if(first_foot_flag == 10000 && first_check_p){
                ROS_ERROR("first inter flag: 1111---get init data msg");
                odom_position = odom_positioninit;
                first_check_p = false;
            }

            else if(first_foot_flag != 1111 && first_foot_flag != 10000){
                ROS_WARN("again inter flag: 1111");
                P_leg1 = P_BW[0];
                P_leg2 = P_BW[1];
                P_leg3 = P_BW[2];
                P_leg4 = P_BW[3];
                odom_position_tmp = -( (P_BW[0]-P_leg1)+(P_BW[1]-P_leg2)+(P_BW[2]-P_leg3)+(P_BW[3]-P_leg4) )/4 ;
                odom_position += odom_position_tmp;
            }
            else{
//                if(leg1_first(2) > -0.46){
                if(foot_cb_flag){
                    ROS_WARN("previous data is 0!!!");
                    return;
                }else{
                    ROS_INFO("inter flag: 1111");
                    P_leg1 = leg1_first;
                    P_leg2 = leg2_first;
                    P_leg3 = leg3_first;
                    P_leg4 = leg4_first;
                    odom_position_tmp = -( (P_BW[0]-P_leg1)+(P_BW[1]-P_leg2)+(P_BW[2]-P_leg3)+(P_BW[3]-P_leg4) )/4 ;
                    odom_position += odom_position_tmp;
                }
            }
        }
        else if(foot_flag == 1010)
        {
//            P_leg1 = leg1_first;
//            P_leg3 = leg3_first;
//            P_leg2 = leg2_first;
//            P_leg4 = leg4_first;
//            if( (first_foot_flag != foot_flag) && first_foot_flag == 1111){
//                ROS_INFO("first inter flag: 1010");
//                cycle_T = cycle_T + 1;
//                odom_position_tmp = -( (P_BW[0]-P_leg1)+(P_BW[2]-P_leg3))/2 ;
//                odom_position += odom_position_tmp;
//                tmp_legposition = odom_position;
//            }
//            else if( (first_foot_flag != foot_flag) && first_foot_flag != 1111 ){
//                ROS_INFO("again step inter flag: 1010");
//                cycle_T = cycle_T + 1;
//                odom_position_tmp << 0,0,0;
//                odom_position += odom_position_tmp;
//                tmp_legposition = odom_position;
//            }else{
//                ROS_INFO("inter flag: 1010");
//                odom_position_tmp = -( (P_BW[0]-P_leg1)+(P_BW[2]-P_leg3))/2 ;
//                odom_position = tmp_legposition + odom_position_tmp;
//            }

            P_leg1_ = P_BW[0];
            P_leg2_ = P_BW[1];
            P_leg3_ = P_BW[2];
            P_leg4_ = P_BW[3];
            if( (first_foot_flag != foot_flag) && first_foot_flag == 1111){
                P_leg1_tmp = P_leg1_;
                P_leg2_tmp = P_leg2_;
                P_leg3_tmp = P_leg3_;
                P_leg4_tmp = P_leg4_;
                ROS_INFO("first inter flag: 1010");
                cycle_T = cycle_T + 1;
                odom_position_tmp(0) = -( (P_leg1_(0) - leg1_first(0))+ (P_leg3_(0) - leg3_first(0)))/2;
                odom_position_tmp(1) = -( (P_leg1_(1) - leg1_first(1))+ (P_leg3_(1) - leg3_first(1)))/2;
                odom_position_tmp(2) = -( P_leg1_(2) + P_leg3_(2) )/2;
                odom_position(0) += odom_position_tmp(0);
                odom_position(1) += odom_position_tmp(1);
                odom_position(2) = odom_position_tmp(2);
                tmp_legposition = odom_position;
            }
            else if( (first_foot_flag != foot_flag) && first_foot_flag != 1111 ){
                P_leg1_tmp = P_leg1_;
                P_leg2_tmp = P_leg2_;
                P_leg3_tmp = P_leg3_;
                P_leg4_tmp = P_leg4_;
                ROS_INFO("again step inter flag: 1010");
                cycle_T = cycle_T + 1;
                odom_position_tmp(0) = 0;
                odom_position_tmp(1) = 0;
                odom_position_tmp(2) = -( P_leg1_(2) + P_leg3_(2) )/2;
                odom_position(0) += odom_position_tmp(0);
                odom_position(1) += odom_position_tmp(1);
                odom_position(2) = odom_position_tmp(2);
                tmp_legposition = odom_position;

            }else{
                ROS_INFO("inter flag: 1010");
                odom_position_tmp(0) = -( (P_leg1_(0) - P_leg1_tmp(0))+ (P_leg3_(0) - P_leg3_tmp(0)))/2;
                odom_position_tmp(1) = -( (P_leg1_(1) - P_leg1_tmp(1))+ (P_leg3_(1) - P_leg3_tmp(1)))/2;
    //            odom_position_tmp(2) = -( P_leg1_(2)-P_leg1_tmp + P_leg3_(2)-P_leg3_tmp )/2;
                odom_position(0) = tmp_legposition(0) + odom_position_tmp(0);
                odom_position(1) = tmp_legposition(1) + odom_position_tmp(1);
                odom_position(2) = tmp_legposition(2) ;
            }
        }
        else if(foot_flag == 10101)
        {
//            P_leg1 = leg1_first;
//            P_leg3 = leg3_first;
//            P_leg2 = leg2_first;
//            P_leg4 = leg4_first;
//            if( (first_foot_flag != foot_flag) && first_foot_flag == 1111){
//                ROS_INFO("first inter flag: 10101");
//                cycle_T = cycle_T + 1;
//                odom_position_tmp = -( (P_BW[1]-P_leg2)+(P_BW[3]-P_leg4))/2 ;
//                odom_position += odom_position_tmp;
//                tmp_legposition = odom_position;
//            }
//            else if( (first_foot_flag != foot_flag) && first_foot_flag != 1111 ){
//                ROS_INFO("again step inter flag: 10101");
//                cycle_T = cycle_T + 1;
//                odom_position_tmp << 0,0,0;
//                odom_position_tmp << 0,0,0;
//                odom_position += odom_position_tmp;
//                tmp_legposition = odom_position;
//            }else{
//                ROS_INFO("inter flag: 10101");
//                odom_position_tmp = -( (P_BW[1]-P_leg2)+(P_BW[3]-P_leg4))/2 ;
//                odom_position = tmp_legposition + odom_position_tmp;
//            }

            P_leg1_ = P_BW[0];
            P_leg2_ = P_BW[1];
            P_leg3_ = P_BW[2];
            P_leg4_ = P_BW[3];
            if( (first_foot_flag != foot_flag) && first_foot_flag == 1111){
                ROS_INFO("first inter flag: 10101");
                cycle_T = cycle_T + 1;
                P_leg1_tmp = P_leg1_;
                P_leg2_tmp = P_leg2_;
                P_leg3_tmp = P_leg3_;
                P_leg4_tmp = P_leg4_;
                odom_position_tmp(0) = -( (P_leg2_(0) - leg2_first(0))+ (P_leg4_(0) - leg4_first(0)))/2;
                odom_position_tmp(1) = -( (P_leg2_(1) - leg2_first(1))+ (P_leg4_(1) - leg4_first(1)))/2;
                odom_position_tmp(2) = -( P_leg2_(2) + P_leg4_(2) )/2;
                odom_position(0) += odom_position_tmp(0);
                odom_position(1) += odom_position_tmp(1);
                odom_position(2) = odom_position_tmp(2);
                tmp_legposition = odom_position;
            }
            else if( (first_foot_flag != foot_flag) && first_foot_flag != 1111 ){
                ROS_INFO("again step inter flag: 10101");
                cycle_T = cycle_T + 1;
                P_leg1_tmp = P_leg1_;
                P_leg2_tmp = P_leg2_;
                P_leg3_tmp = P_leg3_;
                P_leg4_tmp = P_leg4_;
                odom_position_tmp(0) = 0;
                odom_position_tmp(1) = 0;
                odom_position_tmp(2) = -( P_leg2_(2) + P_leg4_(2) )/2;
                odom_position(0) += odom_position_tmp(0);
                odom_position(1) += odom_position_tmp(1);
                odom_position(2) = odom_position_tmp(2);
                tmp_legposition = odom_position;

            }else{
                ROS_INFO("inter flag: 10101");
                odom_position_tmp(0) = -( (P_leg2_(0) - P_leg2_tmp(0))+ (P_leg4_(0) - P_leg4_tmp(0)))/2;
                odom_position_tmp(1) = -( (P_leg2_(1) - P_leg2_tmp(1))+ (P_leg4_(1) - P_leg4_tmp(1)))/2;
    //            odom_position_tmp(2) = -( P_leg2_(2)-P_leg2_tmp(2) + P_leg4_(2)-P_leg4_tmp(2) )/2;
                odom_position(0) = tmp_legposition(0) + odom_position_tmp(0);
                odom_position(1) = tmp_legposition(1) + odom_position_tmp(1);
                odom_position(2) = tmp_legposition(2) ;
            }
        }

        ////cal
        if(cycle_T == 0){
            cycle_step = 0;
        }else{
            cycle_step = (cycle_T-1) /2 + 1;
        }
        cout << "odom_position_tmp: " << odom_position_tmp << endl;
        ROS_INFO("In cal stage");
    }

}

void QuadrupedEstimation::GetPositionAddFromInit() {
    ////**********FromInit************////
    ROS_INFO("GetPositionAddFromInit");
    ///////////need to know ever step length,so don't need to try!///////////
    cycle_long << cycle_T*X_everystep;

    if(P_init_flag){
        if(foot_flag == 1111 || foot_flag == 10000){
            P_leg1 = P_BW[0];
            P_leg2 = P_BW[1];
            P_leg3 = P_BW[2];
            P_leg4 = P_BW[3];
            odom_position(0) = ( (P_leg1(0)-leg1_first(0))+(P_leg2(0)-leg2_first(0))+(P_leg3(0)-leg3_first(0))+(P_leg4(0)-leg4_first(0)))/4 ;
            odom_position(1) = ( (P_leg1(1)-leg1_first(1))+(P_leg2(1)-leg2_first(1))+(P_leg3(1)-leg3_first(1))+(P_leg4(1)-leg4_first(1)))/4 ;
            odom_position(2) = ( P_leg1(2)+P_leg2(2)+P_leg3(2)+P_leg4(2))/4;
            first_tmp_position = odom_position;
        }else{
            ROS_ERROR("get wrong init state:!");
        }
        if(P_init_time > 10){
            P_init_flag = false;
        }
        ROS_INFO("In Init stage");
    }
    else{
        if(foot_flag == 10000)
        {
            //test
            if((foot_flag == first_foot_flag) && (no_data_input_flag)){
                ROS_INFO("no data---init flag: 0000");
            }
            else if((foot_flag != first_foot_flag ) && (no_data_input_flag) ){
                ROS_INFO("last get data---now no data(first): 0000");
                no_data_input_flag = false;
            }
            else if((foot_flag == first_foot_flag) && (!no_data_input_flag)){
                ROS_INFO("before get data---again no data(again+++): 0000");
            }
            else if((foot_flag != first_foot_flag) && (!no_data_input_flag)){
                ROS_INFO("before get data---again no data(second+++): 0000");
            }
            odom_position = odom_position;
        }
        else if(foot_flag == 1111)
        {
            if(first_foot_flag != 1111){
                ROS_INFO("first/again inter flag: 1111");
                odom_position = odom_position;
            }else{
                ROS_INFO("inter flag: 1111");
                P_leg1 = P_BW[0];
                P_leg2 = P_BW[1];
                P_leg3 = P_BW[2];
                P_leg4 = P_BW[3];
                odom_position(0) = ( (P_leg1(0)-leg1_first(0))+(P_leg2(0)-leg2_first(0))+(P_leg3(0)-leg3_first(0))+(P_leg4(0)-leg4_first(0)))/4 ;
                odom_position(1) = ( (P_leg1(1)-leg1_first(1))+(P_leg2(1)-leg2_first(1))+(P_leg3(1)-leg3_first(1))+(P_leg4(1)-leg4_first(1)))/4 ;
                odom_position(2) = ( P_leg1(2)+P_leg2(2)+P_leg3(2)+P_leg4(2))/4;
            }
        }
        else if(foot_flag == 1010)
        {
            if( (first_foot_flag != foot_flag) && first_foot_flag == 1111){
                ROS_INFO("first inter flag: 1010");
                cycle_T = cycle_T + 1;
                P_leg1 = leg1_first + cycle_long;
                P_leg3 = leg3_first + cycle_long;
            }
            else if( (first_foot_flag != foot_flag) && first_foot_flag != 1111 ){
                ROS_INFO("again step inter flag: 1010");
                cycle_T = cycle_T + 1;
                P_leg1 = P_BW[0] + cycle_long;
                P_leg3 = P_BW[2] + cycle_long;
//                P_leg1 = leg1_first + cycle_long;
//                P_leg3 = leg3_first + cycle_long;

            }else{
                ROS_INFO("inter flag: 1010");
            }
            odom_position_tmp = -( (P_BW[0]-P_leg1)+(P_BW[2]-P_leg3))/2 ;
            odom_position = first_tmp_position + odom_position_tmp;

        }
        else if(foot_flag == 10101)
        {
            P_leg1 = leg1_first;
            P_leg3 = leg3_first;
            P_leg2 = leg2_first;
            P_leg4 = leg4_first;
            if( (first_foot_flag != foot_flag) && first_foot_flag == 1111){
                ROS_INFO("first inter flag: 10101");
                cycle_T = cycle_T + 1;
                P_leg2 = leg2_first + cycle_long;
                P_leg4 = leg4_first + cycle_long;
            }
            else if( (first_foot_flag != foot_flag) && first_foot_flag != 1111 ){
                ROS_INFO("again step inter flag: 10101");
                cycle_T = cycle_T + 1;
//                P_leg2 = P_BW[1] + cycle_long;
//                P_leg4 = P_BW[3] + cycle_long;
                P_leg2 = leg2_first + cycle_long;
                P_leg4 = leg4_first + cycle_long;
            }else{
                ROS_INFO("inter flag: 10101");
            }

            odom_position_tmp = -( (P_BW[1]-P_leg2)+(P_BW[3]-P_leg4))/2 ;
            odom_position = first_tmp_position + odom_position_tmp ;

        }

        ////cal
        if(cycle_T == 0){
            cycle_step = 0;
        }else{
            cycle_step = (cycle_T-1) /4;
        }

        cout << "odom_position_tmp: " << odom_position_tmp << endl;

        ROS_INFO("In cal stage");
    }

}

void QuadrupedEstimation::GetPositionAddFootStep() {
    ROS_INFO("GetPositionAddFootStep");
    ///////////need to know ever foot length,so don't need to try!///////////
//    ////**********FootStep************////
//    Eigen::Vector3d tmp1,tmp2,tmp22; //init
//    tmp1 << 0,-(double)200/3000,0;
//    tmp2 << 0,(double)200/3000,0 ;
//    tmp22 << 0,(double)200/3000,0 ;

//    ///////////////
//    if(P_init_flag){
//        if(foot_flag == 1111 || foot_flag == 10000){
//            P_leg1 = P_BW[0].vector();
//            P_leg2 = P_BW[1].vector();
//            P_leg3 = P_BW[2].vector();
//            P_leg4 = P_BW[3].vector();
//            odom_position(0) = ( (P_leg1(0)-leg1_first(0))+(P_leg2(0)-leg2_first(0))+(P_leg3(0)-leg3_first(0))+(P_leg4(0)-leg4_first(0)))/4 ;
//            odom_position(1) = ( (P_leg1(1)-leg1_first(1))+(P_leg2(1)-leg2_first(1))+(P_leg3(1)-leg3_first(1))+(P_leg4(1)-leg4_first(1)))/4 ;
//            odom_position(2) = ( P_leg1(2)+P_leg2(2)+P_leg3(2)+P_leg4(2))/4;

//            P_feet_1 << 0.42, 0.075, 0.0;
//            P_feet_2 << 0.42, -0.075, 0.0;
//            P_feet_3 << -0.42, 0.075, 0.0;
//            P_feet_4 << -0.42, -0.075, 0.0;
//        }else{
//            ROS_ERROR("get wrong init state:!");
//        }
//        if(P_init_time > 10){
//            P_init_flag = false;
//        }
//    }
//    else{
//        if(foot_flag == 10000)
//        {
//            //test
//            if((foot_flag == first_foot_flag) && (no_data_input_flag)){
//                ROS_INFO("no data---init flag: 0000");
//            }
//            else if((foot_flag != first_foot_flag ) && (no_data_input_flag) ){
//                ROS_INFO("last get data---now no data(first): 0000");
//                no_data_input_flag = false;
//            }
//            else if((foot_flag == first_foot_flag) && (!no_data_input_flag)){
//                ROS_INFO("before get data---again no data(again+++): 0000");
//            }
//            else if((foot_flag != first_foot_flag) && (!no_data_input_flag)){
//                ROS_INFO("before get data---again no data(second+++): 0000");
//            }
//            odom_position = odom_position;
//        }
//        else if(foot_flag == 1111)
//        {
//            if(first_foot_flag != 1111){
//                ROS_INFO("first/again inter flag: 1111");
//                P_feet_1 = P_feet_1 + cycle_T*tmp22;
//                P_feet_2 = P_feet_2 + cycle_T*tmp22;
//                P_feet_3 = P_feet_3 + cycle_T*tmp22;
//                P_feet_4 = P_feet_4 + cycle_T*tmp22;
//            }else{
//                ROS_INFO("inter flag: 1111");
//            }
//            odom_position_tmp = ( (P_BW[0].vector()-P_feet_1)+(P_BW[1].vector()-P_feet_2)+(P_BW[2].vector()-P_feet_3)+(P_BW[3].vector()-P_feet_4) )/4 ;
//            odom_position = odom_position_tmp;
//        }
//        else if(foot_flag == 1010)
//        {
//            if( (first_foot_flag != foot_flag) && first_foot_flag != 1111 ){
//                ROS_INFO("again step inter flag: 1010");
//                cycle_T = cycle_T + 1;
//                P_feet_1 = P_feet_1 + cycle_T*tmp22;
//                P_feet_2 = P_feet_2;
//                P_feet_3 = P_feet_3 + cycle_T*tmp22;
//                P_feet_4 = P_feet_4;
//            }else{
//                ROS_INFO("inter flag: 1010");
//            }
//            odom_position_tmp = ( (P_BW[0].vector()-P_feet_1)+(P_BW[2].vector()-P_feet_3))/2 ;
//            odom_position = odom_position_tmp;
//        }
//        else if(foot_flag == 10101)
//        {

//            if( (first_foot_flag != foot_flag)){
//                ROS_INFO("first inter flag: 10101");
//                cycle_T = cycle_T + 1;
//                P_feet_2 = P_feet_2 + cycle_T*tmp22;
//                P_feet_4 = P_feet_4 + cycle_T*tmp22;
//            }else{
//                ROS_INFO("inter flag: 10101");
//            }
//            odom_position_tmp = -( (P_BW[1].vector()-P_feet_2)+(P_BW[3].vector()-P_feet_4))/2 ;
//            odom_position += odom_position_tmp;
//        }

//        //cal
//        if(cycle_T == 0){
//            cycle_step = 0;
//        }else{
//            cycle_step = (cycle_T-1) /2 +1;
//        }
//    }
}


void QuadrupedEstimation::GetPositionParms() {
//    ROS_WARN("GetPositionParms...");

    if(foot_state_T < 2){
        flag_tmp.push(foot_flag);
        ++foot_state_T;
    }else{
        flag_tmp.pop();
        flag_tmp.push(foot_flag);
    }
    first_foot_flag = flag_tmp.front();
//    cout <<"front flag: " << first_foot_flag <<endl;


    //foot_pose_in world
    GetFootInWorld();
//    ROS_INFO("GetFootInWorld");


    ///////////////
    //queue leg1
    if(leg1_T < 2){
        leg1w_tmp.push(P_BW[0]);
        ++ leg1_T;
    }else {
        leg1w_tmp.pop();
        leg1w_tmp.push(P_BW[0]);
    }
    leg1_first = leg1w_tmp.front();
//    cout << "leg1_front: "<<leg1_first <<endl;
    //queue leg2
    if(leg2_T < 2){
        leg2w_tmp.push(P_BW[1]);
        ++ leg2_T;
    }else {
        leg2w_tmp.pop();
        leg2w_tmp.push(P_BW[1]);
    }
    leg2_first = leg2w_tmp.front();
//    cout << "leg2_front: "<<leg2_first <<endl;
    //queue leg3
    if(leg3_T < 2){
        leg3w_tmp.push(P_BW[2]);
        ++ leg3_T;
    }else {
        leg3w_tmp.pop();
        leg3w_tmp.push(P_BW[2]);
    }
    leg3_first = leg3w_tmp.front();
//    cout << "leg3_front: "<<leg3_first <<endl;
    //queue leg4
    if(leg4_T < 2){
        leg4w_tmp.push(P_BW[3]);
        ++ leg4_T;
    }else {
        leg4w_tmp.pop();
        leg4w_tmp.push(P_BW[3]);
    }
    leg4_first = leg4w_tmp.front();
//    cout << "leg4_front: "<<leg4_first <<endl;


}


void QuadrupedEstimation::GetFootInWorld() {

// 2 ways to get!!!
//*********************************************************//
    //p_ = q*p*q^(-1)
//    odom2init_orientation = odom2init_orientation.normalized();
//    for(int i=0;i<4;++i){
//        Eigen::Vector3d pose_rotated;
//        Eigen::Vector3d tmp_p;//position->vector
//        tmp_p << P_DH[i].vector();//key!!!
//        pose_rotated = odom2init_orientation*tmp_p;// q*p*q^(-1)!!!
//        P_BW[i] << pose_rotated;
////        cout << "after trans: " << pose_rotated.transpose()<<endl;
////        cout << "leg: "<< i << "  P_BW: " <<P_BW[i]<<endl;
//    }


//*********************************************************//
//    rotation_matrix
    Eigen::Matrix3d rotation_matrix_trans;
    Eigen::Vector3d tmp_p, leg_w_single;
//    use baselink_to_initodom

//    odom2init_orientation = odom2init_orientation.normalized();//unit q
//    rotation_matrix_trans = odom2init_orientation.toRotationMatrix();

    RPY_bw_q = RPY_bw_q.normalized();
    rotation_matrix_trans = RPY_bw_q.toRotationMatrix();

    //    just use odom_orientation
//    odom2init_orientation = odom_orientation.normalized();//unit q
//    rotation_matrix_trans = odom_orientation.toRotationMatrix();

    for(int k=0; k<4; ++k){
//        cout << "P_DH[k]: "<<endl<< P_DH[k] <<endl;
        tmp_p << P_DH[k].vector();
//        cout << "tmp_p: "<<endl<< tmp_p <<endl;
        leg_w_single = rotation_matrix_trans*tmp_p;
//        cout << "leg-" << k <<"-in world: " <<endl<< leg_w_single <<endl;
        P_BW[k] << leg_w_single;
//        cout << "legPosition_in_world: "<< k << "---" <<P_BW[k] <<endl;

    }

}

void QuadrupedEstimation::GetFootPoseInBase(free_gait::JointPositions& jointsall){
//    ROS_WARN("GetFootPoseInBase...");
    //    test for
//    ThetaInPI(jointsall);

    // just for in case
    if(jointsall(2) ==0 && jointsall(5) ==0 && jointsall(8) ==0 && jointsall(11) ==0) {
        ROS_INFO("***Joints_init---no msg");
//        jointsall.setZero(12);
//        jointsleg_[0] << 0,1.57,-3.14;
//        jointsleg_[1] << 0,-1.57,3.14;
//        jointsleg_[2] << 0,-1.57,3.14;
//        jointsleg_[3] << 0,1.57,-3.14;
    } else {
        ROS_INFO("********Joints_get_data************");
    }

    jointsleg_[0] << jointsall(0),jointsall(1),jointsall(2);
    jointsleg_[1] << jointsall(3),-jointsall(4),-jointsall(5);
    jointsleg_[2] << jointsall(6),-jointsall(7),-jointsall(8);
    jointsleg_[3] << jointsall(9),jointsall(10),jointsall(11);

//    for(int i=0;i<12;++i){
//        cout <<"jointsall_(i): " << i << jointsall(i) << endl;
//    }


//    from shunyao---get 4 footPoseInBaseFrame_
    T_DH[0] = robot_state_->getPoseFootInBaseFrame(free_gait::LimbEnum::LF_LEG, jointsleg_[0]);
    T_DH[1] = robot_state_->getPoseFootInBaseFrame(free_gait::LimbEnum::RF_LEG, jointsleg_[1]);
    T_DH[2] = robot_state_->getPoseFootInBaseFrame(free_gait::LimbEnum::RH_LEG, jointsleg_[2]);
    T_DH[3] = robot_state_->getPoseFootInBaseFrame(free_gait::LimbEnum::LH_LEG, jointsleg_[3]);

    for(int i=0;i<4;++i){
        P_DH[i] = T_DH[i].getPosition();
        R_Q[i] = T_DH[i].getRotation().toUnitQuaternion();


//        cout << "T_DH[i].getRotation(): " << T_DH[i].getRotation() <<
//                "T_DH[i].getQuaternionMatrix(): "<< T_DH[i].getRotation().getQuaternionMatrix() <<
//                "T_DH[i].toUnitQuaternion(): "<< T_DH[i].getRotation().toUnitQuaternion() << endl;
//        cout << "leg: "<< i << "T_DH: " << endl << T_DH[i] << endl <<
//                " foot_orientation_in base:  "<< R_Q[i] << endl <<
//                " foot_pose_in base:  "<< P_DH[i] << endl;

    }
}

void QuadrupedEstimation::GetFootPoseInBase() {
//    ROS_WARN("GetFootPoseInBase...");

    if(joints_output.position.at(0) ==0) {
        ROS_WARN("***Joints_init---no msg");
    }
    /*else {
        ROS_WARN("********Joints_get_data************");
    }*/
    //    from shunyao---get 4 footPoseInBaseFrame_
    T_DH[0] = robot_state_->getPoseFootInBaseFrame(free_gait::LimbEnum::LF_LEG);
    T_DH[1] = robot_state_->getPoseFootInBaseFrame(free_gait::LimbEnum::RF_LEG);
    T_DH[2] = robot_state_->getPoseFootInBaseFrame(free_gait::LimbEnum::RH_LEG);
    T_DH[3] = robot_state_->getPoseFootInBaseFrame(free_gait::LimbEnum::LH_LEG);
    for(int i=0;i<4;++i){
        P_DH[i] = T_DH[i].getPosition();
//        cout << " foot_pose_in base:  "<< P_DH[i] << endl ;
        R_Q[i] = T_DH[i].getRotation().toUnitQuaternion();
//        cout << " foot_orientation_in base:  "<< R_Q[i] << endl ;
//        Eigen::Vector3d YPR_ =  QuaterniondToRPY(R_Q[i]);
        YPRfromQ[i] << QuaterniondToRPY(R_Q[i]);
//        cout << " foot_rpy_in base:  "<< YPRfromQ[i] << endl ;
//        cout << "leg: "<< i << "T_DH: " << endl << T_DH[i] << endl <<
//        cout << " foot_orientation_in base:  "<< R_Q[i] << endl <<
//        cout << " foot_pose_in base:  "<< P_DH[i] << endl;
    }


}

Eigen::Vector3d QuadrupedEstimation::QuaterniondToRPY(RotationQuaternion imu_rq){

    double imu_0_yaw, imu_0_pitch, imu_0_roll;
    Eigen::Vector3d rpy_vec_;
    imu_rq = imu_rq.toUnitQuaternion();
    tf::Matrix3x3 mat_be(tf::Quaternion(imu_rq.x(),imu_rq.y(),imu_rq.z(),imu_rq.w()));
    mat_be.getEulerYPR(imu_0_yaw,imu_0_pitch,imu_0_roll);
    rpy_vec_ << imu_0_roll,imu_0_pitch,imu_0_yaw;
//    cout << "GetFootPoseInBase---YPR: " << endl << "yaw: "<< rpy_vec_(0) << " pitch: " << rpy_vec_(1) << " roll: " <<rpy_vec_(2) << endl;
    return rpy_vec_;

}

void QuadrupedEstimation::FootstateJudge(const std_msgs::Float64MultiArray foot_out_){
//    ROS_WARN("Judge FootState...");
    if((foot_out_.data[0] == 1) && (foot_out_.data[1] == 1) && (foot_out_.data[2] == 1) && (foot_out_.data[3] == 1)) {
        foot_flag = 1111;
    }
    else if((foot_out_.data[0] == 0) && (foot_out_.data[1] == 1) && (foot_out_.data[2] == 1) && (foot_out_.data[3] == 1)) {
        foot_flag = 10111;
    }
    else if((foot_out_.data[0] == 1) && (foot_out_.data[1] == 0) && (foot_out_.data[2] == 1) && (foot_out_.data[3] == 1)) {
        foot_flag = 1011;
    }
    else if((foot_out_.data[0] == 1) && (foot_out_.data[1] == 1) && (foot_out_.data[2] == 0) && (foot_out_.data[3] == 1)) {
        foot_flag = 1101;
    }
    else if((foot_out_.data[0] == 1) && (foot_out_.data[1] == 1) && (foot_out_.data[2] == 1) && (foot_out_.data[3] == 0)) {
        foot_flag = 1110;
    }
    else if((foot_out_.data[0] == 1) && (foot_out_.data[1] == 0) && (foot_out_.data[2] == 1) && (foot_out_.data[3] == 0)) {
        foot_flag = 1010;
    }
    else if((foot_out_.data[0] == 0) && (foot_out_.data[1] == 1) && (foot_out_.data[2] == 0) && (foot_out_.data[3] == 1)) {
        foot_flag = 10101;
    }

    else if((foot_out_.data[0] == 0) && (foot_out_.data[1] == 0) && (foot_out_.data[2] == 0) && (foot_out_.data[3] == 0)) {
        foot_flag = 10000;
//        ROS_WARN("no bumper check!!");
    }

//    cout << " foot_flag : "<< foot_flag << endl;
}




///**
// * @brief QuadrupedEstimation::QuadrupedVel
// */
void QuadrupedEstimation::QuadrupedVel(){
//    cout <<"cal_vel_way:   " << _cal_vel_way  << endl;

    if( _cal_vel_way == "dpdtvel") {
        GetLinearVelFromKin();
    }
    else if( _cal_vel_way == "jacobvel" ) {
        GetLinearVelFromJointvel();
    }
    else if( _cal_vel_way == "imuang") {
        GetLinearVelFromIMUAng();
    }
    else if( _cal_vel_way == "imuacc") {
        ros::Time time_;
        time_ = ros::Time::now();
        GetLinearVelFromIMUAcc(time_);
    }

}

void QuadrupedEstimation::GetLinearVelFromKin() {
    ROS_WARN("GetLinearVelFromKin");
    Position tmp_vel_, odom_vel_p;
    Eigen::Vector3d odom_p_before,odom_p_vec,odom_v_vec;
    odom_p_vec << odom_position(0),odom_position(1),odom_position(2);
    ros::Time time_v, time_before;
    ros::Duration time_duartion;
    time_v = ros::Time::now();
    if( vel_kin_T_ < _vel_set_T){
        odom_vel << 0.0, 0.0, 0.0;
//        vel_tmp.push(odom_position);
        vel_info_all.push_back(make_pair(time_v,odom_p_vec));
        ++vel_kin_T_;
        ROS_INFO("vel_kin_T_ <<< vel_set_T");
    } else {
//        vel_tmp.pop();
//        vel_tmp.push(odom_position);
//        tmp_vel_ = vel_tmp.front();
        vel_info_all.pop_front();
        vel_info_all.push_back(make_pair(time_v,odom_p_vec));
        odom_p_before = vel_info_all.front().second;
        time_before = vel_info_all.front().first;
        time_duartion = time_v - time_before;
        double t = time_duartion.toSec();
        if(t == 0.0){
            t=1;
            ROS_ERROR("get no data,t_duration=0 !!!");
        }
        odom_v_vec = (odom_p_vec - odom_p_before) /t;


        odom_vel = 2.5*GetLinearVelFilter(odom_v_vec);

//        if(velget_flag && !foot_cb_flag){
//            odom_vel << 0.0, 0.0, 0.0;
//            ROS_ERROR("not get bag data,so v=0!!!");

//        }
//        else if(!velget_flag  || odom_p_before(0)==0){
//           velget_flag = true;
//           odom_vel << 0.0, 0.0, 0.0;
//           ++velfromkin_;
//           ROS_ERROR("frist get bag data,so v=0!!!");
//        }else{
////            odom_vel_p = (odom_position - tmp_vel_) /( _vel_set_T * _odom_dt);
////            odom_vel << odom_vel_p.vector();
//            time_duartion = time_v - time_before;
//            double t = time_duartion.toSec();
//            if(t == 0.0){
//                t=1;
//                ROS_ERROR("get no data,t_duration=0 !!!");
//            }
//            odom_v_vec = (odom_p_vec - odom_p_before) /t;
//            odom_vel << odom_v_vec;
//        }



    }
//    cout << "time_v: "<< time_v << endl <<
//         "time_before: "<< time_before << endl <<
//         "time_duartion: "<<  time_duartion.toSec() << endl;
//    cout << "vel_kin_Time: " << vel_kin_T_ << endl;
    cout << "odom_vel: " << odom_vel << endl;
}

LinearVelocity QuadrupedEstimation::GetLinearVelFilter(Eigen::Vector3d& v_filter) {
//    vel_filter_buf,vel_filter_vec;
//    ROS_ERROR("GetLinearVelFilter");
    Eigen::Vector3d sum_;
    sum_.setZero();
    vel_filter_buf.push_back(v_filter);
    if(vel_filter_buf.size()>10){
        vel_filter_buf.pop_front();
//        vel_filter_vec.push_back(vel_filter_buf.front());
        vel_filter_vec = vel_filter_buf;
        vector<Eigen::Vector3d> vec_tmp;
        for(int i=0;i<10;++i){
            vec_tmp.push_back(vel_filter_vec.front());
            sum_ += vel_filter_vec.front();
            vel_filter_vec.pop_front();
        }
    }

    return LinearVelocity(sum_/10);
}


void QuadrupedEstimation::GetLinearVelFromKinforTwoQueue(){
    ROS_WARN("GetLinearVelFromKinforTwoQueue");
    Position tmp_vel_, odom_vel_p;
    Eigen::Vector3d odom_p_before,odom_p_vec,odom_v_vec;
    odom_p_vec << odom_position(0),odom_position(1),odom_position(2);
    ros::Time time_v, time_before;
    ros::Duration time_duartion;
    time_v = ros::Time::now();
    if( vel_kin_T_ < _vel_set_T){
        odom_vel << 0.0, 0.0, 0.0;
//        vel_tmp.push(odom_position);
        vel_info_all.push_back(make_pair(time_v,odom_p_vec));
        ++vel_kin_T_;
        ROS_INFO("vel_kin_T_ <<< vel_set_T");
    } else {


    }
}

void QuadrupedEstimation::GetLinearVelFromJointvel() {
//    ROS_WARN("GetLinearVelFromJointvel");
//    if(joints_output.velocity.at(0) ==0) {
//        ROS_WARN("***Joints_init---no msg");
//    }
//    else {
//        ROS_WARN("********Joints_get_data************");
//    }

    //form shunyao
    V_Jacob[0] = robot_state_->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LF_LEG);
    V_Jacob[1] = robot_state_->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RF_LEG);
    V_Jacob[2] = robot_state_->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RH_LEG);
    V_Jacob[3] = robot_state_->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LH_LEG);

    Eigen::Vector3d min_vector_error;
    min_vector_error << 100,100,100;
    int min_index_a, min_index_b;
    std::vector<int> foots;
    foots.resize(4);
    for(int i = 0;i<4;i++)
      {
        for(int j = 0;j<4;j++)
          {
            Eigen::Vector3d vector_error = V_Jacob[i].cross(V_Jacob[j]).vector();
            if(vector_error.norm() < min_vector_error.norm())
              {
                min_vector_error = vector_error;
                min_index_a = i;
                min_index_b = j;
              }
          }
        if(foot_output.data[i] == 1 && V_Jacob[i](0)<1 && V_Jacob[i](1)<1 && V_Jacob[i](2)<1)
          foots[i] = 1;
        else
          foots[i] = 0;
      }


//    for(int i=0;i<4;++i){
//        cout << "V_Jacob[i]:" << i << " " <<V_Jacob[i] << endl;
//    }

//    if(foot_flag == 1111) {
////        odom_vel_inodom =  -(V_Jacob[0] + V_Jacob[1] + V_Jacob[2] + V_Jacob[3] )/4  ;
//        odom_vel_inodom_1 = -(V_Jacob[1] + V_Jacob[3] ) * 1/2;
//        odom_vel_inodom_2 = -(V_Jacob[0]  + V_Jacob[2] ) * 1/2;
//        odom_vel_inodom = odom_vel_inodom_1.norm() > odom_vel_inodom_2.norm() ? odom_vel_inodom_2 : odom_vel_inodom_1;

////        odom_vel_inodom = std::min(abs(odom_vel_inodom_1,odom_vel_inodom_2));

//    }
//    else if(foot_flag == 1110) {
//        odom_vel_inodom =  -(V_Jacob[0] + V_Jacob[1] + V_Jacob[2] ) * 1/3;
//    }
//    else if(foot_flag == 1101) {
//        odom_vel_inodom =  -(V_Jacob[0] + V_Jacob[1] + V_Jacob[3] ) * 1/3;
//    }
//    else if(foot_flag == 1011) {
//        odom_vel_inodom =  -(V_Jacob[0] + V_Jacob[2] + V_Jacob[3] ) * 1/3;
//    }
//    else if(foot_flag == 10111) {
//        odom_vel_inodom =  -( V_Jacob[1] + V_Jacob[2] + V_Jacob[3] ) * 1/3;
//    }
//    else if(foot_flag == 10101) {
//        odom_vel_inodom =  -(V_Jacob[1] + V_Jacob[3] ) * 1/2;
//    }
//    else if(foot_flag == 1010) {
//        odom_vel_inodom =  -(V_Jacob[0]  + V_Jacob[2] ) * 1/2;
//    }
//    odom_vel_inodom = -0.5*(V_Jacob[min_index_a] + V_Jacob[min_index_b]);
    int number_of_contact = foots[0] + foots[1] + foots[2] + foots[3];
    if(number_of_contact > 0)
      {
        odom_vel_inodom = -(foots[0] * V_Jacob[0] + foots[1] * V_Jacob[1] + foots[2] * V_Jacob[2] + foots[3] * V_Jacob[3])
            /number_of_contact;
    //    cout << "odom_vel_inodom: "<< odom_vel_inodom << endl;
        LinearVelocity imu_ang_vel = GetLinearVelFromIMUAng();
        odom_vel_inodom = odom_vel_inodom + imu_ang_vel;
      }


    GetVelInWorld(odom_vel_inodom);


    Eigen::Vector3d odom_tmp_v;

//    geometry_msgs::Twist imu_vel_msg;
//    imu_vel_msg.linear.x = imu_ang_vel(0);
//    imu_vel_msg.linear.y = imu_ang_vel(1);
//    imu_vel_msg.linear.z = imu_ang_vel(2);
//    imuvel_pub.publish(imu_vel_msg);

//    odom_tmp_v << odom_vel(0)+imu_ang_vel(0), odom_vel(1)+imu_ang_vel(1), odom_vel(2)+imu_ang_vel(2);
    odom_tmp_v << odom_vel(0), odom_vel(1), odom_vel(2);

    odom_vel = GetLinearVelFilter(odom_tmp_v);
}

void QuadrupedEstimation::GetVelInWorld(LinearVelocity odomvel_odom){
    //rotation_matrix
    Eigen::Matrix3d rotation_matrix_trans;
//    use baselink_to_initodom
//    odom2init_orientation = odom2init_orientation.normalized();//unit q
//    rotation_matrix_trans = odom2init_orientation.toRotationMatrix();

//    RPY_bw_q = RPY_bw_q.normalized();
    rotation_matrix_trans = RPY_bw_q.toRotationMatrix();
//    cout << "rotation_matrix_trans: " << endl<< rotation_matrix_trans << endl;
    Eigen::Vector3d odom_vel_vec,legodom_vel_vec;
    odom_vel_vec  << odomvel_odom.vector();
    legodom_vel_vec = rotation_matrix_trans * odom_vel_vec;
    odom_vel << legodom_vel_vec ;
//    cout  << "odom_vel: " << odom_vel <<endl;

}

Eigen::Vector3d QuadrupedEstimation::GetVelInOdom(LinearVelocity odomvel_world){

    Eigen::Matrix3d rotation_matrix_trans;
//    use baselink_to_initodom
    odom2init_orientation = odom2init_orientation.normalized();//unit q
    rotation_matrix_trans = odom2init_orientation.toRotationMatrix();
//    cout << "rotation_matrix_trans: " << endl<< rotation_matrix_trans << endl;
    Eigen::Vector3d odom_vel_vec,legodom_vel_vec;
    odom_vel_vec  << odomvel_world.vector();
    legodom_vel_vec = rotation_matrix_trans.inverse() * odom_vel_vec;
    return legodom_vel_vec;
}

void QuadrupedEstimation::GetLinearVelFromIMUAcc(ros::Time current_time){
    static ros::Time previous_time = current_time;
    double diff_time = (current_time - previous_time).toSec();

    Eigen::Vector3d ImuAcc;
    ImuAcc(0) = imu_output.linear_acceleration.x;
    ImuAcc(1) = imu_output.linear_acceleration.y;
    ImuAcc(2) = imu_output.linear_acceleration.z;

    odom_vel.x() += ImuAcc(0)*diff_time;
    odom_vel.y() += ImuAcc(1)*diff_time;
    odom_vel.z() += ImuAcc(2)*diff_time;

    previous_time = current_time;
}

LinearVelocity QuadrupedEstimation::GetLinearVelFromIMUAng(){

    Eigen::Vector3d ImuAng,ImuAngInWorld, foot13,foot24,odom_,net_vector,imu_vel;
    ImuAng(0) = imu_output.angular_velocity.x;
    ImuAng(1) = imu_output.angular_velocity.y;
    ImuAng(2) = imu_output.angular_velocity.z;

//    ImuAngInWorld = robot_state_->getOrientationBaseToWorld().rotate(ImuAng);



    int num_of_contacts = foot_output.data[0]+foot_output.data[1]+foot_output.data[2]+foot_output.data[3];
    if(num_of_contacts<=2 && num_of_contacts>0)
      {
        net_vector = (foot_output.data[0] * robot_state_->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::LF_LEG).vector()
            + foot_output.data[1] * robot_state_->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::RF_LEG).vector()
            + foot_output.data[2] * robot_state_->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::RH_LEG).vector()
            + foot_output.data[3] * robot_state_->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::LH_LEG).vector())
            /num_of_contacts;
        imu_vel = -ImuAng.cross(net_vector);
      }else {
        imu_vel << 0,0,0;
      }



//    if(foot_flag == 1111) {
////        odom_vel_inodom =  -(V_Jacob[0] + V_Jacob[1] + V_Jacob[2] + V_Jacob[3] )/4  ;
//        odom_ << 0,0,0;

//    }
//    else if(foot_flag == 10101) {
//        foot13 = (robot_state_->getPositionWorldToFootInWorldFrame(free_gait::LimbEnum::LF_LEG).vector()
//                + robot_state_->getPositionWorldToFootInWorldFrame(free_gait::LimbEnum::RH_LEG).vector()) * 1/2;
//        odom_ = - ( ImuAng.cross(foot13));

//    }
//    else if(foot_flag == 1010) {
//        foot24 = (robot_state_->getPositionWorldToFootInWorldFrame(free_gait::LimbEnum::LH_LEG).vector()
//                + robot_state_->getPositionWorldToFootInWorldFrame(free_gait::LimbEnum::RF_LEG).vector()) * 1/2;

//        odom_ = - ( ImuAng.cross(foot24));

//    }
    return LinearVelocity(imu_vel);
}

///**
// * @brief QuadrupedEstimation::QuadrupedOrientation
// */
void QuadrupedEstimation::QuadrupedOrientation() {
//    cout << "_orientation_way; " << _orientation_way << endl;

    if( _orientation_way == "imu_way"){
        GetQFromImu(imu_output);
    }
    else if( _orientation_way == "kin_way") {
        GetQFromKin();
    }

}

void QuadrupedEstimation::QuadrupedOrientation(sensor_msgs::Imu& imu_in_) {
    cout << "_orientation_way; " << _orientation_way << endl;

    if( _orientation_way == "imu_way"){
        GetQFromImu(imu_in_);
    }
    else if( _orientation_way == "kin_way") {
        GetQFromKin();
    }

}

void QuadrupedEstimation::GetQFromImu(const sensor_msgs::Imu& imu_output) {
//    ROS_WARN("GetQFromKin...");
    Eigen::Quaterniond imu_o;
    imu_o.x() = imu_output.orientation.x;
    imu_o.y() = imu_output.orientation.y;
    imu_o.z() = imu_output.orientation.z;
    imu_o.w() = imu_output.orientation.w;

//    cout<< "imu_output: "<< imu_o.coeffs() << endl;
//    四元输不能相加!!!//切记SO3是矩阵，是李群
    imu_o.normalized();
    Sophus::SO3 SO3_q(imu_o);
//    Eigen::MatrixX3d rota_ma;
//    rota_ma = imu_o.toRotationMatrix();

//    Eigen::Vector3d  q2rpy = QuaterniondToRPY(imu_o);
//    cout << "before: " << "imu_yaw: " <<q2rpy(0) << "   imu_pitch: " <<q2rpy(1) << " imu_roll: " << q2rpy(2)<< endl;

    Eigen::Vector3d update_so3(1e-10, 1e-10, 1e-10); //假设更新量为这么多
    Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3)*SO3_q;
    RPY_bw_q = Eigen::Quaterniond(SO3_updated.matrix());
//    cout << "RPY_bw_q: " <<RPY_bw_q.coeffs() << endl;
//    odom_orientation.vector() <<  RPY_bw_q.w(),RPY_bw_q.x(),RPY_bw_q.y(),RPY_bw_q.z();
    odom_orientation.w() = RPY_bw_q.w();
    odom_orientation.x() = RPY_bw_q.x();
    odom_orientation.y() = RPY_bw_q.y();
    odom_orientation.z() = RPY_bw_q.z();
//    cout<< "odom_orientation: "<< odom_orientation << endl;
//    odom_YPR = QuaterniondToRPY(RPY_bw_q);
//    cout << "after: " <<"imu_yaw: " <<odom_YPR(0) << "   imu_pitch: " <<odom_YPR(1) << "    imu_roll: " << odom_YPR(2)<< endl;

}

Eigen::Vector3d QuadrupedEstimation::QuaterniondToRPY(Eigen::Quaterniond& imu_000){

    double imu_0_yaw, imu_0_pitch, imu_0_roll;
    Eigen::Vector3d rpy_vec;
    tf::Matrix3x3 mat_be(tf::Quaternion(imu_000.x(),imu_000.y(),imu_000.z(),imu_000.w()));
    mat_be.getEulerYPR(imu_0_yaw,imu_0_pitch,imu_0_roll);
    rpy_vec << imu_0_roll,imu_0_pitch,imu_0_yaw;
//    cout << "GetQFromImu----rpy_vec: "<< endl << rpy_vec << endl;
    return rpy_vec;

}

//void QuadrupedEstimation::GetQPlus(const Eigen::VectorXcd& delta) {

//    Eigen::Quaterniond q_updata(imu_output.orientation.w,imu_output.orientation.x,imu_output.orientation.y,imu_output.orientation.z);
//    q_updata = q_updata * Sophus::SO3::exp(Vec3(delta[3], delta[4], delta[5])).unit_quaternion();
//    q_updata.normalized();
//    imu_output.orientation.w = q_updata.w();
//    imu_output.orientation.x = q_updata.x();
//    imu_output.orientation.y = q_updata.y();
//    imu_output.orientation.z = q_updata.z();
//}

void QuadrupedEstimation::GetQFromKin() {
    ROS_WARN("GetQFromKin...");
    Eigen::Vector3d QfromYPR;

    if(foot_flag == 1111) {
        QfromYPR = 1/4 * (YPRfromQ[0] + YPRfromQ[1] + YPRfromQ[2] + YPRfromQ[3] );
    }
    else if(foot_flag == 1110) {
        QfromYPR = 1/3 * (YPRfromQ[0] + YPRfromQ[1] + YPRfromQ[2] );
    }
    else if(foot_flag == 1101) {
        QfromYPR = 1/3 * (YPRfromQ[0] + YPRfromQ[1] + YPRfromQ[3] );
    }
    else if(foot_flag == 1011) {
        QfromYPR = 1/3 * (YPRfromQ[0] + YPRfromQ[2] + YPRfromQ[3] );
    }
    else if(foot_flag == 10111) {
        QfromYPR = 1/3 * (YPRfromQ[1] + YPRfromQ[2] + YPRfromQ[3] );
    }
    else if(foot_flag == 1010) {
        QfromYPR = 1/2 * (YPRfromQ[0] + YPRfromQ[2] );
    }
    else if(foot_flag == 10101) {
        QfromYPR = 1/2 * (YPRfromQ[1] + YPRfromQ[3] );
    }

    cout <<"kin_yaw: " <<QfromYPR(0) << "   kin_pitch: " <<QfromYPR(1) << "    kin_roll: " << QfromYPR(2)<< endl;
    odom_orientation = RPYTOQuaterniond(QfromYPR);
    cout << "odom_orientation: " << odom_orientation << endl;

}

RotationQuaternion QuadrupedEstimation::RPYTOQuaterniond(Eigen::Vector3d q_rpy){
    RotationQuaternion rpy2q;
    Eigen::AngleAxisd rollAngle(q_rpy[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(q_rpy[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(q_rpy[2], Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond rpytoq_ = yawAngle * pitchAngle * rollAngle;
//    cout << "rpytoq_: " << endl << rpytoq_.coeffs() << endl;
    QuaterniondToRPY(rpytoq_);
    rpy2q.vector() << rpytoq_.w(),rpytoq_.x(),rpytoq_.y(),rpytoq_.z();
    return rpy2q;
}

void QuadrupedEstimation::ThetaInPI(quadruped_model::JointPositions &jointsall){

    for(int i=0; i< 12; ++i){
//        cout << "jointsall(i): " << jointsall(i)<<endl;
//        cout << "M_PI: " << M_PI<<endl;
        if(jointsall(i) > M_PI){

            jointsall(i) = (M_PI - jointsall(i));
            cout << "jointsall(i): " << jointsall(i)<<endl;
        }
    }
}

Eigen::VectorXd QuadrupedEstimation::GetStopStateOdom(){
    Eigen::VectorXd LegOdomState;
//    LegOdomState.resize(10);
    LegOdomState = Eigen::VectorXd(10,1);
    LegOdomState.segment(0,3) << odom_position.vector();
    LegOdomState.segment(3,3) = odom_vel.vector();
    LegOdomState.segment(6,4) = odom_orientation.vector();
//    cout << "LegOdomState" << endl << LegOdomState.transpose() << endl;
    return LegOdomState;
}



}
