#include "single_leg_test/model_test_header.hpp"
//#include "assert.h"
#include <ros/package.h>

MyRobotSolver::MyRobotSolver(const ros::NodeHandle& node_handle,
                             std::shared_ptr<free_gait::State> robot_state)
  : length_of_data(10001),
    Time_derta(0.001),
    num_of_joints(3),
    calculation_iterstions(0),
    node_handle_(node_handle),
    robot_state_(robot_state)
{
  QPlanned.resize(length_of_data,3);
  QDotPlanned.resize(length_of_data,3);
  QDDotPlanned.resize(length_of_data,3);
  QAcutal.resize(length_of_data,3);
  QDotAcutal.resize(length_of_data,3);
  QDDotAcutal.resize(length_of_data,3);
  TauofIDynamics.resize(length_of_data,3);
  Tauacutal.resize(length_of_data,3);

  VecQAct.resize(3);
  VecQDotAct.resize(3);
  VecQDDotAct.resize(3);
  VecTauAct.resize(3);
  VecTauerror.resize(3);
  VecQerror.resize(3);
  VecQDoterror.resize(3);
  TauFeedForward.resize(3);
  std::queue<VectorNd> vector_queue;
  for(int i=0;i<4;i++)
    {
      free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
      QActQueueLimb[limb] = vector_queue;
      QDotActQueueLimb[limb] = vector_queue;
      LimbRBDLModel[limb] = new Model();
    }
//  joint_state_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/inverse_dynamic_joint_state", 1);
}

MyRobotSolver::~MyRobotSolver()
{
  std::cout<<"MyRobotSolver Destroied"<<std::endl;
}

Model& MyRobotSolver::getModel()
{
  return QuadrupedRobotModel;
}

const MatrixNd& MyRobotSolver::getTau()
{
  return TauofIDynamics;
}

const unsigned int& MyRobotSolver::getnum_of_joints()
{
  return num_of_joints;
}

const double& MyRobotSolver::getTime_derta()
{
  return Time_derta;
}

const MatrixNd& MyRobotSolver::getQDotPlanned()
{
  return QDotPlanned;
}

const MatrixNd& MyRobotSolver::getQPlanned()
{
  return QPlanned;
}

const unsigned int& MyRobotSolver::getlength_of_data()
{
  return length_of_data;
}

const unsigned int& MyRobotSolver::getcalculation_iterations()
{
  return calculation_iterstions;
}

const VectorNd& MyRobotSolver::getVecTauAct()
{
  return VecTauAct;
}

const VectorNd& MyRobotSolver::getVecQAct()
{
  return VecQAct;
}

const VectorNd& MyRobotSolver::getVecQDAct()
{
  return VecQDotAct;
}

const VectorNd& MyRobotSolver::getVecQDDAct()
{
  return VecQDDotAct;
}

void MyRobotSolver::setvecQAct(const Eigen::Vector3d& joint_positions, const free_gait::LimbEnum& limb)
{
//  VecQAct = joint_positions;
  QActQueueLimb.at(limb).push(VecQAct);
  if(QActQueueLimb.at(limb).size()>11)
    QActQueueLimb.at(limb).pop();
}

void MyRobotSolver::setvecQDotAct(const Eigen::Vector3d& joint_velocities, const free_gait::LimbEnum& limb)
{
  QDotActQueueLimb.at(limb).push(joint_velocities);
  if(QDotActQueueLimb.at(limb).size()>11)
    QDotActQueueLimb.at(limb).pop();
}

void MyRobotSolver::setQAcutal(const VectorNd Rowszero)
{
  QAcutal.row(0) = Rowszero.transpose();
}

void MyRobotSolver::setQDotAcutal(const VectorNd Rowszero){
  QDotAcutal.row(0) = Rowszero.transpose();
}

const MatrixNd& MyRobotSolver::getQAcutal()
{
  return QAcutal;
}
const MatrixNd& MyRobotSolver::getQDotAcutal()
{
  return QDotAcutal;
}
const MatrixNd& MyRobotSolver::getQDDotAcutal()
{
  return QDDotAcutal;
}

const MatrixNd& MyRobotSolver::getTauAcutal()
{
  return Tauacutal;
}

void MyRobotSolver::GetPlannedTorque()
{
  ifstream OpenPlannedtorque;

  OpenPlannedtorque.open("/home/kun/catkin_ws/src/single_leg_test/DataFloder/TauofInversedynamics.txt");
  if (!OpenPlannedtorque.is_open())
  {
      cout << "could not open the TauofInversedynamixcs file" << endl;
      cout << "Program terminating" << endl;
      exit(EXIT_FAILURE);
  }
  unsigned int i = 0;
  for (int i = 0; i < length_of_data; ++i)
  {
      OpenPlannedtorque >> TauofIDynamics(i,0) >> TauofIDynamics(i,1) >> TauofIDynamics(i,2);
  }
  cout << "finish the GetplannedTorque"<<endl;
  OpenPlannedtorque.close();
}

void MyRobotSolver::FileStoreIntoTextFile(const char* filestoredlocation, const MatrixNd& Stored_data)
{
  ofstream fout;
  fout.open(filestoredlocation);
  if (!fout.is_open())
  {
      cout << "could not open the data_generate.txt file" << endl;
      cout << "Program terminating" << endl;
      exit(EXIT_FAILURE);
  }
  for (int i = 0; i < length_of_data; ++i) {
    fout << Stored_data(i,0) << "\t" << Stored_data(i,1) << "\t" << Stored_data(i,2) << endl;
  }
  fout.close();
  cout << "Finish the" << filestoredlocation <<" Data stored" << endl;
}

void MyRobotSolver::model_initialization()
{
    rbdl_check_api_version (RBDL_API_VERSION);

    unsigned int body_a_id, body_b_id, body_c_id;
    Body body_a, body_b, body_c;
    Joint joint_a, joint_b, joint_c;

    QuadrupedRobotModel.gravity = Vector3d (0., 0., -9.81);

    body_a = Body (1.17, Vector3d (0., 0.0128, 0.), Matrix3d(0.00172, 0. , 0., 0., 0.00132, 0., 0., 0., 0.00215 ));//Body (const double &mass, const Math::Vector3d &com, const Math::Matrix3d &inertia_C)
    joint_a = Joint(
        JointTypeRevolute,
        Vector3d (0., 0., 1.)
    );//body_a's mass 1.0, center of mass and the inertia at the center of mass/or the radius gradius.

    Matrix3d B0toB1Rotation = roty(M_PI/2);

    body_a_id = QuadrupedRobotModel.AddBody(0, SpatialTransform(B0toB1Rotation,Vector3d(0., 0., 0.)), joint_a, body_a);
    body_b = Body (3.39, Vector3d (0.114, 0., 0.0594), Matrix3d (0.00302, 0., 0., 0., 0.0269, 0., 0., 0., 0.0285));
        joint_b = Joint (
        JointTypeRevolute,
        Vector3d (0., 0., 1.)
    );
    //pay attention to the transform order of the matrix!!!!!!from the right multiply.
    Matrix3d B1toB2Rotation = rotx(-M_PI/2);
    body_b_id = QuadrupedRobotModel.AddBody(body_a_id, SpatialTransform(B1toB2Rotation,Vector3d(0., 0., 0.1)), joint_b, body_b);

    body_c = Body (1.41, Vector3d (0.0949, 0., -0.00166),Matrix3d (0.000547, 0., 0.000222, 0., 0.0109, 0., 0.000222, 0., 0.0111));
        joint_c = Joint (
        JointTypeRevolute,
        Vector3d (0., 0., 1.)
    );

    body_c_id = QuadrupedRobotModel.AddBody(body_b_id, Xtrans(Vector3d(0.25, 0., 0.1)), joint_c, body_c);
    cout << " the DOF is " << QuadrupedRobotModel.dof_count << endl;
    cout << "Finish the model construction" << endl;
}

bool MyRobotSolver::loadLimbModelFromURDF()
{
//  string lf_leg_urdf_dir_str = ros::package::getPath("quadruped_model") + "/urdf/simpledog_lf_leg.urdf";
  string lf_leg_urdf_dir_str = ros::package::getPath("quadruped_model") + "/urdf/quadruped_model_lf_leg.urdf";
  char* lf_leg_urdf_dir = (char*)lf_leg_urdf_dir_str.c_str();
  RigidBodyDynamics::Addons::URDFReadFromFile(lf_leg_urdf_dir, LimbRBDLModel.at(free_gait::LimbEnum::LF_LEG), false, false);
//  std::cout<<" Gravity :"<<LimbRBDLModel.at(free_gait::LimbEnum::LF_LEG)->gravity<<std::endl;
//  VecQAct << 0,1,1;
//  VecQDotAct.setZero();
//  VecQDDotAct.setZero();
//  InverseDynamics(*LimbRBDLModel.at(free_gait::LimbEnum::LF_LEG),VecQAct,VecQDotAct,VecQDDotAct,VecTauAct);
//  std::cout<<" Tau :"<<VecTauAct<<endl;
//  string rf_leg_urdf_dir_str = ros::package::getPath("quadruped_model") + "/urdf/simpledog_rf_leg.urdf";
  string rf_leg_urdf_dir_str = ros::package::getPath("quadruped_model") + "/urdf/quadruped_model_rf_leg.urdf";
  char* rf_leg_urdf_dir = (char*)rf_leg_urdf_dir_str.c_str();
  RigidBodyDynamics::Addons::URDFReadFromFile(rf_leg_urdf_dir, LimbRBDLModel.at(free_gait::LimbEnum::RF_LEG), false, false);

//  string rh_leg_urdf_dir_str = ros::package::getPath("quadruped_model") + "/urdf/simpledog_rh_leg.urdf";
  string rh_leg_urdf_dir_str = ros::package::getPath("quadruped_model") + "/urdf/quadruped_model_rh_leg.urdf";
  char* rh_leg_urdf_dir = (char*)rh_leg_urdf_dir_str.c_str();
  RigidBodyDynamics::Addons::URDFReadFromFile(rh_leg_urdf_dir, LimbRBDLModel.at(free_gait::LimbEnum::RH_LEG), false, false);

//  string lh_leg_urdf_dir_str = ros::package::getPath("quadruped_model") + "/urdf/simpledog_lh_leg.urdf";
  string lh_leg_urdf_dir_str = ros::package::getPath("quadruped_model") + "/urdf/quadruped_model_lh_leg.urdf";
  char* lh_leg_urdf_dir = (char*)lh_leg_urdf_dir_str.c_str();
  RigidBodyDynamics::Addons::URDFReadFromFile(lh_leg_urdf_dir, LimbRBDLModel.at(free_gait::LimbEnum::LH_LEG), false, false);

  return true;
}

void MyRobotSolver::GetLengthofPlannedData()
{

    ifstream OpenPlannedParameters;

    OpenPlannedParameters.open("/home/kun/catkin_ws/src/single_leg_test/DataFloder/PlannedData.txt");
    if (!OpenPlannedParameters.is_open())
    {
        cout << "could not open the PlannedData file" << endl;
        cout << "Program terminating" << endl;
        exit(EXIT_FAILURE);
    }
    unsigned int i = 0;
    for (int i = 0; i < length_of_data; ++i)
    {

        OpenPlannedParameters >> QPlanned(i,0) >> QPlanned(i,1) >> QPlanned(i,2)
                              >> QDotPlanned(i,0) >> QDotPlanned(i,1) >> QDotPlanned(i,2)
                              >> QDDotPlanned(i,0) >> QDDotPlanned(i,1) >>QDDotPlanned(i,2);
     }
    cout << "finish the GetLengthofPlannedData"<<endl;
    OpenPlannedParameters.close();
}

bool MyRobotSolver::IDynamicsCalculation()
{
  VectorNd VecQ = VectorNd::Zero (QuadrupedRobotModel.dof_count);
  VectorNd VecQDot = VectorNd::Zero (QuadrupedRobotModel.dof_count);
  VectorNd VecTau = VectorNd::Zero (QuadrupedRobotModel.dof_count);
  VectorNd VecQDDot = VectorNd::Zero (QuadrupedRobotModel.dof_count);

    for (unsigned int i = 0; i < length_of_data; i++)
    {
        VecQ = QPlanned.row(i).transpose();

        VecQDot = QDotPlanned.row(i).transpose();
        VecQDDot = QDDotPlanned.row(i).transpose();

        InverseDynamics(QuadrupedRobotModel,VecQ,VecQDot,VecQDDot,VecTau);

        TauofIDynamics.row(i).transpose() = VecTau;

    }
    const char* filestoredlocation = "/home/kun/catkin_ws/src/single_leg_test/DataFloder/TauofInversedynamics.txt";
    FileStoreIntoTextFile(filestoredlocation, TauofIDynamics);
    cout <<"finish the Inverse Dynamics calculation" << endl;
    return true;
}

void MyRobotSolver::FDynamicsCalculation()
{
  QAcutal.row(0) = QPlanned.row(0);
  QDotAcutal.row(0) = QDotPlanned.row(0);
  QDDotAcutal.row(0) = QDDotPlanned.row(0);

  double kp = 2;
  double kd = 2;
  for (int i = 0; i < length_of_data - 1 ; ++i) {
    // get the i-th row planned data;
    VecQAct = QAcutal.row(i).transpose();
    VecQDotAct = QDotAcutal.row(i).transpose();

    // get the Tau of the ID and tau error with PD controller;
    VecTauAct = TauofIDynamics.row(i).transpose() + VecTauerror;

    // calculate the acceleration
    ForwardDynamics(QuadrupedRobotModel,VecQAct,VecQDotAct,VecTauAct,VecQDDotAct);

    // stored the acc
    QDDotAcutal.row(i) = VecQDDotAct.transpose();

    // iteration to calculate the actual velocity and position
    QDotAcutal.row(i+1) = QDotAcutal.row(i) + QDDotAcutal.row(i) * Time_derta;
    QAcutal.row(i+1) = QAcutal.row(i) + QDotAcutal.row(i) * Time_derta;

    // Calculate the error of Q and QDot
    VecQerror.transpose() = QPlanned.row(i+1) - QAcutal.row(i+1) ;
    VecQDoterror.transpose() = QDotPlanned.row(i+1) - QDotAcutal.row(i+1);

    // Calculate the torque error with PD controller
    VecTauerror = kp * VecQerror + kd * VecQDoterror;
  }
   // Stored the actual Q;
  cout << "finish the forward dynamics calculation" << endl;

  const char *filestoredlocation = "/home/kun/catkin_ws/src/single_leg_test/DataFloder/PositionofForward.txt";
  FileStoreIntoTextFile(filestoredlocation, QAcutal);

  // Stored the actual QDot;
  filestoredlocation = "/home/kun/catkin_ws/src/single_leg_test/DataFloder/VelocityofForward.txt";
  FileStoreIntoTextFile(filestoredlocation, QDotAcutal);

  // Stored the actual Acceleration
  filestoredlocation = "/home/kun/catkin_ws/src/single_leg_test/DataFloder/AccelerationofForward.txt";
  FileStoreIntoTextFile(filestoredlocation, QDDotAcutal);

}

void MyRobotSolver::setGains(const Eigen::Vector3d& kp, const Eigen::Vector3d& kd)
{
  kp_ = kp;
  kd_ = kd;
}

bool MyRobotSolver::loadParameters()
{
  Eigen::Vector3d kp,kd;
  if (node_handle_.hasParam("/single_leg_controller/x_direction/kp")) {
    node_handle_.getParam("/single_leg_controller/x_direction/kp", kp.x());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/single_leg_controller/x_direction/kp'.");
    return false;
  }
  if (node_handle_.hasParam("/single_leg_controller/x_direction/kd")) {
    node_handle_.getParam("/single_leg_controller/x_direction/kd", kd.x());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/single_leg_controller/x_direction/kd'.");
    return false;
  }

  if (node_handle_.hasParam("/single_leg_controller/y_direction/kp")) {
    node_handle_.getParam("/single_leg_controller/y_direction/kp", kp.y());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/single_leg_controller/y_direction/kp'.");
    return false;
  }
  if (node_handle_.hasParam("/single_leg_controller/y_direction/kd")) {
    node_handle_.getParam("/single_leg_controller/y_direction/kd", kd.y());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/single_leg_controller/y_direction/kd'.");
    return false;
  }

  if (node_handle_.hasParam("/single_leg_controller/z_direction/kp")) {
    node_handle_.getParam("/single_leg_controller/z_direction/kp", kp.z());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/single_leg_controller/z_direction/kp'.");
    return false;
  }
  if (node_handle_.hasParam("/single_leg_controller/z_direction/kd")) {
    node_handle_.getParam("/single_leg_controller/z_direction/kd", kd.z());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/single_leg_controller/z_direction/kd'.");
    return false;
  }
  setGains(kp, kd);

  return true;
}

void MyRobotSolver::setDesiredPositionAndVelocity(const Eigen::Vector3d& position,
                                                  const Eigen::Vector3d& velocity)
{
  desired_foot_positions = position;
  desired_foot_velocities = velocity;
}
const VectorNd& MyRobotSolver::getTauFeedForward()
{
  return TauFeedForward;
}
/**
 * @brief MyRobotSolver::update
 * @param time
 * @param period
 * @param limb
 * @return
 */
bool MyRobotSolver::update(const ros::Time& time, const ros::Duration& period,
                            const free_gait::LimbEnum& limb, bool real_time, const Eigen::Vector3d& acc_ref)
{
//    ROS_INFO("In swing leg controller update");
    calculation_iterstions = 1;//calculation_iterstions + 1;
    Eigen::Matrix3d jacobian = robot_state_->getTranslationJacobianFromBaseToFootInBaseFrame(limb);
    Eigen::Matrix3d jacobian_dot = robot_state_->getTranslationJacobianDotFromBaseToFootInBaseFrame(limb);
//    cout<<"Jacobian : "<<jacobian<<endl;
    Time_derta = period.toSec()*10;
    QAcutal.row(calculation_iterstions) = QActQueueLimb.at(limb).back();//VecQAct.transpose();
    QAcutal.row(calculation_iterstions - 1) = QActQueueLimb.at(limb).front();
    QDotAcutal.row(calculation_iterstions) = QDotActQueueLimb.at(limb).back();
    QDotAcutal.row(calculation_iterstions - 1) = QDotActQueueLimb.at(limb).front();
    for (int num = 0; num < num_of_joints; ++num) {
//      QDotAcutal(calculation_iterstions,num) = (QAcutal(calculation_iterstions,num) - QAcutal(calculation_iterstions - 1, num))/Time_derta;
      QDDotAcutal(calculation_iterstions,num) = (QDotAcutal(calculation_iterstions,num) - QDotAcutal(calculation_iterstions - 1, num))/Time_derta;
     }
//    ROS_INFO("Got Here");
    VecQAct = QAcutal.row(calculation_iterstions).transpose();
    VecQDotAct = QDotAcutal.row(calculation_iterstions).transpose();
    VecQDDotAct = QDDotAcutal.row(calculation_iterstions).transpose();
    Eigen::Vector3d jac_mul_qddot = acc_ref - jacobian_dot*VecQDotAct;
    VecQDDotAct = jacobian.transpose()*jac_mul_qddot;
//    VecQDDotAct =

    if(real_time)
      {
        std::queue<VectorNd> last_queue, current_queue;
        last_queue = QDotActQueueLimb.at(limb);
        current_queue = QDotActQueueLimb.at(limb);
        int window_size = QDotActQueueLimb.at(limb).size() - 1;
        for(int i = 0;i<QDotActQueueLimb.at(limb).size() - 1 ;i++)
          {
            current_queue.pop();
            QDotAcutal.row(calculation_iterstions) = QDotAcutal.row(calculation_iterstions) + current_queue.front();
            QDotAcutal.row(calculation_iterstions - 1) = QDotAcutal.row(calculation_iterstions - 1) + last_queue.front();
            last_queue.pop();
          }
        QDotAcutal.row(calculation_iterstions) = QDotAcutal.row(calculation_iterstions)/window_size;
        QDotAcutal.row(calculation_iterstions - 1) = QDotAcutal.row(calculation_iterstions - 1)/window_size;

        for (int num = 0; num < num_of_joints; ++num) {
          QDDotAcutal(calculation_iterstions,num) = (QDotAcutal(calculation_iterstions,num) - QDotAcutal(calculation_iterstions - 1, num))/period.toSec();

          }
//        VecQDDotAct = QDDotAcutal.row(calculation_iterstions).transpose();
        InverseDynamics(*LimbRBDLModel.at(limb),VecQAct,VecQDotAct,VecQDDotAct,VecTauAct);
      }else{
        InverseDynamics(*LimbRBDLModel.at(limb),VecQAct,VecQDotAct,VecQDDotAct,VecTauAct);
//        if(limb == free_gait::LimbEnum::RF_LEG || limb == free_gait::LimbEnum::LH_LEG)
//          VecTauAct = -VecTauAct;
      }

//    VecQDotAct = QDotAcutal.row(calculation_iterstions).transpose();
//    VecQDDotAct = QDDotAcutal.row(calculation_iterstions).transpose();

//    InverseDynamics(QuadrupedRobotModel,VecQAct,VecQDotAct,VecQDDotAct,VecTauAct);


//    for (int num = 0; num < num_of_joints; ++num) {
//      VecTauAct[num] = kp_ * (QPlanned(calculation_iterstions,num) - QAcutal(calculation_iterstions,num))
//                     + kd_* (QDotPlanned(calculation_iterstions,num)-QDotAcutal(calculation_iterstions,num))
//                     + VecTauAct[num];
//    }
//    if(limb == free_gait::LimbEnum::LF_LEG)
//      {
//        sensor_msgs::JointState q_state;
//        for (int num = 0; num < num_of_joints; num++) {
//            q_state.name.push_back("lf_leg_joint"+std::to_string(num));
//            q_state.effort.push_back(VecTauAct(num));
//            q_state.velocity.push_back(VecQDDotAct(num));
//            q_state.position.push_back(VecQDotAct(num));
//          }
//        joint_state_pub_.publish(q_state);
//      }

//    Tauacutal.row(calculation_iterstions) = VecTauAct.transpose();
//    ROS_INFO("Got Here!");
//    ROS_INFO("ID joint torque : ");
//    std::cout<<VecTauAct<<std::endl;
//    ROS_INFO("joint accelleration : ");
//    std::cout<<VecQDDotAct<<std::endl;
//    std::cout<<VecTauAct<<std::endl;

//    if(limb == free_gait::LimbEnum::RF_LEG || limb == free_gait::LimbEnum::LH_LEG)
//      VecTauAct = -VecTauAct;
    Tauacutal.row(0) = VecTauAct;
    TauFeedForward = VecTauAct;
    Eigen::Vector3d position_error_in_base, velocity_error_in_base;
    position_error_in_base = robot_state_->getTargetFootPositionInBaseForLimb(limb).vector()
        - robot_state_->getPositionBaseToFootInBaseFrame(limb).vector();
//    ROS_INFO("Position error in base : ");
//    std::cout<<position_error_in_base<<std::endl;
//    ROS_INFO("current Position in base : ");
//    std::cout<<robot_state_->getPositionBaseToFootInBaseFrame(limb).vector()<<std::endl;
//    ROS_INFO("Target Position error in base : ");
//    std::cout<<robot_state_->getTargetFootPositionInBaseForLimb(limb).vector()<<std::endl;
    velocity_error_in_base = robot_state_->getTargetFootVelocityInBaseForLimb(limb).vector()
        - robot_state_->getEndEffectorVelocityInBaseForLimb(limb).vector();
//    ROS_WARN_STREAM("Inertial Matrix :" <<QuadrupedRobotModel.IA<<std::endl);
    VecTauAct = jacobian.transpose() * (kp_.cwiseProduct(position_error_in_base)
                                        + kd_.cwiseProduct(velocity_error_in_base))
                + VecTauAct;
//
//    ROS_INFO("Finish swing leg controller update");
    return true;
}
