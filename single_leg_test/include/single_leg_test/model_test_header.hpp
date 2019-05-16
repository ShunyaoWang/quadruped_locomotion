#ifndef MODEL_TEST_HEADER_HPP
#define MODEL_TEST_HEADER_HPP
#include <rbdl/rbdl.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <string>
#include "eigen3/Eigen/Core"
#include "ros/ros.h"
#include "rbdl/addons/urdfreader/urdfreader.h"

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/free_gait_core.hpp"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
#define PI acos(-1)

class MyRobotSolver
{
public:
  typedef std::unordered_map<free_gait::LimbEnum, std::queue<VectorNd>, EnumClassHash> LimbQueue;
  typedef std::unordered_map<free_gait::LimbEnum, Model*, EnumClassHash> LimbModel;
    MyRobotSolver(const ros::NodeHandle& node_handle,
                  std::shared_ptr<free_gait::State> robot_state);
    ~MyRobotSolver();
    void GetLengthofPlannedData();//get the length of the planned data;
    void GetPlannedTorque();
    void model_initialization();//initialization of a model;
    bool IDynamicsCalculation();//Calculate the torque by inverse dynamics.
    Model& getModel();
    const MatrixNd& getTau();
    const MatrixNd& getQPlanned();
    const double& getTime_derta();
    const MatrixNd& getQDotPlanned();
    const unsigned int& getlength_of_data();
    const unsigned int& getnum_of_joints();
    const unsigned int& getcalculation_iterations();
    void setvecQAct(const Eigen::Vector3d& joint_positions, const free_gait::LimbEnum& limb);
    void setvecQDotAct(const Eigen::Vector3d& joint_velocities, const free_gait::LimbEnum& limb);
    void setQAcutal(const VectorNd Rowszero);
    void setQDotAcutal(const VectorNd Rowszero);
    void setGains(const Eigen::Vector3d& kp, const Eigen::Vector3d& kd);
    void setDesiredPositionAndVelocity(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);
    const VectorNd& getVecQAct();
    const VectorNd& getVecTauAct();
    const MatrixNd& getQAcutal();
    const MatrixNd& getQDotAcutal();
    const MatrixNd& getQDDotAcutal();
    const MatrixNd& getTauAcutal();

    bool loadParameters();
    bool loadLimbModelFromURDF();

    void FDynamicsCalculation();//Calculate the forward dynamic with PD controller
    void FileStoreIntoTextFile(const char *filestoredlocation, const MatrixNd & Stored_data);
    bool update(const ros::Time& time, const ros::Duration& period, const free_gait::LimbEnum& limb);
protected:

    ros::NodeHandle node_handle_;

    std::shared_ptr<free_gait::State> robot_state_;

    MatrixNd QPlanned, QDotPlanned, QDDotPlanned;
    MatrixNd TauofIDynamics;
    unsigned int length_of_data;
    unsigned int calculation_iterstions;
    unsigned int num_of_joints;
    MatrixNd QAcutal, QDotAcutal, QDDotAcutal,Tauacutal;
    Model QuadrupedRobotModel;
    LimbModel LimbRBDLModel;
    double Time_derta;
    VectorNd VecQAct, VecQDotAct, VecQDDotAct, VecTauAct;
    VectorNd VecTauerror, VecQerror, VecQDoterror;
    Eigen::Vector3d kp_, kd_;
    Eigen::Vector3d desired_foot_positions, desired_foot_velocities;

    LimbQueue QActQueueLimb, QDotActQueueLimb;
};

#endif
