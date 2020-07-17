#include "quadruped_model/quadrupedkinematics.h"
#include "ros/ros.h"

using namespace quadruped_model;
using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"kinematics_CONTROLLER");
    quadruped_model::QuadrupedKinematics robot_kinetics;
    vector<Position> foot_pose_to_hip_in_hip_frame;
    foot_pose_to_hip_in_hip_frame.resize(4);
    vector<Pose> foot_pose_to_base_in_base_frame;
    foot_pose_to_base_in_base_frame.resize(4);

    quadruped_model::JointPositionsLimb joints(0,0,0);
//    quadruped_model::JointPositionsLimb joints(0.0126214,0.728666,4.56553);//0.479233 0.310018 -0.40561 actual
//    quadruped_model::JointPositionsLimb joints(0.0142948,0.727999,-1.64635);//0.466799  0.310936 -0.423091 desired before
//    quadruped_model::JointPositionsLimb joints(0,-0.774029,1.54807);//0.427 -0.0875649   0.153204 desired after

//    HipPoseInBase hip_pose_in_base;

//    hip_pose_in_base[LimbEnum::LF_LEG] = Pose(Position(0,0,0),RotationQuaternion());

    if(robot_kinetics.FowardKinematicsSolve(joints,LimbEnum::LF_LEG,foot_pose_to_base_in_base_frame[0]))
    {
//        EulerAnglesXyz eular_xyz(foot_pose_to_base_in_base_frame.getRotation());
//        cout << "Kinematics solve result:" << foot_pose_to_base_in_base_frame.getPosition() << endl <<
//            "Rotation: " << "roll:" << eular_xyz.roll() << endl << "yaw" << eular_xyz.yaw() << endl <<
//            "pitch" << eular_xyz.pitch() << endl;
        cout << "bingo" << endl;
    }
    std::cout << "foot_pose_to_base_in_base_frame is" << foot_pose_to_base_in_base_frame[0].getPosition() << std::endl;

//    foot_pose_to_hip_in_hip_frame[0] =
//        robot_kinetics.getPositionFootToHipInHipFrame(LimbEnum::LF_LEG,foot_pose_to_base_in_base_frame[0].getPosition());

//    std::cout << "foot_pose_to_hip_in_hip_frame is" << foot_pose_to_hip_in_hip_frame[0] << std::endl;

//    for (unsigned int i = 0; i < 3; i++) {
        foot_pose_to_base_in_base_frame[0].getPosition().x() = 0.431;
        foot_pose_to_base_in_base_frame[0].getPosition().y() = 0.287;
        foot_pose_to_base_in_base_frame[0].getPosition().z() = -0.625626;

        if(robot_kinetics.InverseKinematicsSolve(foot_pose_to_base_in_base_frame[0].getPosition(), LimbEnum::LF_LEG, joints, joints,"IN_LEFT"))
        {
            cout << "Inverse kinematics slove results in_left: " << endl << "joint1 = "<<joints(0)<<endl
                 <<"joint2 = "<<joints(1)<<endl<<"joint3 = "<<joints(2)<<endl;
        }

//    }


//    if(robot_kinetics.InverseKinematicsSolve(foot_pose_to_base_in_base_frame[0].getPosition(), LimbEnum::LF_LEG, joints, joints,"IN_RIGHT"))
//    {
//        cout << "Inverse kinematics slove results in_right: " << endl << "joint1 = "<<joints(0)<<endl
//             <<"joint2 = "<<joints(1)<<endl<<"joint3 = "<<joints(2)<<endl;
//    }

//    if(robot_kinetics.InverseKinematicsSolve(foot_pose_to_base_in_base_frame[0].getPosition(), LimbEnum::LF_LEG, joints, joints,"OUT_RIGHT"))
//    {
//        cout << "Inverse kinematics slove results OUT_right: " << endl << "joint1 = "<<joints(0)<<endl
//             <<"joint2 = "<<joints(1)<<endl<<"joint3 = "<<joints(2)<<endl;
//    }

//    if(robot_kinetics.InverseKinematicsSolve(foot_pose_to_base_in_base_frame[0].getPosition(), LimbEnum::LF_LEG, joints, joints,"OUT_LEFT"))
//    {
//        cout << "Inverse kinematics slove results OUT_LEFT: " << endl << "joint1 = "<<joints(0)<<endl
//             <<"joint2 = "<<joints(1)<<endl<<"joint3 = "<<joints(2)<<endl;
//    }
    //joint1 = 0.153218
    //joint2 = 0.730329
    //joint3 = -1.44074


//    robot_kinetics.FowardKinematicsSolve(joints,LimbEnum::RF_LEG,foot_pose_to_base_in_base_frame[1]);
//    robot_kinetics.FowardKinematicsSolve(joints,LimbEnum::RH_LEG,foot_pose_to_base_in_base_frame[2]);
//    robot_kinetics.FowardKinematicsSolve(joints,LimbEnum::LH_LEG,foot_pose_to_base_in_base_frame[3]);
//    foot_pose_to_hip_in_hip_frame[1] =
//        robot_kinetics.getPositionFootToHipInHipFrame(LimbEnum::RF_LEG,foot_pose_to_base_in_base_frame[1].getPosition());
//    foot_pose_to_hip_in_hip_frame[2] =
//        robot_kinetics.getPositionFootToHipInHipFrame(LimbEnum::RH_LEG,foot_pose_to_base_in_base_frame[2].getPosition());
//    foot_pose_to_hip_in_hip_frame[3] =
//        robot_kinetics.getPositionFootToHipInHipFrame(LimbEnum::LH_LEG,foot_pose_to_base_in_base_frame[3].getPosition());

//    for (unsigned int i = 0; i < 4; i++) {
//        cout << "foot_pose_to_hip_in_hip_frame is " << foot_pose_to_base_in_base_frame[i].getPosition() << endl;
//    }





//    Pose Base_pose_world_frame;


////    JointPositionsLeg joint_limb;
////    Position I_r_IF, B_r_BF;
////    Position I_r_IB = state_->getPositionWorldToBaseInWorldFrame();
////    RotationQuaternion I_R_B = state_->getOrientationBaseToWorld();
////    for(const auto& limb : getLimbs())
////    {
////        if(state_->isSupportLeg(limb))
////        {
////            footholdsInSupport_[limb] = state_->getPositionWorldToFootInWorldFrame(limb);
////            I_r_IF = footholdsInSupport_.at(limb);
////            B_r_BF = I_R_B.inverseRotate(I_r_IF - I_r_IB);
////            state_->getLimbJointPositionsFromPositionBaseToFootInBaseFrame(B_r_BF, limb, joint_limb);
////            state_->setJointPositionsForLimb(limb, joint_limb);
////        }

    return 0;
}

