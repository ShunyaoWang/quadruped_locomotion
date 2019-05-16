/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2016 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <rbdl/rbdl.h>
#include <fstream>
#include <math.h>


using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;

int main (int argc, char* argv[]) {
    rbdl_check_api_version (RBDL_API_VERSION);

    Model* model = NULL;
    unsigned int body_a_id, body_b_id, body_c_id;
    Body body_a, body_b, body_c;
    Joint joint_a, joint_b, joint_c;

    model = new Model();

    model->gravity = Vector3d (0., 0., -9.81);

    body_a = Body (1.17, Vector3d (0., 0.0128, 0.), Matrix3d(0.00172, 0. , 0., 0., 0.00132, 0., 0., 0., 0.00215 ));//Body (const double &mass, const Math::Vector3d &com, const Math::Matrix3d &inertia_C)
    joint_a = Joint(
        JointTypeRevolute,
        Vector3d (0., 0., 1.)
    );//body_a's mass 1.0, center of mass and the inertia at the center of mass/or the radius gradius.

    Matrix3d B0toB1Rotation = roty(M_PI/2);

    body_a_id = model->AddBody(0, SpatialTransform(B0toB1Rotation,Vector3d(0., 0., 0.)), joint_a, body_a);
    body_b = Body (3.39, Vector3d (0.114, 0., 0.0594), Matrix3d (0.00302, 0., 0., 0., 0.0269, 0., 0., 0., 0.0285));
        joint_b = Joint (
        JointTypeRevolute,
        Vector3d (0., 0., 1.)
    );
    //pay attention to the transform order of the matrix!!!!!!from the right multiply.
    Matrix3d B1toB2Rotation = rotx(-M_PI/2);
    body_b_id = model->AddBody(body_a_id, SpatialTransform(B1toB2Rotation,Vector3d(0., 0., 0.1)), joint_b, body_b);

    body_c = Body (1.41, Vector3d (0.00949, 0., -0.00166),Matrix3d (0.0040547, 0., 0., 0., 0.0109, 0.000222, 0., 0.000222, 0.0111));
        joint_c = Joint (
        JointTypeRevolute,
        Vector3d (0., 0., 1.)
    );

    body_c_id = model->AddBody(body_b_id, Xtrans(Vector3d(0.25, 0., 0.1)), joint_c, body_c);

    std::cout<<"The name of body_a_id is:"<<body_a_id<<std::endl;
    std::cout<<"The name of body_b_id is:"<<body_b_id<<std::endl;
    std::cout<<"The name of body_c_id is:"<<body_c_id<<std::endl;

    //read planned joint,velocity and acceleration from the file
    ifstream fin;
    fin.open("/home/kun/catkin_ws/src/single_leg_test/DataFloder/PlannedData.txt");
    if (!fin.is_open())
    {
        cout << "could not open the d_delete file" << endl;
        cout << "Program terminating" << endl;
        exit(EXIT_FAILURE);
    }
    /*Store the torque from the Inverse Dynamics*/
    ofstream fout;
    fout.open("/home/kun/Desktop/torque.txt");
    if (!fout.is_open())
    {
        cout << "could not open the torque.txt file" << endl;
        cout << "Program terminating" << endl;
        exit(EXIT_FAILURE);
    }
    /*Store the acceleration from the Forward Dynamics*/
    ofstream fout_acc;
    fout_acc.open("/home/kun/Desktop/acc_FD.txt");
    if (!fout_acc.is_open())
    {
        cout << "could not open the acc_FD.txt file" << endl;
        cout << "Program terminating" << endl;
        exit(EXIT_FAILURE);
    }

    unsigned int length_of_data = 10001;
    /*Store the planned joint's angle, velocity and acceleration*/
    MatrixNd Q(length_of_data,3) ,QDot(length_of_data,3),QDDot(length_of_data,3);
    MatrixNd Tau = MatrixNd::Zero(length_of_data,3);
    for (unsigned int i = 0; i < length_of_data; i++) {
        fin >> Q(i,0) >> Q(i,1) >> Q(i,2);
        fin >> QDot(i,0) >> QDot(i,1) >> QDot(i,2);
        fin >> QDDot(i,0) >> QDDot(i,1) >> QDDot(i,2);
     }


    fin.close();
    /*confirm the parents of the body*/
    cout << model ->dof_count <<endl;
    for(unsigned int i = 0; i < model->dof_count; i++)
    {
        std::cout <<"the lambda is " << model->lambda[i] <<std::endl;
        cout << i <<endl;
    }

    /*Calculation of the inverse dynamic*/
    VectorNd VTau_1;
    VectorNd VQ_1, VQd_1, VQdd_1;
    VTau_1 = Vector3d::Zero(3);
    for (unsigned int i = 0; i < length_of_data; i++) {

        VQ_1 = Q.row(i).transpose();
        VQd_1 = QDot.row(i).transpose();
        VQdd_1 = QDDot.row(i).transpose();

        InverseDynamics(*model, VQ_1, VQd_1, VQdd_1, VTau_1);
        Tau.row(i) = VTau_1.transpose();
        fout << VTau_1(0) << "\t" << VTau_1(1) << "\t" << VTau_1(2) << endl;
        cout << i <<endl;
    }
    fout.close();
    cout << "finish the ID calculation" <<endl;
    /*Calculation about the ForwardDynamic*/
    MatrixNd MQ_actual(length_of_data,3) ,MQDot_actual(length_of_data,3),MQDDot_actual(length_of_data,3),MTau_acutal(length_of_data,3);
    /*The initial value of the Q,Qdot;*/
    MQ_actual(0,0) = -1.2; MQ_actual(0,1) = -1.; MQ_actual(0,2) = -0.4;
    MQDot_actual(0,0) = 0.; MQDot_actual(0,1) = 0.; MQDot_actual(0,2)=0.;
    /*the time duration dt*/
    double dt = 0.001;
    Vector3d Verror_p, Verror_v,Vtau_error;
    Vtau_error = Vector3d::Zero(3);
    Verror_p = Vector3d::Zero(3);
    Verror_v = Vector3d::Zero(3);
    double kp = 2.; double kd = 2.;

    for (unsigned int i = 0; i < length_of_data-1; i++) {
        //cout << i <<endl;
        /*Q_1 is the i-th row of the Q_actual, used in the parameters of FD*/
        VQ_1 = MQ_actual.row(i).transpose();
        VQd_1 = MQDot_actual.row(i).transpose();
        //cout << VQ_1 << VQd_1 << VQdd_1 <<endl;

        /*Tau is calculated by the Inverse Dynamics*/
        VTau_1 = Tau.row(i).transpose()+ Vtau_error;
        //cout << "the tourque is :"  << Tau.row(i) <<endl;
        ForwardDynamics(*model, VQ_1, VQd_1, VTau_1, VQdd_1);
        MQDDot_actual.row(i) = VQdd_1.transpose();//store the Qdd_1 to QDDot_actual
       // cout << "the acc is: " << VQdd_1 <<endl;

        /*Iteration of the position and velocity*/
        MQDot_actual.row(i+1) = MQDot_actual.row(i) + VQdd_1.transpose() * dt;
        MQ_actual.row(i+1) = MQ_actual.row(i) + MQDot_actual.row(i) * dt;

        /*Calculate the error,Q, QDot and QDDot is the theorem values */
        Verror_p.transpose() = Q.row(i+1) - MQ_actual.row(i+1);
        Verror_v.transpose() = QDot.row(i+1) - MQDot_actual.row(i+1);
        /*PD control law*/
        Vtau_error = kp * Verror_p + kd * Verror_v;
        //cout << "the torque error is: " <<endl << Vtau_error <<endl;

        fout_acc << VQ_1(0) << "\t" << VQ_1(1) << "\t" << VQ_1(2) << "\t" <<
                    VQd_1(0) << "\t" << VQd_1(1) << "\t" << VQd_1(2) << "\t" <<
                    VQdd_1(0) << "\t" << VQdd_1(1) << "\t" << VQdd_1(2) << "\t" <<
                    VTau_1(0) << "\t" << VTau_1(1) << "\t" <<VTau_1(2) << endl;        

    }
    cout << "finish the inverse dynamics calculation" <<endl;
    fout_acc.close();
    delete model;
    return 0;
}
