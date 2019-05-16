#include "single_leg_test/model_test_header.hpp"
#include <iostream>
#include <fstream>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;

int main(int argc, char *argv[])
{
    rbdl_check_api_version(RBDL_API_VERSION);

    MyRobotSolver solver_test;
    solver_test.model_initialization();
    solver_test.GetLengthofPlannedData();
    solver_test.IDynamicsCalculation();
    solver_test.FDynamicsCalculation();
    return 0;
}
