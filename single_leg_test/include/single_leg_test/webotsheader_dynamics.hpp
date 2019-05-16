#ifndef WEBOTSHEADER_DYNAMICS_HPP
#define WEBOTSHEADER_DYNAMICS_HPP
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include "single_leg_test/model_test_header.hpp"

class webotsolver:public MyRobotSolver
{
public:
    webotsolver();
    void moternameinitialization(string& Motorname);
    void

private:

};

#endif // WEBOTSHEADER_DYNAMICS_HPP
