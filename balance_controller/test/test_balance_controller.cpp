/*
 *  filename.cpp
 *  Descriotion: gtest
 *
 *  Created on: Mar 15, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
// gtest
#include <gtest/gtest.h>
#include "ros/ros.h"

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_balance_controller");
  ros::NodeHandle nh("~");
  srand((int)time(0));
  return RUN_ALL_TESTS();
}
