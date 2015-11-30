
#include <gtest/gtest.h>
#include <ros/init.h>
#include <exception>
#include <iostream>
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ice_engine_test");
//  try
//  {
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
//  }
//  catch (std::exception &e)
//  {
//    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
//  }
//  return 1;
}
