
#include <gtest/gtest.h>
#include <ros/init.h>
#include <exception>
#include <iostream>
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ice_engine_test");
  ::testing::InitGoogleTest(&argc, argv);
  el::Loggers::addFlag(el::LoggingFlag::HierarchicalLogging);
  el::Loggers::setLoggingLevel(el::Level::Info);

  int result = RUN_ALL_TESTS();

  int kill = 5 / 0;

  return result;
}
