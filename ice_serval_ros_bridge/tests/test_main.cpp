
#include <gtest/gtest.h>
#include <ros/init.h>
#include <exception>
#include <iostream>
#include "easylogging++.h"
#include "IceServalBridge.h"

INITIALIZE_EASYLOGGINGPP

void setup()
{
  // cleanup
  system("killall servald");
  system("rm -r /tmp/mops");

  // create folder
  mkdir("/tmp/mops", 0700);

  // creating dummy file
  std::ofstream dummy;
  dummy.open("/tmp/dummy");
  dummy.close();

  std::ofstream myfile;
  myfile.open("/tmp/mops/serval.conf");
  myfile << "interfaces.0.file=/tmp/dummy\n";
  myfile << "interfaces.0.socket_type=file\n";
  myfile << "rhizome.http.port=4110\n";
  myfile << "api.restful.users.peter.password=venkman\n";
  myfile.close();
}

int main(int argc, char **argv)
{
  setup();

  ros::init(argc, argv, "ice_serval_bridge_test");
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
