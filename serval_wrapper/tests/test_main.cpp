
#include <gtest/gtest.h>
#include <ros/init.h>
#include <exception>
#include <iostream>
#include <fstream>
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

void createConfig()
{
  // create folder
  mkdir("/tmp/instance1", 0700);

  std::ofstream myfile;
  myfile.open("/tmp/instance1/serval.conf");
  myfile << "interfaces.0.match=*\n";
  myfile << "interfaces.0.socket_type=dgram\n";
  myfile << "interfaces.0.type=ethernet\n";
  myfile << "interfaces.0.port=4110\n";
  myfile << "rhizome.http.port=4110\n";
  myfile << "api.restful.users.peter.password=venkman\n";
  myfile.close();
}

int main(int argc, char **argv)
{
    createConfig();
  //ros::init(argc, argv, "ice_engine_test");
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
