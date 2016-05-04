
#include <gtest/gtest.h>
#include <ros/init.h>
#include <exception>
#include <iostream>
#include <fstream>
#include <stdlib.h>

void createConfig()
{
  // create folder
  mkdir("/tmp/instance1", 0700);
  mkdir("/tmp/instance2", 0700);

  // creating dummy file
  std::ofstream dummy;
  dummy.open("/tmp/dummy");
  dummy.close();

  std::ofstream myfile;
  myfile.open("/tmp/instance1/serval.conf");
  myfile << "interfaces.0.file=/tmp/dummy\n";
  myfile << "interfaces.0.socket_type=file\n";
//  myfile << "interfaces.0.match=*\n";
//  myfile << "interfaces.0.socket_type=dgram\n";
//  myfile << "interfaces.0.type=ethernet\n";
//  myfile << "interfaces.0.port=4110\n";
  myfile << "rhizome.http.port=4110\n";
  myfile << "api.restful.users.peter.password=venkman\n";
  myfile.close();

  std::ofstream myfile2;
  myfile2.open("/tmp/instance2/serval.conf");
  myfile2 << "interfaces.0.file=/tmp/dummy\n";
  myfile2 << "interfaces.0.socket_type=file\n";
//  myfile << "interfaces.0.match=*\n";
//  myfile << "interfaces.0.socket_type=dgram\n";
//  myfile << "interfaces.0.type=ethernet\n";
//  myfile << "interfaces.0.port=4110\n";
  myfile2 << "rhizome.http.port=4110\n";
  myfile2 << "api.restful.users.peter.password=venkman\n";
  myfile2.close();
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
