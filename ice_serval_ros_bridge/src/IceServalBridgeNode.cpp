/*
 * iceservalbridgenode.cpp
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */


#include <ros/ros.h>
#include "easylogging++.h"
#include "IceServalBridge.h"

INITIALIZE_EASYLOGGINGPP


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ice_serval_bridge");

  ros::NodeHandle nh_("");
  ros::NodeHandle pnh_("~");

  auto node = std::make_shared<ice::IceServalBridge>(nh_, pnh_);
  node->init();

  ros::spin();

  return 0;
}
