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
//  serval_ros_bridge::ImageToServal hd(nh_, pnh_);

  ice::IceServalBridge node(nh_, pnh_);
  node.init();

  ros::spin();

  return 0;
}
