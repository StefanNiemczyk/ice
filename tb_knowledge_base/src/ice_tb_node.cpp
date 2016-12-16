/*
 * ice_tb_node.cpp
 *
 *  Created on: Dec 16, 2016
 *      Author: sni
 */

#include <ros/ros.h>
#include "easylogging++.h"
#include "TBKnowledgeBase.h"

INITIALIZE_EASYLOGGINGPP


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ice_tb_node");

  auto node = std::make_shared<ice::TBKnowledgeBase>("Leonardo");
  node->init();
  node->start();

  ros::spin();

  node->cleanUp();

  return 0;
}
