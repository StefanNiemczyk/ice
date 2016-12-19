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
  std::string name = "Leonardo";
  std::string lower = name;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  ros::init(argc, argv, "ice_tb_node");
  el::Loggers::addFlag(el::LoggingFlag::HierarchicalLogging);
  el::Loggers::setLoggingLevel(el::Level::Info);

  auto node = std::make_shared<ice::TBKnowledgeBase>(name);
  node->init();
  node->start();

  ros::spin();

  node->cleanUp();

  return 0;
}
