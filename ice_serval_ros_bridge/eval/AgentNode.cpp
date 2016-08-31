/*
 * KnowledgeBaseNode.cpp
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */


#include <chrono>
#include <ros/ros.h>
#include <signal.h>
#include "easylogging++.h"
#include "IceServalBridge.h"
#include <ice/information/InformationStore.h>
#include <ice/communication/jobs/InformationRequest.h>

INITIALIZE_EASYLOGGINGPP


std::chrono::time_point<std::chrono::system_clock> start;
std::shared_ptr<ice::IceServalBridge> node;

void callback(std::shared_ptr<ice::InformationRequest> request)
{
  auto end = std::chrono::system_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();



  std::cout << "#########################################################################" << std::endl;
  std::cout << "Finished " << dur << std::endl;
  std::cout << "#########################################################################" << std::endl;
}

void mySigintHandler(int sig)
{
  node->cleanUp();
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agent_eval");

  ros::NodeHandle nh_("");
  ros::NodeHandle pnh_("~");

  node = std::make_shared<ice::IceServalBridge>(nh_, pnh_);
  node->init();
  signal(SIGINT, mySigintHandler);

  // stuff
  sleep(5);
  auto mops = node->getEntityDirector()->lookup(ice::EntityDirectory::ID_ONTOLOGY, "http://vs.uni-kassel.de/TurtleBot#Mops", false);

  assert(mops != nullptr);

  auto spec = std::make_shared<ice::InformationSpecification>("*",
                                                              "http://vs.uni-kassel.de/TurtleBot#ChargeStation",
                                                              "http://vs.uni-kassel.de/Ice#Position",
                                                              "http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark",
                                                              "http://vs.uni-kassel.de/TurtleBot#Landmark");

  auto request = std::make_shared<ice::InformationRequest>(node, mops);
  request->getRequests().push_back(spec);
  request->setCallbackFinished(&callback);


  std::cout << "#########################################################################" << std::endl;
  std::cout << "Start" << std::endl;
  std::cout << "#########################################################################" << std::endl;

  start = std::chrono::system_clock::now();
  node->getCommunicationInterface()->addComJob(request);

  ros::spin();

  return 0;
}
