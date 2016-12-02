/*
 * KnowledgeBaseNode.cpp
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */


#include <ros/ros.h>
#include <signal.h>
#include "easylogging++.h"
#include "IceServalBridge.h"
#include <ice/information/InformationStore.h>

INITIALIZE_EASYLOGGINGPP

std::shared_ptr<ice::IceServalBridge> node;

void mySigintHandler(int sig)
{
  node->cleanUp();
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "knowledge_base_eval");

  ros::NodeHandle nh_("");
  ros::NodeHandle pnh_("~");

  int number;
  pnh_.param("infos", number, 0);

  if (number == 0)
  {
    std::cerr << "Define the number of information in knowledgebase 'infos:=[number]'" << std::endl;
    return 1;
  }

  node = std::make_shared<ice::IceServalBridge>(nh_, pnh_);
  node->init();
  signal(SIGINT, mySigintHandler);

  // stuff
  auto infoStore = node->getKnowlegeBase()->informationStore;
  auto rep = node->getGContainerFactory()->getRepresentation("http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark");
  auto x = rep->accessPath({"http://vs.uni-kassel.de/Ice#XCoordinate"});
  auto y = rep->accessPath({"http://vs.uni-kassel.de/Ice#YCoordinate"});
  auto z = rep->accessPath({"http://vs.uni-kassel.de/Ice#ZCoordinate"});
  auto lid = rep->accessPath({"http://vs.uni-kassel.de/TurtleBot#LandmarkId"});

  assert(rep != nullptr);
  assert(x != nullptr);
  assert(y != nullptr);
  assert(z != nullptr);
  assert(lid != nullptr);

  for (int i=0; i < number; ++i)
  {
    auto spec = std::make_shared<ice::InformationSpecification>("http://vs.uni-kassel.de/TurtleBot#CS" + i,
                                                                "http://vs.uni-kassel.de/TurtleBot#ChargeStation",
                                                                "http://vs.uni-kassel.de/Ice#Position",
                                                                "http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark",
                                                                "http://vs.uni-kassel.de/TurtleBot#Landmark");

    auto instance = node->getGContainerFactory()->makeInstance(rep);

    double xv = 1000;//rand();
    double yv = 2000;//rand();
    double zv = 3000;//rand();
    long lidv = 4000;//rand();
    instance->set(x, &xv);
    instance->set(y, &yv);
    instance->set(z, &zv);
    instance->set(lid, &lidv);

    infoStore->addInformation(spec, instance);
  }

  std::cout << "#########################################################################" << std::endl;
  std::cout << "Initialized" << std::endl;
  std::cout << "#########################################################################" << std::endl;

  ros::spin();

  return 0;
}
