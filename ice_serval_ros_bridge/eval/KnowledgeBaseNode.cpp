/*
 * KnowledgeBaseNode.cpp
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */


#include <ros/ros.h>
#include "easylogging++.h"
#include "IceServalBridge.h"
#include <ice/information/InformationStore.h>

INITIALIZE_EASYLOGGINGPP


int main(int argc, char **argv)
{
  ros::init(argc, argv, "knowledge_base_eval");

  ros::NodeHandle nh_("");
  ros::NodeHandle pnh_("~");

  auto node = std::make_shared<ice::IceServalBridge>(nh_, pnh_);
  node->init();

  // stuff
  auto infoStore = node->getInformationStore();
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

  for (int i=0; i < 500; ++i)
  {
    auto spec = std::make_shared<ice::InformationSpecification>("http://vs.uni-kassel.de/TurtleBot#CS" + i,
                                                                "http://vs.uni-kassel.de/TurtleBot#ChargeStation",
                                                                "http://vs.uni-kassel.de/Ice#Position",
                                                                "http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark",
                                                                "http://vs.uni-kassel.de/TurtleBot#Landmark");

    auto instance = node->getGContainerFactory()->makeInstance(rep);

    double xv = rand();
    double yv = rand();
    double zv = rand();
    long lidv = rand();
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
