
#include <fstream>
#include <iostream>
#include <memory>
#include <sys/stat.h>
#include <sys/types.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <serval_interface.h>

#include "Entity.h"
#include "EntityDirectory.h"
#include "IceServalBridge.h"
#include "ServalCommunication.h"

#include "gtest/gtest.h"

TEST(Bridge, discovery)
{
  ros::NodeHandle nh_("");
  ros::NodeHandle pnh_("~");

  ice::InitParams *params1 = new ice::InitParams();
  ice::InitParams *params2 = new ice::InitParams();

  std::string path = ros::package::getPath("ice_serval_ros_bridge");

  params1->ontologyIri = "http://vs.uni-kassel.de/IceServalBridgeTest";
  params1->ontologyIriSelf = "http://vs.uni-kassel.de/IceServalBridgeTest#Mops";
  params1->ontologyPath = path + "/tests/data/";
  params1->servalInstancePath = "/tmp/mops";
  params1->servalHost = "localhost";
  params1->servalPort = 4111;
  params1->servalUser = "peter";
  params1->servalPassword = "venkman";
  params1->xmlInfoPath = path + "/tests/data/info_bridge_off.xml";

  params2->ontologyIri = "http://vs.uni-kassel.de/IceServalBridgeTest";
  params2->ontologyIriSelf = "http://vs.uni-kassel.de/IceServalBridgeTest#Zwerg";
  params2->ontologyPath = path + "/tests/data/";
  params2->servalInstancePath = "/tmp/zwerg";
  params2->servalHost = "localhost";
  params2->servalPort = 4112;
  params2->servalUser = "peter";
  params2->servalPassword = "venkman";
  params2->xmlInfoPath = path + "/tests/data/info_bridge_req.xml";

  ice::IceServalBridge::createConfig(params1);
  ice::IceServalBridge::createConfig(params2);

  ice::IceServalBridge mops = ice::IceServalBridge(nh_, pnh_, params1);
  ice::IceServalBridge zwerg = ice::IceServalBridge(nh_, pnh_, params2);

  mops.init();
  zwerg.init();

  // sleep some time and let the discovery happen
  sleep(5);

  // Check if the robots has found each other
  std::string servalZwerg, servalMops;
  ASSERT_TRUE(zwerg.identityDirectory->self->getId(ice::EntityDirectory::ID_SERVAL, servalZwerg));
  ASSERT_TRUE(mops.identityDirectory->self->getId(ice::EntityDirectory::ID_SERVAL, servalMops));

  auto idMopsByZwerg = zwerg.identityDirectory->lookup(ice::EntityDirectory::ID_SERVAL, servalMops, false);
  auto idZwergByMops = mops.identityDirectory->lookup(ice::EntityDirectory::ID_SERVAL, servalZwerg, false);

  ASSERT_NE(nullptr, idZwergByMops);
  ASSERT_NE(nullptr, idMopsByZwerg);

  // check ontology ids
  idMopsByZwerg = zwerg.identityDirectory->lookup(ice::EntityDirectory::ID_ONTOLOGY, params1->ontologyIriSelf, false);
  idZwergByMops = mops.identityDirectory->lookup(ice::EntityDirectory::ID_ONTOLOGY, params2->ontologyIriSelf, false);

  ASSERT_NE(nullptr, idMopsByZwerg);
  ASSERT_NE(nullptr, idZwergByMops);
}
