
#include <fstream>
#include <iostream>
#include <memory>
#include <sys/stat.h>
#include <sys/types.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <serval_interface.h>

#include "Identity.h"
#include "IdentityDirectory.h"
#include "IceServalBridge.h"
#include "ServalCommunication.h"

#include "gtest/gtest.h"

void createConfig(ice::InitParams const * const params)
{
  // create folder
  mkdir(params->servalInstancePath.c_str(), 0700);

  std::ofstream myfile;
  myfile.open(params->servalInstancePath + "/serval.conf");
  myfile << "interfaces.0.match=*\n";
  myfile << "interfaces.0.socket_type=dgram\n";
  myfile << "interfaces.0.type=ethernet\n";
  myfile << "interfaces.0.port=4110\n";
  myfile << "rhizome.http.port=" << params->servalPort << "\n";
  myfile << "api.restful.users." << params->servalUser << ".password=" << params->servalPassword << "\n";
  myfile.close();
}


TEST(Bridge, discovery)
{
  int values = 0;
  char **args;
  ros::init(values, args, "ice_serval_bridge");

  ros::NodeHandle nh_("");
  ros::NodeHandle pnh_("~");

  ice::InitParams *params1 = new ice::InitParams();
  ice::InitParams *params2 = new ice::InitParams();

  params1->ontologyIri = "http://vs.uni-kassel.de/IceTest";
  params1->ontologyIriSelf = "http://vs.uni-kassel.de/IceTest#Mops";
  params1->ontologyPath = "/tmp";
  params1->servalInstancePath = "/tmp/mops";
  params1->servalHost = "localhost";
  params1->servalPort = 4111;
  params1->servalUser = "peter";
  params1->servalPassword = "venkman";

  params2->ontologyIri = "http://vs.uni-kassel.de/IceTest";
  params2->ontologyIriSelf = "http://vs.uni-kassel.de/IceTest#Zwerg";
  params2->ontologyPath = "/tmp";
  params2->servalInstancePath = "/tmp/zwerg";
  params2->servalHost = "localhost";
  params2->servalPort = 4112;
  params2->servalUser = "peter";
  params2->servalPassword = "venkman";

  createConfig(params1);
  createConfig(params2);

  ice::IceServalBridge mops = ice::IceServalBridge(nh_, pnh_, params1);
  ice::IceServalBridge zwerg = ice::IceServalBridge(nh_, pnh_, params2);

  mops.init();
  zwerg.init();

  // sleep some time and let the discovery happen
  sleep(20);

  // Check if the robots has found each other
  std::string servalZwerg, servalMops;
  ASSERT_TRUE(zwerg.identityDirectory->self->getId(ice::IdentityDirectory::ID_SERVAL, servalZwerg));
  ASSERT_TRUE(mops.identityDirectory->self->getId(ice::IdentityDirectory::ID_SERVAL, servalMops));

  auto idMopsByZwerg = zwerg.identityDirectory->lookup(ice::IdentityDirectory::ID_SERVAL, servalMops, false);
  auto idZwergByMops = mops.identityDirectory->lookup(ice::IdentityDirectory::ID_SERVAL, servalZwerg, false);

  ASSERT_NE(nullptr, idZwergByMops);
  ASSERT_NE(nullptr, idMopsByZwerg);

  // check ontology ids
  idMopsByZwerg = zwerg.identityDirectory->lookup(ice::IdentityDirectory::ID_ONTOLOGY, params1->ontologyIriSelf, false);
  idZwergByMops = mops.identityDirectory->lookup(ice::IdentityDirectory::ID_ONTOLOGY, params2->ontologyIriSelf, false);

  ASSERT_NE(nullptr, idMopsByZwerg);
  ASSERT_NE(nullptr, idZwergByMops);
}
