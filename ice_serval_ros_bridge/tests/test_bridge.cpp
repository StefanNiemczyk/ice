
#include <fstream>
#include <iostream>
#include <memory>
#include <sys/stat.h>
#include <sys/types.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <serval_interface.h>
#include <ice/information/InformationElement.h>
#include <ice/information/InformationStore.h>
#include <ice/representation/GContainer.h>

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
  params1->servalPort = 4110;
  params1->servalUser = "peter";
  params1->servalPassword = "venkman";
  params1->xmlInfoPath = path + "/tests/data/info_bridge_off.xml";

  params2->ontologyIri = "http://vs.uni-kassel.de/IceServalBridgeTest";
  params2->ontologyIriSelf = "http://vs.uni-kassel.de/IceServalBridgeTest#Zwerg";
  params2->ontologyPath = path + "/tests/data/";
  params2->servalInstancePath = "/tmp/mops";
  params2->servalHost = "localhost";
  params2->servalPort = 4110;
  params2->servalUser = "peter";
  params2->servalPassword = "venkman";
  params2->xmlInfoPath = path + "/tests/data/info_bridge_req.xml";

  ice::IceServalBridge mops = ice::IceServalBridge(nh_, pnh_, params1);
  ice::IceServalBridge zwerg = ice::IceServalBridge(nh_, pnh_, params2);

  zwerg.init();
  mops.init();

  // store information in information store
  std::string repStr = mops.ontologyInterface->toShortIri("http://www.semanticweb.org/sni/ontologies/2013/7/Ice#CoordinatePositionRep");
  auto rep = mops.gcontainerFactory->getRepresentation(repStr);

  ASSERT_NE(nullptr, rep);

  auto x = rep->accessPath({mops.ontologyInterface->toShortIri("http://www.semanticweb.org/sni/ontologies/2013/7/Ice#XCoordinate")});
  auto y = rep->accessPath({mops.ontologyInterface->toShortIri("http://www.semanticweb.org/sni/ontologies/2013/7/Ice#YCoordinate")});
  auto z = rep->accessPath({mops.ontologyInterface->toShortIri("http://www.semanticweb.org/sni/ontologies/2013/7/Ice#ZCoordinate")});

  ASSERT_NE(nullptr, x);
  ASSERT_NE(nullptr, y);
  ASSERT_NE(nullptr, z);

  auto instance = mops.gcontainerFactory->makeInstance(rep);

  double xVal = 1.2;
  double yVal = 2.0;
  double zVal = 3.0;

  instance->set(x, &xVal);
  instance->set(y, &yVal);
  instance->set(z, &zVal);

  ASSERT_EQ(xVal, instance->getValue<double>(x));
  ASSERT_EQ(yVal, instance->getValue<double>(y));
  ASSERT_EQ(zVal, instance->getValue<double>(z));

  auto spec = std::make_shared<ice::InformationSpecification>(
      "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Mops",
      "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Robot",
      "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Position",
      "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#CoordinatePositionRep"
      );
  auto element = std::make_shared<ice::InformationElement<ice::GContainer>>(spec, instance);
  mops.informationStore->addInformation(spec, element);


  // sleep some time and let the discovery happen
  sleep(3);

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

  // check number of found identities, 1 default, 1 self, 1 other = 3
  ASSERT_EQ(3, mops.identityDirectory->count());
  ASSERT_EQ(3, zwerg.identityDirectory->count());

  // check information store
  // TODO
}
