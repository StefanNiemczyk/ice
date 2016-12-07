/*
 * test_GPSTransformation.cpp
 *
 *  Created on: 18.09.2016
 *      Author: sni
 */

#include <gtest/gtest.h>
#include <map>

#include "ice/ICEngine.h"
#include "ice/information/InformationSpecification.h"
#include "ice/information/StreamStore.h"
#include "ice/model/aspModel/ASPModelGenerator.h"
#include "ice/model/updateStrategie/UpdateStrategie.h"
#include "ice/representation/GContainer.h"
#include "ice/representation/GContainerFactory.h"

#include "GPSPosition.h"


TEST(GPSTransformation, gpspostion)
{

  std::string path = ros::package::getPath("tb_knowledge_base");

  std::shared_ptr<ice::Configuration> config = std::make_shared<ice::Configuration>();
  config->ontologyIriMapper = path + "/ontology";
  config->ontologyIri = "http://vs.uni-kassel.de/TurtleBot";
  config->ontologyIriOwnEntity = "http://vs.uni-kassel.de/TurtleBot#Leonardo";

  std::shared_ptr<ice::ICEngine> engine = std::make_shared<ice::ICEngine>(config);

  auto timeFactory = std::make_shared<ice::SimpleTimeFactory>();
  auto streamFactory = std::make_shared<ice::CollectionFactory>(engine);
  engine->setTimeFactory(timeFactory);
  engine->setCollectionFactory(streamFactory);

  engine->init();

  auto factory = engine->getGContainerFactory();

  auto creator = [](std::shared_ptr<ice::GContainerFactory> factory){
    auto rep = factory->getRepresentation("http://vs.uni-kassel.de/Ice#WGS84Rep");
    return std::make_shared<ice::Pos3D>(rep);
  };
  bool result = factory->registerCustomCreator("http://vs.uni-kassel.de/Ice#WGS84Rep", creator);

  ASSERT_TRUE(result);

  auto rep = factory->getRepresentation("http://vs.uni-kassel.de/Ice#WGS84Rep");
  auto lat = rep->accessPath({"http://vs.uni-kassel.de/Ice#Latitude"});
  auto lon = rep->accessPath({"http://vs.uni-kassel.de/Ice#Longitude"});
  auto alt = rep->accessPath({"http://vs.uni-kassel.de/Ice#Altitude"});

  ASSERT_NE(nullptr, lat);
  ASSERT_NE(nullptr, lon);
  ASSERT_NE(nullptr, alt);

  auto pos1 = std::static_pointer_cast<ice::Pos3D>(factory->makeInstance(rep));

  pos1->x = 11.111;
  pos1->y = 22.222;
  pos1->z = 33.333;

  ASSERT_EQ(11.111, pos1->getValue<double>(lat));
  ASSERT_EQ(22.222, pos1->getValue<double>(lon));
  ASSERT_EQ(33.333, pos1->getValue<double>(alt));

  double dLat = 99;
  double dLon = 9990;
  double dAlt = 999999;

  pos1->set(lat, &dLat);
  pos1->set(lon, &dLon);
  pos1->set(alt, &dAlt);

  ASSERT_EQ(dLat, pos1->x);
  ASSERT_EQ(dLon, pos1->y);
  ASSERT_EQ(dAlt, pos1->z);
}


