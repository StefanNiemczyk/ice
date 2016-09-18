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


TEST(TransformationNode, simpleTest)
{
  std::shared_ptr<ice::Configuration> config = std::make_shared<ice::Configuration>();
  config->ontologyIri = "http://vs.uni-kassel.de/IceTest";
  config->ontologyIriOwnEntity = "http://vs.uni-kassel.de/IceTest#TestSystem";

  std::shared_ptr<ice::ICEngine> engine = std::make_shared<ice::ICEngine>(config);

  auto timeFactory = std::make_shared<ice::SimpleTimeFactory>();
  auto streamFactory = std::make_shared<ice::StreamFactory>(engine);
  engine->setTimeFactory(timeFactory);
  engine->setStreamFactory(streamFactory);

  engine->init();

  auto factory = engine->getGContainerFactory();

}


