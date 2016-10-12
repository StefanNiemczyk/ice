/*
 * test_RepresentationTransformation.cpp
 *
 *  Created on: Nov 25, 2015
 *      Author: sni
 */

#include <chrono>
#include <memory>
#include <ros/package.h>

#include "ice/representation/Representation.h"
#include "ice/representation/GContainer.h"
#include "ice/representation/GContainerFactory.h"
#include "ice/representation/Transformation.h"
#include "ice/representation/XMLTransformationReader.h"

#include "ice/ICEngine.h"

#include "gtest/gtest.h"

namespace
{

TEST(EulerRepTransformation, rollPitchYawToEulerAngles)
{
  ice::Node::clearNodeStore();
  auto timeFactory = std::make_shared<ice::SimpleTimeFactory>();
  std::shared_ptr<ice::Configuration> config = std::make_shared<ice::Configuration>();
  config->ontologyIri = "http://vs.uni-kassel.de/IceTest";
  config->ontologyIriOwnEntity = "http://vs.uni-kassel.de/IceTest#TestSystem";
  std::shared_ptr<ice::ICEngine> engine = std::make_shared<ice::ICEngine>(config);
  engine->setTimeFactory(std::make_shared<ice::SimpleTimeFactory>());

  engine->init();
  auto factory = engine->getGContainerFactory();

  ice::XMLTransformationReader reader;

  auto result = reader.readFile("data/euler_transformation.xml");

  ASSERT_TRUE(result);

  auto rollPitchYawRep = factory->getRepresentation("http://vs.uni-kassel.de/Ice#RollPitchYawRep");
  auto eulerRep = factory->getRepresentation("http://vs.uni-kassel.de/Ice#EulerAnglesRep");

  ASSERT_TRUE(rollPitchYawRep != false);

  auto roll = rollPitchYawRep->accessPath({"http://vs.uni-kassel.de/Ice#Roll"});
  auto pitch = rollPitchYawRep->accessPath({"http://vs.uni-kassel.de/Ice#Pitch"});
  auto yaw = rollPitchYawRep->accessPath({"http://vs.uni-kassel.de/Ice#Yaw"});

  auto alpha = eulerRep->accessPath({"http://vs.uni-kassel.de/Ice#Alpha"});
  auto beta = eulerRep->accessPath({"http://vs.uni-kassel.de/Ice#Beta"});
  auto gamma = eulerRep->accessPath({"http://vs.uni-kassel.de/Ice#Gamma"});

  bool foundRollPitchYawRep = false;

  for (auto desc : reader.getTransformations())
  {
    auto trans = factory->fromXMLDesc(desc);
    ASSERT_TRUE(trans != false);

    if (trans->getName() == "RollPitchYawToEuler")
    {
      auto rollPitchYaw = factory->makeInstance(rollPitchYawRep);

      double valRoll = 0.6;
      double valPitch = 0.3;
      double valYaw = 0.7;

      rollPitchYaw->set(roll, &valRoll);
      rollPitchYaw->set(pitch, &valPitch);
      rollPitchYaw->set(yaw, &valYaw);

      std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

      auto euler = trans->transform(&rollPitchYaw);

      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
      std::cout << "Transformation took "
                << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()
                << "ns.\n";


      ASSERT_NE(euler->getValue<double>(yaw), valYaw);
      ASSERT_NE(euler->getValue<double>(pitch), valPitch);
      ASSERT_NE(euler->getValue<double>(roll), valRoll);
  
      foundRollPitchYawRep = true;
    }
    else
    {
      ASSERT_FALSE(true);
    }
  }

  ASSERT_TRUE(foundRollPitchYawRep);
}

}

