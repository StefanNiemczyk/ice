/*
 * test_RepresentationTransformation.cpp
 *
 *  Created on: Nov 25, 2015
 *      Author: sni
 */

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

TEST(RepresentationTransformationTest, rollPitchYawToEulerAngles)
{
  std::string path = ros::package::getPath("ice");
  bool result;

  auto oi = std::make_shared<ice::OntologyInterface>(path + "/java/lib/");
  oi->addIRIMapper(path + "/ontology/");

  ASSERT_FALSE(oi->errorOccurred());

  result = oi->addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice");

  ASSERT_FALSE(oi->errorOccurred());
  ASSERT_TRUE(result);

  result = oi->loadOntologies();

  ASSERT_FALSE(oi->errorOccurred());
  ASSERT_TRUE(result);

  result = oi->isConsistent();

  ASSERT_FALSE(oi->errorOccurred());
  ASSERT_TRUE(result);

  std::shared_ptr<ice::GContainerFactory> factory = std::make_shared<ice::GContainerFactory>();
  factory->setOntologyInterface(oi);
  factory->init();

  ice::XMLTransformationReader reader;

  result = reader.readFile("data/euler_transformation.xml");

  ASSERT_TRUE(result);

  auto rollPitchYawRep = factory->getRepresentation("o0_RollPitchYawRep");
  auto eulerRep = factory->getRepresentation("o0_EulerAnglesRep");

  ASSERT_TRUE(rollPitchYawRep != false);

  auto roll = rollPitchYawRep->accessPath({"o0_Roll"});
  auto pitch = rollPitchYawRep->accessPath({"o0_Pitch"});
  auto yaw = rollPitchYawRep->accessPath({"o0_Yaw"});

  auto alpha = eulerRep->accessPath({"o0_Alpha"});
  auto beta = eulerRep->accessPath({"o0_Beta"});
  auto gamma = eulerRep->accessPath({"o0_Gamma"});

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

      auto euler = trans->transform(&rollPitchYaw);

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

