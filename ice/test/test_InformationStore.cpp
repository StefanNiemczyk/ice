#include <gtest/gtest.h>
#include "ClingWrapper.h"
#include "External.h"
#include "BaseLiteral.h"
#include "BoolLiteral.h"
#include "clasp/solver.h"
#include <chrono>
#include <map>
#include <memory>
#include <vector>
#include <ros/package.h>

#include "ice/ontology/OntologyInterface.h"
#include "ice/representation/ASPTransformationGeneration.h"
#include "ice/representation/Representation.h"
#include "ice/representation/GContainer.h"
#include "ice/representation/GContainerFactory.h"
#include "ice/representation/Transformation.h"
#include "ice/representation/XMLTransformationReader.h"
#include "ice/information/InformationElement.h"
#include "ice/information/InformationStore.h"
#include "ice/information/InformationSpecification.h"
#include "ice/processing/Node.h"

TEST(InformationStore, simpleTest)
{
  ice::Node::clearNodeStore();

  std::string path = ros::package::getPath("ice");
  bool result;

  auto oi = std::make_shared<ice::OntologyInterface>(path + "/java/lib/");
  oi->addIRIMapper(path + "/ontology/");

  ASSERT_FALSE(oi->errorOccurred());

  result = oi->addOntologyIRI("http://vs.uni-kassel.de/Ice");

  ASSERT_FALSE(oi->errorOccurred());
  ASSERT_TRUE(result);

  result = oi->loadOntologies();

  ASSERT_FALSE(oi->errorOccurred());
  ASSERT_TRUE(result);

  result = oi->isConsistent();

  ASSERT_FALSE(oi->errorOccurred());
  ASSERT_TRUE(result);

  ice::GContainerFactory fac;
  fac.setOntologyInterface(oi);
  fac.init();

  auto rep = fac.getRepresentation("http://vs.uni-kassel.de/Ice#DefaultMovementRep");

  ASSERT_TRUE(rep != false);

  auto movement = fac.makeInstance(rep);

  const double testVal = 4.2f;
  auto pos = rep->accessPath({"http://vs.uni-kassel.de/Ice#Translation"});

  ASSERT_TRUE(pos != nullptr);

  movement->set(pos, &testVal);

  auto store = std::make_shared<ice::InformationStore>();
  auto spec = std::make_shared<ice::InformationSpecification>("entity", "entityType", "spec", "rep");

  store->addInformation(spec, movement);

  std::vector<std::shared_ptr<ice::InformationElement<ice::GContainer>>> outInfo;
  result = store->getInformation(spec, outInfo, false);

  ASSERT_TRUE(result);
  ASSERT_EQ(1, outInfo.size());

  double val = outInfo.at(0)->getInformation()->getValue<double>(pos);
  ASSERT_EQ(testVal, val);
}

TEST(InformationStore, ontology1)
{
  ice::Node::clearNodeStore();

  std::string path = ros::package::getPath("ice");
  bool result;

  auto oi = std::make_shared<ice::OntologyInterface>(path + "/java/lib/");
  oi->addIRIMapper(path + "/ontology/");

  ASSERT_FALSE(oi->errorOccurred());

  result = oi->addOntologyIRI("http://vs.uni-kassel.de/IceTest");

  ASSERT_FALSE(oi->errorOccurred());
  ASSERT_TRUE(result);

  result = oi->loadOntologies();

  ASSERT_FALSE(oi->errorOccurred());
  ASSERT_TRUE(result);

  result = oi->isConsistent();

  ASSERT_FALSE(oi->errorOccurred());
  ASSERT_TRUE(result);

  auto factory = std::make_shared<ice::GContainerFactory>();
  factory->setOntologyInterface(oi);
  factory->init();

  ice::ASPTransformationGeneration asp;

  asp.setOntology(oi);
  asp.setGContainerFactory(factory);

  asp.extractTransformations();

  // Test transformation
  std::string name = "autoTrans_http://vs.uni-kassel.de/Ice#Position_http://vs.uni-kassel.de/Ice#CoordinatePositionRep_http://vs.uni-kassel.de/IceTest#Pos3D";
  auto trans = factory->getTransformationByName(name);

  ASSERT_TRUE(trans != false);

  auto repIn = factory->getRepresentation("http://vs.uni-kassel.de/Ice#CoordinatePositionRep");
  auto repOut = factory->getRepresentation("http://vs.uni-kassel.de/IceTest#Pos3D");

  ASSERT_TRUE(repIn != false);
  ASSERT_TRUE(repOut != false);

  auto inX = repIn->accessPath( {"http://vs.uni-kassel.de/Ice#XCoordinate"});
  auto inY = repIn->accessPath( {"http://vs.uni-kassel.de/Ice#YCoordinate"});
  auto inZ = repIn->accessPath( {"http://vs.uni-kassel.de/Ice#ZCoordinate"});

  auto outX = repOut->accessPath( {"http://vs.uni-kassel.de/Ice#XCoordinate"});
  auto outY = repOut->accessPath( {"http://vs.uni-kassel.de/Ice#YCoordinate"});
  auto outZ = repOut->accessPath( {"http://vs.uni-kassel.de/Ice#ZCoordinate"});

  ASSERT_TRUE(inX != nullptr);
  ASSERT_TRUE(inY != nullptr);
  ASSERT_TRUE(inZ != nullptr);

  ASSERT_TRUE(outX != nullptr);
  ASSERT_TRUE(outY != nullptr);
  ASSERT_TRUE(outZ != nullptr);

  auto in = factory->makeInstance(repIn);

  double x = 4.44;
  double y = 5.55;
  double z = 6.66;

  in->set(inX, &x);
  in->set(inY, &y);
  in->set(inZ, &z);

  auto store = std::make_shared<ice::InformationStore>();
  store->setGContainerFactory(factory);

  auto specIn = std::make_shared<ice::InformationSpecification>("entity", "entityType", "spec", "http://vs.uni-kassel.de/Ice#CoordinatePositionRep");
  auto specOut = std::make_shared<ice::InformationSpecification>("entity", "entityType", "spec", "http://vs.uni-kassel.de/IceTest#Pos3D");

  store->addInformation(specIn, in);

  std::vector<std::shared_ptr<ice::InformationElement<ice::GContainer>>> outInfo;
  int count = store->getInformation(specOut, outInfo, false);

  ASSERT_EQ(0, count);
  ASSERT_EQ(0, outInfo.size());

  count = store->getInformation(specOut, outInfo, true);

  ASSERT_EQ(1, count);
  ASSERT_EQ(1, outInfo.size());

  auto out = outInfo.at(0)->getInformation();

  EXPECT_EQ(out->getValue<double>(outX), x);
  EXPECT_EQ(out->getValue<double>(outY), y);
  EXPECT_EQ(out->getValue<double>(outZ), z);
}
