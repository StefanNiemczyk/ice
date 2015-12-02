#include <gtest/gtest.h>
#include "ClingWrapper.h"
#include "External.h"
#include "BaseLiteral.h"
#include "BoolLiteral.h"
#include "clasp/solver.h"
#include <chrono>
#include <map>
#include <vector>
#include <ros/package.h>

#include "ice/ontology/OntologyInterface.h"
#include "ice/representation/ASPTransformationGeneration.h"
#include "ice/representation/Representation.h"
#include "ice/representation/GContainer.h"
#include "ice/representation/GContainerFactory.h"
#include "ice/representation/Transformation.h"
#include "ice/representation/XMLTransformationReader.h"

TEST(ASPRepComp, simple)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("data/asp/ontology.lp");
  cw->addKnowledgeFile("../asp/transformation/computing.lp");
  cw->init();

  cw->add("query", {}, "#external iro(system1,coords2Wgs84,any,none).");
  auto coords2Wgs84 = cw->getExternal("iro", {"system1", "coords2Wgs84", "any", "none"}, "coords2Wgs84", {}, true);
  cw->add("query", {}, "input(system1,coords2Wgs84,velocity,vel1,none,1,1) :- iro(system1,coords2Wgs84,any,none).");
  cw->add("query", {}, "output(system1,coords2Wgs84,velocity,vel2,none).");
  cw->ground("query", {});

  auto query1 = cw->getExternal("query", {1}, "query", {1, 3, 10}, true);

  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(true, cw->query("hasRelatedEntity(relCoords)"));
  EXPECT_EQ(true, cw->query("simpleRep(coords)"));
  EXPECT_EQ(true, cw->query("simpleRep(coords2D)"));

  EXPECT_EQ(true, cw->query("simRep(position,coords,coords2D)"));
  EXPECT_EQ(true, cw->query("dimensionDeviation(simRep(position,coords,coords2D),z,miss)"));
  EXPECT_EQ(true, cw->query("fix(simRep(position,coords,coords2D),z,remove)"));
  EXPECT_EQ(true, cw->query("autoIRO(position,coords,coords2D)"));

  EXPECT_EQ(true, cw->query("simRep(position,coords2D,coords)"));
  EXPECT_EQ(true, cw->query("dimensionDeviation(simRep(position,coords2D,coords),z,empty)"));
  EXPECT_EQ(true, cw->query("fix(simRep(position,coords2D,coords),z,default)"));
  EXPECT_EQ(true, cw->query("autoIRO(position,coords2D,coords)"));

  EXPECT_EQ(true, cw->query("match(simRep(position,coords2D,coords),x)"));
  EXPECT_EQ(true, cw->query("match(simRep(position,coords2D,coords),y)"));
  EXPECT_EQ(false, cw->query("match(simRep(position,coords2D,coords),z)"));

  EXPECT_EQ(false, cw->query("autoIRO(position,relCoords,coords)"));

  EXPECT_EQ(true, cw->query("simpleIro(coords2Wgs84,velocity,vel1,vel2)"));
}

TEST(ASPRepComp, ontology)
{
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

  std::shared_ptr<ice::GContainerFactory> factory = std::make_shared<ice::GContainerFactory>();
  factory->setOntologyInterface(oi);
  factory->init();

  ice::ASPTransformationGeneration asp;

  asp.setOntology(oi);
  asp.setGContainerFactory(factory);

  asp.readInfoStructureFromOntology();// TODO
  asp.extractTransformations();

  auto trans = factory->getTransformation("autoTrans_o2_Position-o2_CoordinatePositionRep-o0_Pos3D");

  ASSERT_TRUE(trans != false);

  auto repIn = factory->getRepresentation("o2_CoordinatePositionRep");
  auto repOut = factory->getRepresentation("o0_Pos3D");

  ASSERT_TRUE(repIn != false);
  ASSERT_TRUE(repOut != false);

  auto inX = repIn->accessPath({"o2_XCoordinate"});
  auto inY = repIn->accessPath({"o2_YCoordinate"});
  auto inZ = repIn->accessPath({"o2_ZCoordinate"});

  auto outX = repOut->accessPath({"o2_XCoordinate"});
  auto outY = repOut->accessPath({"o2_YCoordinate"});
  auto outZ = repOut->accessPath({"o2_ZCoordinate"});

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

  auto out = trans->transform(&in);

  EXPECT_EQ(out->getValue<double>(outX), x);
  EXPECT_EQ(out->getValue<double>(outY), y);
  EXPECT_EQ(out->getValue<double>(outZ), z);
}
