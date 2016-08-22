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

  EXPECT_EQ(true, cw->query("-simpleRep(relCoords)"));
  EXPECT_EQ(true, cw->query("simpleRep(coords)"));
  EXPECT_EQ(true, cw->query("simpleRep(coords2D)"));

  EXPECT_EQ(true, cw->query("simRep(position,coords,coords2D)"));
  EXPECT_EQ(true, cw->query("dimensionDeviation(simRep(position,coords,coords2D),z,floatRep,miss)"));
  EXPECT_EQ(true, cw->query("fix(simRep(position,coords,coords2D),z,floatRep,remove)"));
  EXPECT_EQ(true, cw->query("autoIRO(position,coords,coords2D)"));

  EXPECT_EQ(true, cw->query("simRep(position,coords2D,coords)"));
  EXPECT_EQ(true, cw->query("dimensionDeviation(simRep(position,coords2D,coords),z,floatRep,empty)"));
  EXPECT_EQ(true, cw->query("fix(simRep(position,coords2D,coords),z,floatRep,default)"));
  EXPECT_EQ(true, cw->query("autoIRO(position,coords2D,coords)"));

  EXPECT_EQ(true, cw->query("match(simRep(position,coords2D,coords),x,floatRep)"));
  EXPECT_EQ(true, cw->query("match(simRep(position,coords2D,coords),y,floatRep)"));
  EXPECT_EQ(false, cw->query("match(simRep(position,coords2D,coords),z,floatRep)"));

  EXPECT_EQ(false, cw->query("autoIRO(position,relCoords,coords)"));

  EXPECT_EQ(true, cw->query("simpleIro(coords2Wgs84,velocity,vel1,vel2)"));
}

TEST(ASPRepComp, ontology1)
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

  asp.extractTransformations();

  // Test transformation
  std::string name = "autoTrans_http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Position_http://www.semanticweb.org/sni/ontologies/2013/7/Ice#CoordinatePositionRep_http://vs.uni-kassel.de/IceTest#Pos3D";
  auto trans = factory->getTransformationByName(name);

  ASSERT_TRUE(trans != false);

  auto repIn = factory->getRepresentation("http://www.semanticweb.org/sni/ontologies/2013/7/Ice#CoordinatePositionRep");
  auto repOut = factory->getRepresentation("http://vs.uni-kassel.de/IceTest#Pos3D");

  ASSERT_TRUE(repIn != false);
  ASSERT_TRUE(repOut != false);

  auto inX = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#XCoordinate"});
  auto inY = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#YCoordinate"});
  auto inZ = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#ZCoordinate"});

  auto outX = repOut->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#XCoordinate"});
  auto outY = repOut->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#YCoordinate"});
  auto outZ = repOut->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#ZCoordinate"});

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

TEST(ASPRepComp, ontology2)
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

  asp.extractTransformations();

  // Test transformation
  std::string name = "autoTrans_http://vs.uni-kassel.de/IceTest#TestScope1_http://vs.uni-kassel.de/IceTest#TestTransformation1_http://vs.uni-kassel.de/IceTest#TestTransformation2";
  auto trans = factory->getTransformationByName(name);

  ASSERT_TRUE(trans != false);

  auto repIn = factory->getRepresentation("http://vs.uni-kassel.de/IceTest#TestTransformation1");
  auto repOut = factory->getRepresentation("http://vs.uni-kassel.de/IceTest#TestTransformation2");

  ASSERT_TRUE(repIn != false);
  ASSERT_TRUE(repOut != false);

  auto inO = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Orientation"});
  auto inP = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Position"});
  auto inX = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Position", "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#XCoordinate"});
  auto inY = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Position", "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#YCoordinate"});
  auto inZ = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Position", "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#ZCoordinate"});

  auto inOa = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Orientation", "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Alpha"});
  auto inOb = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Orientation", "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Beta"});
  auto inOc = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Orientation", "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Gamma"});

  ASSERT_TRUE(inO != nullptr);
  ASSERT_TRUE(inP != nullptr);
  ASSERT_TRUE(inX != nullptr);
  ASSERT_TRUE(inY != nullptr);
  ASSERT_TRUE(inZ != nullptr);

  ASSERT_TRUE(inOa != nullptr);
  ASSERT_TRUE(inOb != nullptr);
  ASSERT_TRUE(inOc != nullptr);

  auto outO = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Orientation"});
  auto outP = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Position"});
  auto outX = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Position", "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#XCoordinate"});
  auto outY = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Position", "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#YCoordinate"});
  auto outZ = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Position", "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#ZCoordinate"});

  auto outOa = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Orientation", "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Alpha"});
  auto outOb = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Orientation", "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Beta"});
  auto outOc = repIn->accessPath( {"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Orientation", "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Gamma"});

  ASSERT_TRUE(outO != nullptr);
  ASSERT_TRUE(outP != nullptr);
  ASSERT_TRUE(outX != nullptr);
  ASSERT_TRUE(outY != nullptr);
  ASSERT_TRUE(outZ != nullptr);

  ASSERT_TRUE(outOa != nullptr);
  ASSERT_TRUE(outOb != nullptr);
  ASSERT_TRUE(outOc != nullptr);

  auto in = factory->makeInstance(repIn);

  double x = 4.44;
  double y = 5.55;
  double z = 6.66;

  double a = 14.44;
  double b = 15.55;
  double c = 16.66;

  in->set(inX, &x);
  in->set(inY, &y);
  in->set(inZ, &z);

  in->set(inOa, &a);
  in->set(inOb, &b);
  in->set(inOc, &c);

  auto out = trans->transform(&in);

  EXPECT_EQ(out->getValue<double>(outX), x);
  EXPECT_EQ(out->getValue<double>(outY), y);
  EXPECT_EQ(out->getValue<double>(outZ), z);

  EXPECT_EQ(out->getValue<double>(outOa), a);
  EXPECT_EQ(out->getValue<double>(outOb), b);
  EXPECT_EQ(out->getValue<double>(outOc), c);

  auto aspStr = trans->getASPRepreentation("system");
  for (auto s : *aspStr)
  {
    if (s == "iro(system,autoTrans_http://vs.uni-kassel.de/IceTest#TestScope1_http://vs.uni-kassel.de/IceTest#TestTransformation1_http://vs.uni-kassel.de/IceTest#TestTransformation2,any,none).")
    {
      // fine
    }
    else if (s
        == "output(system,autoTrans_http://vs.uni-kassel.de/IceTest#TestScope1_http://vs.uni-kassel.de/IceTest#TestTransformation1_http://vs.uni-kassel.de/IceTest#TestTransformation2,http://vs.uni-kassel.de/IceTest#TestScope1,http://vs.uni-kassel.de/IceTest#TestTransformation2,none).")
    {
      // fine
    }
    else if (s
        == "input(system,autoTrans_http://vs.uni-kassel.de/IceTest#TestScope1_http://vs.uni-kassel.de/IceTest#TestTransformation1_http://vs.uni-kassel.de/IceTest#TestTransformation2,http://vs.uni-kassel.de/IceTest#TestScope1,http://vs.uni-kassel.de/IceTest#TestTransformation1,none,1,1) :- iro(system,autoTrans_http://vs.uni-kassel.de/IceTest#TestScope1_http://vs.uni-kassel.de/IceTest#TestTransformation1_http://vs.uni-kassel.de/IceTest#TestTransformation2,any,none).")
    {
      // fine
    }
    else
    {
      ASSERT_ANY_THROW() << "Unknown string " + s;
    }
  }
}
