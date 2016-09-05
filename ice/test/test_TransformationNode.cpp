/*
 * test_TransformationNode.cpp
 *
 *  Created on: 03.09.2016
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

#include "etc/EngineStuff.cpp"

TEST(TransformationNode, simpleTest)
{
  std::shared_ptr<ice::Configuration> config = std::make_shared<ice::Configuration>();
  config->ontologyIri = "http://vs.uni-kassel.de/IceTest";
  config->ontologyIriOwnEntity = "http://vs.uni-kassel.de/IceTest#TestSystem";

  std::shared_ptr<ice::ICEngine> engine = std::make_shared<ice::ICEngine>(config);

  auto timeFactory = std::make_shared<ice::SimpleTimeFactory>();
  auto streamFactory = std::make_shared<TestFactory>(engine);
  engine->setTimeFactory(timeFactory);
  engine->setStreamFactory(streamFactory);

  engine->init();

  auto factory = engine->getGContainerFactory();
  int count = factory->readXMLTransformation("data/transformation_example_1.xml");
  ASSERT_EQ(5, count);

  //
  auto p2dRep = factory->getRepresentation("http://vs.uni-kassel.de/IceTest#Pos2D");
  auto p3dRep = factory->getRepresentation("http://vs.uni-kassel.de/IceTest#Pos3D");

  ASSERT_TRUE(p2dRep != false);
  ASSERT_TRUE(p3dRep != false);

  auto p2dX = p2dRep->accessPath({"http://vs.uni-kassel.de/Ice#XCoordinate"});
  auto p2dY = p2dRep->accessPath({"http://vs.uni-kassel.de/Ice#YCoordinate"});

  auto p3dX = p3dRep->accessPath({"http://vs.uni-kassel.de/Ice#XCoordinate"});
  auto p3dY = p3dRep->accessPath({"http://vs.uni-kassel.de/Ice#YCoordinate"});
  auto p3dZ = p3dRep->accessPath({"http://vs.uni-kassel.de/Ice#ZCoordinate"});
  //

  // creating streams
//  std::string dataType,
//  std::shared_ptr<InformationSpecification> specification,
//  const std::string name, const int streamSize,
//  std::map<std::string, int> &metadata, std::string provider,
//  std::string sourceSystem
  auto spec = std::make_shared<ice::InformationSpecification>("test", "type", "scope", "http://vs.uni-kassel.de/IceTest#Pos2D");
  std::map<std::string, int> metadata;
  auto streamIn = engine->getStreamStore()->registerStream<ice::GContainer>(spec, "inStream", 100, metadata, "handmade", "this");
  ASSERT_NE(nullptr, streamIn);

  spec = std::make_shared<ice::InformationSpecification>("test", "type", "scope", "http://vs.uni-kassel.de/IceTest#Pos3D");
  auto streamOut = engine->getStreamStore()->registerStream<ice::GContainer>(spec, "outStream", 100, metadata, "handmade", "this");
  ASSERT_NE(nullptr, streamOut);

//  NodeType type,
//  const std::string className,
//  const std::string name,
//  const ont::entity entity,
//  const ont::entity entityRelated,

  std::map<std::string, std::string> nodeConfig;
  auto node = engine->getNodeStore()->registerNode(ice::NodeType::IRO,
                                                   "P2Dto3D",
                                                   "testnode",
                                                   "test",
                                                   "",
                                                   nodeConfig);
  ASSERT_NE(nullptr, node);

  node->addInput(streamIn, true);
  node->addOutput(streamOut);
  node->activate();
  ASSERT_TRUE(node->isValid());

  auto element = factory->makeInstance("http://vs.uni-kassel.de/IceTest#Pos2D");

  double x = 5;
  double y = 6;

  element->set(p2dX, &x);
  element->set(p2dY, &y);

  streamIn->add(element);

  sleep(2);

  auto out = streamOut->getLast();

  ASSERT_NE(nullptr, out);
  ASSERT_EQ(x, out->getInformation()->getValue<double>(p3dX));
  ASSERT_EQ(y, out->getInformation()->getValue<double>(p3dY));
  ASSERT_EQ(1, out->getInformation()->getValue<double>(p3dZ));

  engine->cleanUp();
}


