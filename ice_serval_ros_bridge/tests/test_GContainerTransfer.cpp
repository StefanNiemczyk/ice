/*
 * test_GContainerTransfer.cpp
 *
 *  Created on: Dec 7, 2016
 *      Author: sni
 */

#include <gtest/gtest.h>

#include "ice/ICEngine.h"
#include "ice/information/InformationSpecification.h"
#include "ice/information/KnowledgeBase.h"
#include "ice/information/StreamStore.h"
#include "ice/information/SetStore.h"
#include "ice/model/aspModel/ASPModelGenerator.h"
#include "ice/model/updateStrategie/UpdateStrategie.h"
#include "ice/representation/GContainer.h"

#include "ServalCommunication.h"
#include "IceServalBridge.h"

class SimpleSourceNode : public ice::Node
{
public:
  static std::shared_ptr<ice::Node> createNode()
  {
    return std::make_shared<SimpleSourceNode>();
  }

  SimpleSourceNode() :
      Node()
  {

  }

  virtual std::string getClassName()
  {
    return "SimpleSourceNode";
  }

  virtual int performNode()
  {
    return 0;
  }
};

TEST(GContainerTransfer, transferSimple)
{
  ice::Node::clearNodeStore();
  ice::Node::registerNodeCreator("Pos3DSourceNode", &SimpleSourceNode::createNode);

  ice::InitParams *params1 = new ice::InitParams();
  ice::InitParams *params2 = new ice::InitParams();

  params1->servalInstancePath = "/tmp/mops";
  params1->servalHost = "localhost";
  params1->servalPort = 4110;
  params1->servalUser = "peter";
  params1->servalPassword = "venkman";
  params1->servalLocal = true;

  params2->servalInstancePath = "/tmp/mops";
  params2->servalHost = "localhost";
  params2->servalPort = 4110;
  params2->servalUser = "peter";
  params2->servalPassword = "venkman";
  params2->servalLocal = true;

  // create engine 1
  auto timeFactory = std::make_shared<ice::SimpleTimeFactory>();
  std::shared_ptr<ice::Configuration> config = std::make_shared<ice::Configuration>();
  config->ontologyIri = "http://vs.uni-kassel.de/IceTest";
  config->ontologyIriOwnEntity = "http://vs.uni-kassel.de/IceTest#TestGContainerTransfer_System1Ind";
  std::shared_ptr<ice::ICEngine> engine = std::make_shared<ice::ICEngine>(config);
  engine->setTimeFactory(timeFactory);
  engine->setCommunicationInterface(std::make_shared<ice::ServalCommunication>(engine,
                                                             params1->servalInstancePath,
                                                             params1->servalHost,
                                                             params1->servalPort,
                                                             params1->servalUser,
                                                             params1->servalPassword,
                                                             params1->servalLocal));

  engine->getConfig()->synthesizeTransformations = false;
  engine->getConfig()->generateInformationProcessing = false;

  engine->init();
  engine->start();

  // create engine 2
  timeFactory = std::make_shared<ice::SimpleTimeFactory>();

  std::shared_ptr<ice::Configuration> config2 = std::make_shared<ice::Configuration>();
  config2->ontologyIri = "http://vs.uni-kassel.de/IceTest";
  config2->ontologyIriOwnEntity = "http://vs.uni-kassel.de/IceTest#TestGContainerTransfer_System2Ind";
  std::shared_ptr<ice::ICEngine> engine2 = std::make_shared<ice::ICEngine>(config2);
  engine2->setTimeFactory(timeFactory);
  engine2->setCommunicationInterface(std::make_shared<ice::ServalCommunication>(engine2,
                                                             params2->servalInstancePath,
                                                             params2->servalHost,
                                                             params2->servalPort,
                                                             params2->servalUser,
                                                             params2->servalPassword,
                                                             params2->servalLocal));

  engine2->getConfig()->synthesizeTransformations = false;
  engine2->init();
  engine2->start();

  // wait some time to enable the engines to find each other
  std::this_thread::sleep_for(std::chrono::milliseconds {5000});

  // initializing stuff
  auto specPos3D = ice::InformationSpecification("http://vs.uni-kassel.de/IceTest#TestEntity1",
                                               "http://vs.uni-kassel.de/IceTest#TestEntity",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/IceTest#Pos3D");
  auto specPos2D = ice::InformationSpecification("http://vs.uni-kassel.de/IceTest#TestEntity1",
                                             "http://vs.uni-kassel.de/IceTest#TestEntity",
                                             "http://vs.uni-kassel.de/Ice#Position",
                                             "http://vs.uni-kassel.de/IceTest#Pos2D");


  //  auto p2dRep = engine->getGContainerFactory()->getRepresentation("http://vs.uni-kassel.de/IceTest#Pos2D");
    auto p3dRep = engine->getGContainerFactory()->getRepresentation("http://vs.uni-kassel.de/IceTest#Pos3D");

  //  ASSERT_TRUE(p2dRep != false);
    ASSERT_TRUE(p3dRep != false);

  //  auto p2dX = p2dRep->accessPath( {"http://vs.uni-kassel.de/Ice#XCoordinate"});
  //  auto p2dY = p2dRep->accessPath( {"http://vs.uni-kassel.de/Ice#YCoordinate"});

    auto p3dX = p3dRep->accessPath( {"http://vs.uni-kassel.de/Ice#XCoordinate"});
    auto p3dY = p3dRep->accessPath( {"http://vs.uni-kassel.de/Ice#YCoordinate"});
    auto p3dZ = p3dRep->accessPath( {"http://vs.uni-kassel.de/Ice#ZCoordinate"});


  // test processing
  auto stream1 = engine->getKnowlegeBase()->streamStore->getStream<ice::GContainer>(&specPos3D, "http://vs.uni-kassel.de/IceTest#Pos3DSourceNodeInd",
                                                                         "http://vs.uni-kassel.de/IceTest#TestGContainerTransfer_System1Ind");
  auto stream2 = engine2->getKnowlegeBase()->streamStore->getStream<ice::GContainer>(&specPos3D, "http://vs.uni-kassel.de/IceTest#Pos3DSourceNodeInd",
                                                                         "http://vs.uni-kassel.de/IceTest#TestGContainerTransfer_System1Ind");

  ASSERT_TRUE((stream1 ? true : false));
  ASSERT_TRUE((stream2 ? true : false));

  // generate new element
  auto element = engine->getGContainerFactory()->makeInstance("http://vs.uni-kassel.de/IceTest#Pos3D");

  double x = 5;
  double y = 6;
  double z = 7;

  element->set(p3dX, &x);
  element->set(p3dY, &y);
  element->set(p3dZ, &z);

  stream1->add(element);

  sleep(2);

  auto out = stream2->getLast();

  ASSERT_NE(nullptr, out);
  ASSERT_EQ(x, out->getInformation()->getValue<double>(p3dX));
  ASSERT_EQ(y, out->getInformation()->getValue<double>(p3dY));
  ASSERT_EQ(z, out->getInformation()->getValue<double>(p3dZ));

  engine->cleanUp();
  engine.reset();
  engine2->cleanUp();
  engine2.reset();
}

TEST(GContainerTransfer, transferAndTransform)
{
  ice::Node::clearNodeStore();
  ice::Node::registerNodeCreator("Pos3DSourceNode", &SimpleSourceNode::createNode);

  ice::InitParams *params1 = new ice::InitParams();
  ice::InitParams *params2 = new ice::InitParams();

  params1->servalInstancePath = "/tmp/mops";
  params1->servalHost = "localhost";
  params1->servalPort = 4110;
  params1->servalUser = "peter";
  params1->servalPassword = "venkman";
  params1->servalLocal = true;

  params2->servalInstancePath = "/tmp/mops";
  params2->servalHost = "localhost";
  params2->servalPort = 4110;
  params2->servalUser = "peter";
  params2->servalPassword = "venkman";
  params2->servalLocal = true;

  // create engine 1
  auto timeFactory = std::make_shared<ice::SimpleTimeFactory>();
  std::shared_ptr<ice::Configuration> config = std::make_shared<ice::Configuration>();
  config->ontologyIri = "http://vs.uni-kassel.de/IceTest";
  config->ontologyIriOwnEntity = "http://vs.uni-kassel.de/IceTest#TestGContainerTransfer2_System1Ind";
  std::shared_ptr<ice::ICEngine> engine = std::make_shared<ice::ICEngine>(config);
  engine->setTimeFactory(timeFactory);
  engine->setCommunicationInterface(std::make_shared<ice::ServalCommunication>(engine,
                                                             params1->servalInstancePath,
                                                             params1->servalHost,
                                                             params1->servalPort,
                                                             params1->servalUser,
                                                             params1->servalPassword,
                                                             params1->servalLocal));

  engine->getConfig()->synthesizeTransformations = false;
  engine->getConfig()->generateInformationProcessing = false;
  engine->init();
  engine->start();

  // create engine 2
  timeFactory = std::make_shared<ice::SimpleTimeFactory>();

  std::shared_ptr<ice::Configuration> config2 = std::make_shared<ice::Configuration>();
  config2->ontologyIri = "http://vs.uni-kassel.de/IceTest";
  config2->ontologyIriOwnEntity = "http://vs.uni-kassel.de/IceTest#TestGContainerTransfer2_System2Ind";
  std::shared_ptr<ice::ICEngine> engine2 = std::make_shared<ice::ICEngine>(config2);
  engine2->setTimeFactory(timeFactory);
  engine2->setCommunicationInterface(std::make_shared<ice::ServalCommunication>(engine2,
                                                             params1->servalInstancePath,
                                                             params1->servalHost,
                                                             params1->servalPort,
                                                             params1->servalUser,
                                                             params1->servalPassword,
                                                             params1->servalLocal));

  engine2->init();
  engine2->start();

  // wait some time to enable the engines to find each other
  std::this_thread::sleep_for(std::chrono::milliseconds {2000});

  // initializing stuff
  auto specPos3D = ice::InformationSpecification("http://vs.uni-kassel.de/IceTest#TestEntity1",
                                               "http://vs.uni-kassel.de/IceTest#TestEntity",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/IceTest#Pos3D");
  auto specPos2D = ice::InformationSpecification("http://vs.uni-kassel.de/IceTest#TestEntity1",
                                             "http://vs.uni-kassel.de/IceTest#TestEntity",
                                             "http://vs.uni-kassel.de/Ice#Position",
                                             "http://vs.uni-kassel.de/IceTest#Pos2D");


    auto p2dRep = engine->getGContainerFactory()->getRepresentation("http://vs.uni-kassel.de/IceTest#Pos2D");
    auto p3dRep = engine->getGContainerFactory()->getRepresentation("http://vs.uni-kassel.de/IceTest#Pos3D");

    ASSERT_TRUE(p2dRep != false);
    ASSERT_TRUE(p3dRep != false);

    auto p2dX = p2dRep->accessPath( {"http://vs.uni-kassel.de/Ice#XCoordinate"});
    auto p2dY = p2dRep->accessPath( {"http://vs.uni-kassel.de/Ice#YCoordinate"});

    auto p3dX = p3dRep->accessPath( {"http://vs.uni-kassel.de/Ice#XCoordinate"});
    auto p3dY = p3dRep->accessPath( {"http://vs.uni-kassel.de/Ice#YCoordinate"});
    auto p3dZ = p3dRep->accessPath( {"http://vs.uni-kassel.de/Ice#ZCoordinate"});


  // test processing
  auto stream1 = engine->getKnowlegeBase()->streamStore->getStream<ice::GContainer>(&specPos3D, "http://vs.uni-kassel.de/IceTest#Pos3DSourceNodeInd",
                                                                         "http://vs.uni-kassel.de/IceTest#TestGContainerTransfer2_System1Ind");
  auto stream2 = engine2->getKnowlegeBase()->streamStore->getStream<ice::GContainer>(&specPos2D);

  ASSERT_TRUE((stream1 ? true : false));
  ASSERT_TRUE((stream2 ? true : false));

  // generate new element
  auto element = engine->getGContainerFactory()->makeInstance("http://vs.uni-kassel.de/IceTest#Pos3D");

  double x = 5;
  double y = 6;
  double z = 7;

  element->set(p3dX, &x);
  element->set(p3dY, &y);
  element->set(p3dZ, &z);

  stream1->add(element);

  sleep(2);

  auto out = stream2->getLast();

  ASSERT_NE(nullptr, out);
  ASSERT_EQ(x, out->getInformation()->getValue<double>(p2dX));
  ASSERT_EQ(y, out->getInformation()->getValue<double>(p2dY));

  engine->cleanUp();
  engine.reset();
  engine2->cleanUp();
  engine2.reset();
}
