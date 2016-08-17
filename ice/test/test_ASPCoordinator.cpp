#include <gtest/gtest.h>

#include "ice/ICEngine.h"
#include "ice/information/InformationSpecification.h"
#include "ice/information/StreamStore.h"
#include "ice/model/aspModel/ASPModelGenerator.h"
#include "ice/model/updateStrategie/UpdateStrategie.h"

#include "etc/EngineStuff.cpp"

TEST(ASPModelGenerator, simpleTest)
{
  ice::Node::registerNodeCreator("TestSourceNodeGrounding", &SimpleSourceNode::createNode);
  ice::Node::registerNodeCreator("TestComputationalNodeGrounding", &SmothingNode::createNode);

  auto streamFactory = std::make_shared<TestFactory>();
  auto timeFactory = std::make_shared<ice::SimpleTimeFactory>();
  std::shared_ptr<ice::Configuration> config = std::make_shared<ice::Configuration>();
  config->ontologyIri = "http://vs.uni-kassel.de/IceTest";
  config->ontologyIriOwnEntity = "http://vs.uni-kassel.de/IceTest#TestSystem";
  std::shared_ptr<ice::ICEngine> engine = std::make_shared<ice::ICEngine>(config);
  engine->setTimeFactory(timeFactory);
  engine->setStreamFactory(streamFactory);

  engine->init();
  engine->start();

  auto spec1 = ice::InformationSpecification("http://vs.uni-kassel.de/IceTest#TestEntity1",
                                             "http://vs.uni-kassel.de/IceTest#TestEntity",
                                             "http://vs.uni-kassel.de/IceTest#TestScope1",
                                             "http://vs.uni-kassel.de/IceTest#TestRepresentation1");
  auto spec2 = ice::InformationSpecification("http://vs.uni-kassel.de/IceTest#TestEntity1",
                                             "http://vs.uni-kassel.de/IceTest#TestEntity",
                                             "http://vs.uni-kassel.de/IceTest#TestScope1",
                                             "http://vs.uni-kassel.de/IceTest#TestRepresentation2");

  auto stream1 = engine->getStreamStore()->getStream<ice::Position>(&spec1, "http://vs.uni-kassel.de/IceTest#TestSourceNodeInd",
                                                                         "http://vs.uni-kassel.de/IceTest#TestSystem");
  auto stream2 = engine->getStreamStore()->getStream<ice::Position>(&spec2, "http://vs.uni-kassel.de/IceTest#TestComputationalNodeInd",
                                                                         "http://vs.uni-kassel.de/IceTest#TestSystem");

  ASSERT_TRUE((stream1 ? true : false));
  ASSERT_TRUE((stream2 ? true : false));

  auto position1 = std::make_shared<ice::Position>();
  position1->x = 3;
  position1->y = 2;
  position1->z = 1;

  stream1->add(position1);

  std::this_thread::sleep_for(std::chrono::milliseconds {10});

  auto position2 = stream2->getLast();

  ASSERT_TRUE((position2 ? true : false));
  EXPECT_EQ(2, position2->getInformation()->x);
  EXPECT_EQ(1, position2->getInformation()->y);
  EXPECT_EQ(0, position2->getInformation()->z);
}

TEST(ASPModelGenerator, twoSystemsSimple)
{
  ice::Node::registerNodeCreator("TestSourceNodeGrounding", &SimpleSourceNode::createNode);
  ice::Node::registerNodeCreator("TestComputationalNodeGrounding", &SmothingNode::createNode);

  // create engine 1
  auto streamFactory = std::make_shared<TestFactory>();
  auto timeFactory = std::make_shared<ice::SimpleTimeFactory>();
  std::shared_ptr<ice::Configuration> config = std::make_shared<ice::Configuration>();
  config->ontologyIri = "http://vs.uni-kassel.de/IceTest";
  config->ontologyIriOwnEntity = "http://vs.uni-kassel.de/IceTest#TestCoordination1_SystemInd1";
  std::shared_ptr<ice::ICEngine> engine = std::make_shared<ice::ICEngine>(config);
  engine->setTimeFactory(timeFactory);
  engine->setStreamFactory(streamFactory);

  engine->init();
  engine->start();

  // create engine 2
  streamFactory = std::make_shared<TestFactory>();
  timeFactory = std::make_shared<ice::SimpleTimeFactory>();

  std::shared_ptr<ice::Configuration> config2 = std::make_shared<ice::Configuration>();
  config2->ontologyIri = "http://vs.uni-kassel.de/IceTest";
  config2->ontologyIriOwnEntity = "http://vs.uni-kassel.de/IceTest#TestCoordination1_SystemInd2";
  std::shared_ptr<ice::ICEngine> engine2 = std::make_shared<ice::ICEngine>(config2);
  engine2->setTimeFactory(timeFactory);
  engine2->setStreamFactory(streamFactory);

  engine2->init();
  engine2->start();

  // wait some time to enable the engines to find each other
  std::this_thread::sleep_for(std::chrono::milliseconds {2000});

  // test processing system 1
  auto spec1 = ice::InformationSpecification("http://vs.uni-kassel.de/IceTest#TestEntity1",
                                             "http://vs.uni-kassel.de/IceTest#TestEntity",
                                             "http://vs.uni-kassel.de/IceTest#TestScope1",
                                             "http://vs.uni-kassel.de/IceTest#TestRepresentation1");
  auto stream1 = engine->getStreamStore()->getStream<ice::Position>(&spec1,
                                                                    "http://vs.uni-kassel.de/IceTest#TestSourceNodeInd",
                                                                    "http://vs.uni-kassel.de/IceTest#TestCoordination1_SystemInd1");
  ASSERT_TRUE((stream1 ? true : false));

  // test processing system 2
  auto spec2 = ice::InformationSpecification("http://vs.uni-kassel.de/IceTest#TestEntity1",
                                             "http://vs.uni-kassel.de/IceTest#TestEntity",
                                             "http://vs.uni-kassel.de/IceTest#TestScope1",
                                             "http://vs.uni-kassel.de/IceTest#TestRepresentation1");
  auto stream2 = engine2->getStreamStore()->getStream<ice::Position>(&spec2,
                                                                     "http://vs.uni-kassel.de/IceTest#TestSourceNodeInd",
                                                                     "http://vs.uni-kassel.de/IceTest#TestCoordination1_SystemInd1");
  ASSERT_TRUE((stream2 ? true : false));

  // insert element in stream of system 2
  int x = rand();
  int y = rand();
  int z = rand();

  auto position1 = std::make_shared<ice::Position>();
  position1->x = x;
  position1->y = y;
  position1->z = z;

  stream1->add(position1);

  // wait some time
  std::this_thread::sleep_for(std::chrono::milliseconds {500});

  // check if element was send to system 2 and placed in stream2
  auto position2 = stream2->getLast();

  ASSERT_NE(nullptr, position2);
  EXPECT_EQ(x, position2->getInformation()->x);
  EXPECT_EQ(y, position2->getInformation()->y);
  EXPECT_EQ(z, position2->getInformation()->z);
}

TEST(ASPModelGenerator, twoSystemsComplex)
{
  ice::Node::registerNodeCreator("TestSourceNodeGrounding", &SimpleSourceNode::createNode);
  ice::Node::registerNodeCreator("TestComputationalNodeGrounding", &SmothingNode::createNode);

  // create engine 1
  auto streamFactory = std::make_shared<TestFactory>();
  auto timeFactory = std::make_shared<ice::SimpleTimeFactory>();

  std::shared_ptr<ice::Configuration> config = std::make_shared<ice::Configuration>();
  config->ontologyIri = "http://vs.uni-kassel.de/IceTest";
  config->ontologyIriOwnEntity = "http://vs.uni-kassel.de/IceTest#TestCoordination2_SystemInd1";
  std::shared_ptr<ice::ICEngine> engine = std::make_shared<ice::ICEngine>(config);
  engine->setTimeFactory(timeFactory);
  engine->setStreamFactory(streamFactory);

  engine->init();
  engine->start();

  // create engine 2
  streamFactory = std::make_shared<TestFactory>();
  timeFactory = std::make_shared<ice::SimpleTimeFactory>();

  std::shared_ptr<ice::Configuration> config2 = std::make_shared<ice::Configuration>();
  config2->ontologyIri = "http://vs.uni-kassel.de/IceTest";
  config2->ontologyIriOwnEntity = "http://vs.uni-kassel.de/IceTest#TestCoordination2_SystemInd2";
  std::shared_ptr<ice::ICEngine> engine2 = std::make_shared<ice::ICEngine>(config2);
  engine2->setTimeFactory(timeFactory);
  engine2->setStreamFactory(streamFactory);

  engine2->init();
  engine2->start();

  // wait some time to enable the engines to find each other
  std::this_thread::sleep_for(std::chrono::milliseconds {2000});

  auto spec1 = ice::InformationSpecification("http://vs.uni-kassel.de/IceTest#TestEntity1",
                                             "http://vs.uni-kassel.de/IceTest#TestEntity",
                                             "http://vs.uni-kassel.de/IceTest#TestScope1",
                                             "http://vs.uni-kassel.de/IceTest#TestRepresentation1");
  auto spec2 = ice::InformationSpecification("http://vs.uni-kassel.de/IceTest#TestEntity1",
                                             "http://vs.uni-kassel.de/IceTest#TestEntity",
                                             "http://vs.uni-kassel.de/IceTest#TestScope1",
                                             "http://vs.uni-kassel.de/IceTest#TestRepresentation2");

  // test processing system 1
  auto stream11 = engine->getStreamStore()->getStream<ice::Position>(&spec1,
                                                                     "http://vs.uni-kassel.de/IceTest#TestSourceNodeInd",
                                                                     "http://vs.uni-kassel.de/IceTest#TestCoordination2_SystemInd1");
  ASSERT_TRUE((stream11 ? true : false));

  auto stream12 = engine2->getStreamStore()->getStream<ice::Position>(&spec2,
                                                                      "http://vs.uni-kassel.de/IceTest#TestComputationalNodeInd",
                                                                      "http://vs.uni-kassel.de/IceTest#TestCoordination2_SystemInd2");
  ASSERT_TRUE((stream12 ? true : false));

  // test processing system 2
  auto stream21 = engine2->getStreamStore()->getStream<ice::Position>(&spec1,
                                                                      "http://vs.uni-kassel.de/IceTest#TestSourceNodeInd",
                                                                      "http://vs.uni-kassel.de/IceTest#TestCoordination2_SystemInd1");
  ASSERT_TRUE((stream21 ? true : false));

  auto stream22 = engine2->getStreamStore()->getStream<ice::Position>(&spec2,
                                                                      "http://vs.uni-kassel.de/IceTest#TestComputationalNodeInd",
                                                                      "http://vs.uni-kassel.de/IceTest#TestCoordination2_SystemInd2");
  ASSERT_TRUE((stream22 ? true : false));

  // insert element in stream of system 2
  int x = rand();
  int y = rand();
  int z = rand();

  auto position1 = std::make_shared<ice::Position>();
  position1->x = x;
  position1->y = y;
  position1->z = z;

  stream11->add(position1);

  // wait some time
  std::this_thread::sleep_for(std::chrono::milliseconds {500});

  // check if element was send to system 2 and placed in stream2
  auto position2 = stream12->getLast();

  ASSERT_TRUE((position2 ? true : false));
  EXPECT_EQ(x - 1, position2->getInformation()->x);
  EXPECT_EQ(y - 1, position2->getInformation()->y);
  EXPECT_EQ(z - 1, position2->getInformation()->z);
}
