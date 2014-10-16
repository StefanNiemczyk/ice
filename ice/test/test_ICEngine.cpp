/*
 * test_ICEngine.cpp
 *
 *  Created on: Jun 3, 2014
 *      Author: sni
 */

#include <chrono>
#include <ctime>
#include <memory>
#include <thread>
#include <typeinfo>
#include <vector>

#include <gtest/gtest.h>
#include <gtest/gtest-message.h>
#include <gtest/internal/gtest-internal.h>
#include <gtest/internal/gtest-string.h>

#include "boost/lexical_cast.hpp"
#include "boost/uuid/uuid.hpp"

#include "ice/Logger.h"
#include "ice/ICEngine.h"
#include "ice/container/Position.h"
#include "ice/coordination/InformationModel.h"
#include "ice/coordination/IntersectionInformationModel.h"
#include "ice/coordination/ModelComperator.h"
#include "ice/coordination/NodeDescription.h"
#include "ice/coordination/StreamDescription.h"
#include "ice/coordination/EngineState.h"
#include "ice/information/InformationElement.h"
#include "ice/information/InformationSpecification.h"
#include "ice/information/InformationStore.h"
#include "ice/information/InformationStream.h"
#include "ice/information/InformationStreamTemplate.h"
#include "ice/information/InformationType.h"
#include "ice/processing/Node.h"
#include "ice/processing/NodeStore.h"

#include "etc/EngineStuff.cpp"
#include "etc/TestTime.cpp"

namespace
{

class TestICEngine : public ::testing::Test
{
protected:
  time_t start_time_;

  virtual void SetUp()
  {
    start_time_ = time(NULL);
    ice::Node::registerNodeCreator("SimpleSourceNode", &SimpleSourceNode::createNode);
    ice::Node::registerNodeCreator("smothing", &SmothingNode::createNode);
    ice::Node::registerNodeCreator("ObstacleFusing", &ObstacleFusion::createNode);
    ice::Node::registerNodeCreator("ObstacleTracking", &ObstacleFusion::createNode);
  }

  virtual void TearDown()
  {
    const time_t end_time = time(NULL);

    //  EXPECT_TRUE(end_time - start_time_ <= 5) << "The test took too long.";
  }

  std::shared_ptr<ice::ICEngine> getEngine()
  {
    auto streamFactory = std::make_shared<TestFactory>();
    auto timeFactory = std::make_shared<TestTimeFactory>();

    return std::make_shared<ice::ICEngine>(timeFactory, streamFactory);
  }
};

TEST_F(TestICEngine, create)
{
  auto streamFactory = std::make_shared<TestFactory>();
  auto timeFactory = std::make_shared<TestTimeFactory>();

  auto engine = std::make_shared<ice::ICEngine>(timeFactory, streamFactory);

  EXPECT_TRUE((engine ? true : false));

  engine->init();
}

TEST_F(TestICEngine, read_files)
{
  auto engine = getEngine();

  bool resultVal = engine->readFromFile("data/simple_example.xml");

  EXPECT_TRUE(resultVal);

  auto informationStore = engine->getInformationStore();
  auto nodeStore = engine->getNodeStore();

  // Checking information type
  auto infoPosition = informationStore->getInformationType("/Position");
  auto infoPositionRaw = informationStore->getInformationType("/Position/raw");
  auto infoObstacle = informationStore->getInformationType("/Obstacle");
  auto infoObstacleFused = informationStore->getInformationType("/Obstacle/fused");

  EXPECT_TRUE((infoPosition ? true : false));
  EXPECT_TRUE((infoPositionRaw ? true : false));
  EXPECT_TRUE((infoObstacle ? true : false));
  EXPECT_TRUE((infoObstacleFused ? true : false));

  EXPECT_EQ("Position", infoPosition->getSpecification()->getTypeString());
  EXPECT_EQ("Position", infoPositionRaw->getSpecification()->getTypeString());
  EXPECT_EQ("List[Position]", infoObstacle->getSpecification()->getTypeString());
  EXPECT_EQ("List[Position]", infoObstacleFused->getSpecification()->getTypeString());

  // Checking streams
  auto streamPosition1 = infoPosition->getStream<ice::Position>("/Position|own");
  auto streamPosition2 = informationStore->getStream<ice::Position>("/Position|own");

  EXPECT_EQ(streamPosition1, streamPosition2);
  EXPECT_EQ(100, streamPosition1->getStreamSize());
  EXPECT_EQ("Own position on the field", streamPosition1->getDescription());
  EXPECT_EQ("own", streamPosition1->getProvider());
  EXPECT_EQ(true, streamPosition1->canBeShared());
  EXPECT_EQ(0, streamPosition1->getSharingCount());
  EXPECT_EQ(10, streamPosition1->getSharingMaxCount());

  auto streamPositionRaw1 = infoPositionRaw->getStream<ice::Position>("/Position/raw|own");
  auto streamPositionRaw2 = informationStore->getStream<ice::Position>("/Position/raw|own");

  EXPECT_EQ(streamPositionRaw1, streamPositionRaw2);
  EXPECT_EQ(50, streamPositionRaw1->getStreamSize());
  EXPECT_EQ("Raw position on the field", streamPositionRaw1->getDescription());
  EXPECT_EQ("own", streamPositionRaw1->getProvider());
  EXPECT_EQ(false, streamPositionRaw1->canBeShared());
  EXPECT_EQ(0, streamPositionRaw1->getSharingCount());
  EXPECT_EQ(0, streamPositionRaw1->getSharingMaxCount());

  auto streamObstacle1 = infoObstacle->getStream<std::vector<ice::Position>>("/Obstacle|own");
  auto streamObstacle2 = informationStore->getStream<std::vector<ice::Position>>("/Obstacle|own");

  EXPECT_EQ(streamObstacle1, streamObstacle2);
  EXPECT_EQ(50, streamObstacle1->getStreamSize());
  EXPECT_EQ("Obstacle positions on the field", streamObstacle1->getDescription());
  EXPECT_EQ("own", streamObstacle1->getProvider());
  EXPECT_EQ(true, streamObstacle1->canBeShared());
  EXPECT_EQ(0, streamObstacle1->getSharingCount());
  EXPECT_EQ(10, streamObstacle1->getSharingMaxCount());

  auto streamObstacleFused1 = infoObstacleFused->getStream<std::vector<ice::Position>>("/Obstacle/fused|own");
  auto streamObstacleFused2 = informationStore->getStream<std::vector<ice::Position>>("/Obstacle/fused|own");

  EXPECT_EQ(streamObstacleFused1, streamObstacleFused2);
  EXPECT_EQ(20, streamObstacleFused1->getStreamSize());
  EXPECT_EQ("Fused list of obstacles", streamObstacleFused1->getDescription());
  EXPECT_EQ("own", streamObstacleFused1->getProvider());
  EXPECT_EQ(true, streamObstacleFused1->canBeShared());
  EXPECT_EQ(0, streamObstacleFused1->getSharingCount());
  EXPECT_EQ(10, streamObstacleFused1->getSharingMaxCount());

  try
  {
    auto testException1 = infoPositionRaw->getStream<int>("/Position/raw|own");
    EXPECT_FALSE(true);
  }
  catch (std::bad_cast& e)
  {
    EXPECT_TRUE(true);
  }

  try
  {
    auto testException2 = informationStore->getStream<int>("/Position/raw|own");
    EXPECT_FALSE(true);
  }
  catch (std::bad_cast& e)
  {
    EXPECT_TRUE(true);
  }

  // Checking stream templates
  auto streamTemplatePosition1 = infoPosition->getStreamTemplate("/Position|?provider");
  auto streamTemplatePosition2 = informationStore->getStreamTemplate("/Position|?provider");

  EXPECT_EQ(streamTemplatePosition1, streamTemplatePosition2);
  EXPECT_EQ("$other", streamTemplatePosition1->getProvider());
  EXPECT_EQ("Position of other robots on the field", streamTemplatePosition1->getDescription());
  EXPECT_EQ(10, streamTemplatePosition1->getMaxStreamCount());

  auto streamTemplateObstalce1 = infoObstacle->getStreamTemplate("/Obstacle|?provider");
  auto streamTemplateObstacle2 = informationStore->getStreamTemplate("/Obstacle|?provider");

  EXPECT_EQ(streamTemplateObstalce1, streamTemplateObstacle2);
  EXPECT_EQ("$other", streamTemplateObstalce1->getProvider());
  EXPECT_EQ("List of streams of the given providers", streamTemplateObstalce1->getDescription());
  EXPECT_EQ(10, streamTemplateObstalce1->getMaxStreamCount());

  // Checking nodes
  auto nodeSource = nodeStore->getNode("SimpleSourceNode");

  EXPECT_TRUE((nodeSource ? true : false));
  if (nodeSource)
  {
    EXPECT_EQ(ice::NodeType::SOURCE, nodeSource->getType());
    EXPECT_EQ(-1, nodeSource->getCyclicTriggerTime());
    EXPECT_EQ(0, nodeSource->getInputs()->size());
    EXPECT_EQ(0, nodeSource->getBaseInputs()->size());
    EXPECT_EQ(0, nodeSource->getTriggeredByInputs()->size());
    EXPECT_EQ(0, nodeSource->getInputTemplates()->size());
    EXPECT_EQ(1, nodeSource->getOutputs()->size());

    EXPECT_EQ(streamPositionRaw1, (*nodeSource->getOutputs())[0]);
  }

  auto nodeSimpleProcessing = nodeStore->getNode("SimpleProcessingNode");

  EXPECT_TRUE((nodeSimpleProcessing ? true : false));
  if (nodeSimpleProcessing)
  {
    EXPECT_EQ(ice::NodeType::PROCESSING, nodeSimpleProcessing->getType());
    EXPECT_EQ(30, nodeSimpleProcessing->getCyclicTriggerTime());
    EXPECT_EQ(1, nodeSimpleProcessing->getInputs()->size());
    EXPECT_EQ(1, nodeSimpleProcessing->getBaseInputs()->size());
    EXPECT_EQ(1, nodeSimpleProcessing->getTriggeredByInputs()->size());
    EXPECT_EQ(0, nodeSimpleProcessing->getInputTemplates()->size());
    EXPECT_EQ(1, nodeSimpleProcessing->getOutputs()->size());

    auto inputStream = (*nodeSimpleProcessing->getInputs())[0];
    EXPECT_EQ(streamPositionRaw1, inputStream);
    EXPECT_EQ(1, inputStream->registerTaskAsync(nodeSimpleProcessing));

    EXPECT_EQ(streamPositionRaw1, (*nodeSimpleProcessing->getTriggeredByInputs())[0]);
    EXPECT_EQ(streamPosition1, (*nodeSimpleProcessing->getOutputs())[0]);
  }

  auto nodeObstacleClustering = nodeStore->getNode("ObstacleClustering");

  EXPECT_TRUE((nodeObstacleClustering ? true : false));
  if (nodeObstacleClustering)
  {
    EXPECT_EQ(ice::NodeType::PROCESSING, nodeObstacleClustering->getType());
    EXPECT_EQ(-1, nodeObstacleClustering->getCyclicTriggerTime());
    EXPECT_EQ(1, nodeObstacleClustering->getInputs()->size());
    EXPECT_EQ(1, nodeObstacleClustering->getBaseInputs()->size());
    EXPECT_EQ(1, nodeObstacleClustering->getTriggeredByInputs()->size());
    EXPECT_EQ(1, nodeObstacleClustering->getInputTemplates()->size());
    EXPECT_EQ(1, nodeObstacleClustering->getOutputs()->size());

    EXPECT_EQ(streamObstacle1, (*nodeObstacleClustering->getInputs())[0]);
    EXPECT_EQ(streamObstacle1, (*nodeObstacleClustering->getBaseInputs())[0]);
    EXPECT_EQ(streamObstacle1, (*nodeObstacleClustering->getTriggeredByInputs())[0]);
    EXPECT_EQ(streamTemplateObstalce1, (*nodeObstacleClustering->getInputTemplates())[0].lock());
    EXPECT_EQ(streamObstacleFused1, (*nodeObstacleClustering->getOutputs())[0]);

  }
}

TEST_F(TestICEngine, add_element)
{
  auto engine = getEngine();

  bool resultVal = engine->readFromFile("data/simple_example.xml");

  EXPECT_TRUE(resultVal);

  auto streamPosition = engine->getInformationStore()->getStream<ice::Position>("/Position|own");
  auto streamPositionRaw = engine->getInformationStore()->getStream<ice::Position>("/Position/raw|own");

  std::unique_ptr<ice::Position> pos(new ice::Position());
  int value = 5;

  pos->x = value;
  pos->y = value;
  pos->z = value;

  streamPositionRaw->add(std::move(pos));

  std::this_thread::sleep_for(std::chrono::milliseconds {10});

  ice::Position pos2 = streamPosition->getLast()->getInformation();

  EXPECT_EQ(value - 1, pos2.x);
  EXPECT_EQ(value - 1, pos2.y);
  EXPECT_EQ(value - 1, pos2.z);
}

TEST_F(TestICEngine, check_information_model)
{
  auto engine1 = getEngine();

  bool resultVal1 = engine1->readFromFiles( {"data/information.xml", "data/streams.xml", "data/model1.xml"});

  EXPECT_TRUE(resultVal1);

  auto model1 = engine1->getInformationModel();

  EXPECT_EQ(7, model1->getStreams()->size());
  EXPECT_EQ(2, model1->getStreamTemplates()->size());
  EXPECT_EQ(3, model1->getNodeDescriptions()->size());

  int found = 0;

  for (auto stream : *model1->getStreams())
  {
    if (stream->getId() == boost::lexical_cast<boost::uuids::uuid>("39f9b426-741c-4b89-8698-98ec25000011"))
    {
      ++found;
      EXPECT_EQ(false, stream->isShared());
    }
    else if (stream->getId() == boost::lexical_cast<boost::uuids::uuid>("39f9b426-741c-4b89-8698-98ec25000012"))
    {
      ++found;
      EXPECT_EQ(false, stream->isShared());
    }
    else if (stream->getId() == boost::lexical_cast<boost::uuids::uuid>("39f9b426-741c-4b89-8698-98ec25000013"))
    {
      ++found;
      EXPECT_EQ(true, stream->isShared());
    }
    else if (stream->getId() == boost::lexical_cast<boost::uuids::uuid>("39f9b426-741c-4b89-8698-98ec25000021"))
    {
      ++found;
      EXPECT_EQ(true, stream->isShared());
    }
    else if (stream->getId() == boost::lexical_cast<boost::uuids::uuid>("39f9b426-741c-4b89-8698-98ec25000022"))
    {
      ++found;
      EXPECT_EQ(true, stream->isShared());
    }
    else if (stream->getId() == boost::lexical_cast<boost::uuids::uuid>("39f9b426-741c-4b89-8698-98ec25000023"))
    {
      ++found;
      EXPECT_EQ(false, stream->isShared());
    }
    else if (stream->getId() == boost::lexical_cast<boost::uuids::uuid>("39f9b426-741c-4b89-8698-98ec25000024"))
    {
      ++found;
      EXPECT_EQ(true, stream->isShared());
    }
  }
  EXPECT_EQ(7, found);

  found = 0;

  for (auto streamTemplate : *model1->getStreamTemplates())
  {
    if (streamTemplate->getId() == boost::lexical_cast<boost::uuids::uuid>("39f9b426-741c-4b89-8698-98ec25000013"))
    {
      ++found;
    }
    else if (streamTemplate->getId() == boost::lexical_cast<boost::uuids::uuid>("39f9b426-741c-4b89-8698-98ec25000021"))
    {
      ++found;
    }
  }

  EXPECT_EQ(2, found);

  found = 0;

  for (auto node : *model1->getNodeDescriptions())
  {
    if (node->getClassName() == "smothing")
    {
      found++;
    }
    else if (node->getClassName() == "ObstacleFusing")
    {
      found++;
    }
    else if (node->getClassName() == "ObstacleTracking")
    {
      found++;
    }
  }

  EXPECT_EQ(3, found);
}

TEST_F(TestICEngine, compare_models)
{
  auto engine1 = getEngine();
  auto engine2 = getEngine();

  bool resultVal1 = engine1->readFromFiles( {"data/information.xml", "data/streams.xml", "data/model1.xml"});
  bool resultVal2 = engine2->readFromFiles( {"data/information.xml", "data/streams.xml", "data/model1.xml"});

  EXPECT_TRUE(resultVal1);
  EXPECT_TRUE(resultVal2);

  ice::ModelComperator mc;

  auto model1 = engine1->getInformationModel();
  auto model2 = engine2->getInformationModel();

  auto matches = mc.findModelMatches(model1, model2);

  EXPECT_EQ(1, matches->size());

  auto match = (*matches)[0];
  EXPECT_EQ(2, match->getNodeDescriptions()->size());
  EXPECT_EQ(1, match->getInputTemplates()->size());
  EXPECT_EQ(boost::lexical_cast<boost::uuids::uuid>("39f9b426-741c-4b89-8698-98ec25000021"),
            match->getInputTemplates()->at(0)->getId());
  EXPECT_EQ(2, match->getOutputStreams()->size());
  EXPECT_EQ(boost::lexical_cast<boost::uuids::uuid>("39f9b426-741c-4b89-8698-98ec25000022"),
            match->getOutputStreams()->at(0)->getId());
  EXPECT_EQ(boost::lexical_cast<boost::uuids::uuid>("39f9b426-741c-4b89-8698-98ec25000024"),
            match->getOutputStreams()->at(1)->getId());
  EXPECT_EQ(3, match->getConnectionMatrix()[0]);
  EXPECT_EQ(1, match->getConnectionMatrix()[1]);
  EXPECT_EQ(0, match->getConnectionMatrix()[2]);
  EXPECT_EQ(2, match->getConnectionMatrix()[3]);
}

TEST_F(TestICEngine, simple_communication)
{
  auto engine1 = getEngine();
  auto engine2 = getEngine();

  bool resultVal1 = engine1->readFromFiles( {"data/information.xml", "data/streams.xml", "data/model1.xml"});
  bool resultVal2 = engine2->readFromFiles( {"data/information.xml", "data/streams.xml", "data/model1.xml"});

  ASSERT_TRUE(resultVal1);
  ASSERT_TRUE(resultVal2);
//  const std::string name = "";
//  std::weak_ptr<ice::InformationType> informationType;
//  std::shared_ptr<ice::EventHandler> eventHandler;
//  std::shared_ptr<ice::InformationSpecification> specification;
//  auto stream = std::make_shared<ice::InformationStream<int>>(name, informationType, eventHandler, specification, 5);
//  engine1->getCommunication()->registerStreamAsReceiver(stream);

  std::this_thread::sleep_for(std::chrono::milliseconds {3000});

  auto engineState1to2 = engine1->getCoordinator()->getEngineState(engine2->getId());
  auto engineState2to1 = engine2->getCoordinator()->getEngineState(engine1->getId());

  ASSERT_TRUE((engineState1to2 ? true : false));
  ASSERT_TRUE((engineState2to1 ? true : false));

  EXPECT_EQ(ice::CooperationState::COOPERATION, engineState1to2->getCooperationState());
  EXPECT_EQ(ice::CooperationState::COOPERATION, engineState2to1->getCooperationState());

  EXPECT_EQ(2, engineState1to2->getStreamsOffered()->size());
  EXPECT_EQ(2, engineState1to2->getStreamsRequested()->size());
  EXPECT_EQ(2, engineState2to1->getStreamsOffered()->size());
  EXPECT_EQ(2, engineState2to1->getStreamsRequested()->size());

  auto stream1 = engine1->getInformationStore()->getStream<ice::Position>("/Position/corrected|own");
  auto stream2 = engine2->getInformationStore()->getStream<ice::Position>(
      "/Position/corrected|" + ice::IDGenerator::toString(engine1->getId()));

  ASSERT_TRUE((stream1 ? true : false));
  ASSERT_TRUE((stream2 ? true : false));

  std::unique_ptr<ice::Position> position1(new ice::Position());
  position1->x = 3;
  position1->y = 2;
  position1->z = 1;

  stream1->add(std::move(position1));

  std::this_thread::sleep_for(std::chrono::milliseconds {2000});

  auto position2 = stream2->getLast();

  ASSERT_TRUE((position2 ? true : false));
  EXPECT_EQ(3, position2->getInformation().x);
  EXPECT_EQ(2, position2->getInformation().y);
  EXPECT_EQ(1, position2->getInformation().z);
}

TEST_F(TestICEngine, no_cooperation)
{
  auto engine1 = getEngine();
  auto engine2 = getEngine();

  engine2->init();

  bool resultVal1 = engine1->readFromFiles( {"data/information.xml", "data/streams.xml", "data/model1.xml"});
  // bool resultVal2 = engine2->readFromFiles( {"data/information.xml", "data/streams.xml", "data/model1.xml"});

  ASSERT_TRUE(resultVal1);
  // ASSERT_TRUE(resultVal2);

  std::this_thread::sleep_for(std::chrono::milliseconds {2000});

  auto engineState1to2 = engine1->getCoordinator()->getEngineState(engine2->getId());
  auto engineState2to1 = engine2->getCoordinator()->getEngineState(engine1->getId());

  ASSERT_TRUE((engineState1to2 ? true : false));
  ASSERT_TRUE((engineState2to1 ? true : false));

  EXPECT_EQ(ice::CooperationState::NO_COOPERATION, engineState1to2->getCooperationState());
  EXPECT_EQ(ice::CooperationState::NO_COOPERATION, engineState2to1->getCooperationState());
}

TEST_F(TestICEngine, no_cooperation2)
{
  auto engine1 = getEngine();
  auto engine2 = getEngine();

  engine1->init();

  // bool resultVal1 = engine1->readFromFiles( {"data/information.xml", "data/streams.xml", "data/model1.xml"});
  bool resultVal2 = engine2->readFromFiles( {"data/information.xml", "data/streams.xml", "data/model1.xml"});

  // ASSERT_TRUE(resultVal1);
  ASSERT_TRUE(resultVal2);

  std::this_thread::sleep_for(std::chrono::milliseconds {2000});

  auto engineState1to2 = engine1->getCoordinator()->getEngineState(engine2->getId());
  auto engineState2to1 = engine2->getCoordinator()->getEngineState(engine1->getId());

  ASSERT_TRUE((engineState1to2 ? true : false));
  ASSERT_TRUE((engineState2to1 ? true : false));

  EXPECT_EQ(ice::CooperationState::NO_COOPERATION, engineState1to2->getCooperationState());
  EXPECT_EQ(ice::CooperationState::NO_COOPERATION, engineState2to1->getCooperationState());
}

TEST_F(TestICEngine, no_cooperation3)
{
  auto engine1 = getEngine();
  auto engine2 = getEngine();

  engine1->init();
  engine2->init();

  // bool resultVal1 = engine1->readFromFiles( {"data/information.xml", "data/streams.xml", "data/model1.xml"});
  // bool resultVal2 = engine2->readFromFiles( {"data/information.xml", "data/streams.xml", "data/model1.xml"});

  // ASSERT_TRUE(resultVal1);
  // ASSERT_TRUE(resultVal2);

  std::this_thread::sleep_for(std::chrono::milliseconds {2000});

  auto engineState1to2 = engine1->getCoordinator()->getEngineState(engine2->getId());
  auto engineState2to1 = engine2->getCoordinator()->getEngineState(engine1->getId());

  ASSERT_TRUE((engineState1to2 ? true : false));
  ASSERT_TRUE((engineState2to1 ? true : false));

  EXPECT_EQ(ice::CooperationState::NO_COOPERATION, engineState1to2->getCooperationState());
  EXPECT_EQ(ice::CooperationState::NO_COOPERATION, engineState2to1->getCooperationState());
}

}

