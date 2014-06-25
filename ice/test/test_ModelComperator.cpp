/*
 * test_ModelComperator.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#include <iostream>
#include <string>
#include <time.h>
#include <typeinfo>
#include <memory>

#include "boost/uuid/uuid.hpp"
#include "boost/uuid/uuid_generators.hpp"

#include "ice/coordination/InformationModel.h"
#include "ice/coordination/IntersectionInformationModel.h"
#include "ice/coordination/ModelComperator.h"
#include "ice/coordination/NodeDescription.h"
#include "ice/coordination/StreamDescription.h"
#include "ice/coordination/StreamTemplateDescription.h"

#include "gtest/gtest.h"

namespace
{

class TestModelComperator : public ::testing::Test
{
protected:
  time_t start_time_;

  virtual void SetUp()
  {
    start_time_ = time(NULL);

    for (int i = 0; i < 10; ++i)
    {
      stream[i] = std::make_shared<ice::StreamDescription>(boost::uuids::random_generator()(), true);
      streamTemplate[i] = std::make_shared<ice::StreamTemplateDescription>();
      streamTemplate[i]->setId(stream[i]->getId());
    }
  }

  virtual void TearDown()
  {
    const time_t end_time = time(NULL);

    EXPECT_TRUE(end_time - start_time_ <= 5) << "The test took too long.";
  }

  std::shared_ptr<ice::StreamDescription> stream[10];
  std::shared_ptr<ice::StreamTemplateDescription> streamTemplate[10];
};

TEST_F(TestModelComperator, requests_offers1)
{
  auto model1 = std::make_shared<ice::InformationModel>();
  auto model2 = std::make_shared<ice::InformationModel>();

  model1->getStreams()->push_back(stream[0]);
  model1->getStreams()->push_back(stream[1]);
  model1->getStreams()->push_back(stream[2]);

  model1->getStreamTemplates()->push_back(streamTemplate[5]);
  model1->getStreamTemplates()->push_back(streamTemplate[7]);
  model1->getStreamTemplates()->push_back(streamTemplate[8]);

  model2->getStreams()->push_back(stream[5]);
  model2->getStreams()->push_back(stream[6]);
  model2->getStreams()->push_back(stream[7]);

  model2->getStreamTemplates()->push_back(streamTemplate[1]);
  model2->getStreamTemplates()->push_back(streamTemplate[3]);
  model2->getStreamTemplates()->push_back(streamTemplate[4]);

  EXPECT_EQ(3, model1->getStreams()->size());
  EXPECT_EQ(3, model1->getStreamTemplates()->size());

  ice::ModelComperator mc;

  auto offers = std::make_shared<std::vector<std::shared_ptr<ice::StreamDescription>>>();
  auto requests = std::make_shared<std::vector<std::shared_ptr<ice::StreamTemplateDescription>>>();

  bool result = mc.findOfferesAndRequests(model1, model2, offers, requests);

  EXPECT_TRUE(result);

  EXPECT_EQ(1, offers->size());
  EXPECT_EQ(2, requests->size());

  int found = 0;

  for (auto desc : *offers)
  {
    if (desc->equals(stream[1].get()))
      ++found;
  }
  EXPECT_EQ(1, found);

  found = 0;

  for (auto desc : *requests)
  {
    if (desc->equals(streamTemplate[5].get()))
      ++found;

    if (desc->equals(streamTemplate[7].get()))
      ++found;
  }
  EXPECT_EQ(2, found);
}

TEST_F(TestModelComperator, requests_offers2)
{
  auto model1 = std::make_shared<ice::InformationModel>();
  auto model2 = std::make_shared<ice::InformationModel>();

  model1->getStreams()->push_back(stream[0]);
  model1->getStreams()->push_back(stream[1]);
  model1->getStreams()->push_back(stream[2]);

  model2->getStreams()->push_back(stream[5]);
  model2->getStreams()->push_back(stream[6]);
  model2->getStreams()->push_back(stream[7]);

  model2->getStreamTemplates()->push_back(streamTemplate[1]);
  model2->getStreamTemplates()->push_back(streamTemplate[3]);
  model2->getStreamTemplates()->push_back(streamTemplate[4]);

  EXPECT_EQ(3, model1->getStreams()->size());
  EXPECT_EQ(0, model1->getStreamTemplates()->size());

  ice::ModelComperator mc;

  auto offers = std::make_shared<std::vector<std::shared_ptr<ice::StreamDescription>>>();
  auto requests = std::make_shared<std::vector<std::shared_ptr<ice::StreamTemplateDescription>>>();

  bool result = mc.findOfferesAndRequests(model1, model2, offers, requests);

  EXPECT_TRUE(result);

  EXPECT_EQ(1, offers->size());
  EXPECT_EQ(0, requests->size());

  int found = 0;

  for (auto desc : *offers)
  {
    if (desc->equals(stream[1].get()))
      ++found;
  }
  EXPECT_EQ(1, found);
}

TEST_F(TestModelComperator, requests_offers3)
{
  auto model1 = std::make_shared<ice::InformationModel>();
  auto model2 = std::make_shared<ice::InformationModel>();

  EXPECT_EQ(0, model1->getStreams()->size());
  EXPECT_EQ(0, model1->getStreamTemplates()->size());

  ice::ModelComperator mc;

  auto offers = std::make_shared<std::vector<std::shared_ptr<ice::StreamDescription>>>();
  auto requests = std::make_shared<std::vector<std::shared_ptr<ice::StreamTemplateDescription>>>();

  bool result = mc.findOfferesAndRequests(model1, model2, offers, requests);

  EXPECT_FALSE(result);

  EXPECT_EQ(0, offers->size());
  EXPECT_EQ(0, requests->size());
}

TEST_F(TestModelComperator, find_matchings)
{
  auto model1 = std::make_shared<ice::InformationModel>();
  auto model2 = std::make_shared<ice::InformationModel>();

  model1->getStreams()->push_back(stream[0]);
  model1->getStreams()->push_back(stream[1]);
  model1->getStreamTemplates()->push_back(streamTemplate[0]);

  boost::uuids::uuid* inputUuids = new boost::uuids::uuid[1];
  boost::uuids::uuid* inputTemplateUuids = new boost::uuids::uuid[1];
  boost::uuids::uuid* outputUuids = new boost::uuids::uuid[1];

  inputUuids[0] = stream[0]->getId();
  inputTemplateUuids[0] = streamTemplate[0]->getId();
  outputUuids[0] = stream[1]->getId();

  auto node = std::make_shared<ice::NodeDescription>("node1", inputUuids, inputTemplateUuids, outputUuids, 1, 1, 1);
  model1->getNodeDescriptions()->push_back(node);

  model2->getStreams()->push_back(stream[0]);
  model2->getStreams()->push_back(stream[1]);
  model2->getStreamTemplates()->push_back(streamTemplate[0]);

  boost::uuids::uuid* inputUuids2 = new boost::uuids::uuid[1];
  boost::uuids::uuid* inputTemplateUuids2 = new boost::uuids::uuid[1];
  boost::uuids::uuid* outputUuids2 = new boost::uuids::uuid[1];

  inputUuids2[0] = stream[0]->getId();
  inputTemplateUuids2[0] = streamTemplate[0]->getId();
  outputUuids2[0] = stream[1]->getId();

  auto node2 = std::make_shared<ice::NodeDescription>("node1", inputUuids2, inputTemplateUuids2, outputUuids2, 1, 1, 1);
  model2->getNodeDescriptions()->push_back(node2);

  ice::ModelComperator mc;

  auto result = mc.findModelMatches(model1, model2);

  auto match = (*result)[0];
  EXPECT_EQ(1, result->size());
  EXPECT_EQ(1, match->getNodeDescriptions()->size());
  EXPECT_EQ(1, match->getInputTemplates()->size());
  EXPECT_EQ(stream[0]->getId(), match->getInputTemplates()->at(0)->getId());
  EXPECT_EQ(1, match->getOutputStreams()->size());
  EXPECT_EQ(stream[1]->getId(), match->getOutputStreams()->at(0)->getId());
  EXPECT_EQ(3, match->getConnectionMatrix()[0]);
}

TEST_F(TestModelComperator, find_matchings2)
{
  auto model1 = std::make_shared<ice::InformationModel>();
  auto model2 = std::make_shared<ice::InformationModel>();

  model1->getStreams()->push_back(stream[0]);
  model1->getStreams()->push_back(stream[1]);
  model1->getStreams()->push_back(stream[2]);
  model1->getStreamTemplates()->push_back(streamTemplate[0]);

  boost::uuids::uuid* inputUuids = new boost::uuids::uuid[1];
  boost::uuids::uuid* inputTemplateUuids = new boost::uuids::uuid[1];
  boost::uuids::uuid* outputUuids = new boost::uuids::uuid[1];

  inputUuids[0] = stream[0]->getId();
  inputTemplateUuids[0] = streamTemplate[0]->getId();
  outputUuids[0] = stream[1]->getId();

  auto node = std::make_shared<ice::NodeDescription>("node1", inputUuids, inputTemplateUuids, outputUuids, 1, 1, 1);
  model1->getNodeDescriptions()->push_back(node);

  inputUuids = new boost::uuids::uuid[1];
  inputTemplateUuids = new boost::uuids::uuid[0];
  outputUuids = new boost::uuids::uuid[1];

  inputUuids[0] = stream[1]->getId();
  outputUuids[0] = stream[2]->getId();

  node = std::make_shared<ice::NodeDescription>("node2", inputUuids, inputTemplateUuids, outputUuids, 1, 0, 1);
  model1->getNodeDescriptions()->push_back(node);

  model2->getStreams()->push_back(stream[0]);
  model2->getStreams()->push_back(stream[1]);
  model2->getStreams()->push_back(stream[2]);
  model2->getStreamTemplates()->push_back(streamTemplate[0]);

  boost::uuids::uuid* inputUuids2 = new boost::uuids::uuid[1];
  boost::uuids::uuid* inputTemplateUuids2 = new boost::uuids::uuid[1];
  boost::uuids::uuid* outputUuids2 = new boost::uuids::uuid[1];

  inputUuids2[0] = stream[0]->getId();
  inputTemplateUuids2[0] = streamTemplate[0]->getId();
  outputUuids2[0] = stream[1]->getId();

  auto node2 = std::make_shared<ice::NodeDescription>("node1", inputUuids2, inputTemplateUuids2, outputUuids2, 1, 1, 1);
  model2->getNodeDescriptions()->push_back(node2);

  ice::ModelComperator mc;

  auto result = mc.findModelMatches(model1, model2);

  auto match = (*result)[0];
  EXPECT_EQ(1, result->size());
  EXPECT_EQ(1, match->getNodeDescriptions()->size());
  EXPECT_EQ(1, match->getInputTemplates()->size());
  EXPECT_EQ(stream[0]->getId(), match->getInputTemplates()->at(0)->getId());
  EXPECT_EQ(1, match->getOutputStreams()->size());
  EXPECT_EQ(stream[1]->getId(), match->getOutputStreams()->at(0)->getId());
  EXPECT_EQ(3, match->getConnectionMatrix()[0]);
}

TEST_F(TestModelComperator, find_matchings3)
{
  auto model1 = std::make_shared<ice::InformationModel>();
  auto model2 = std::make_shared<ice::InformationModel>();

  model1->getStreams()->push_back(stream[0]);
  model1->getStreams()->push_back(stream[1]);
  model1->getStreams()->push_back(stream[2]);
  model1->getStreamTemplates()->push_back(streamTemplate[0]);

  boost::uuids::uuid* inputUuids = new boost::uuids::uuid[1];
  boost::uuids::uuid* inputTemplateUuids = new boost::uuids::uuid[1];
  boost::uuids::uuid* outputUuids = new boost::uuids::uuid[1];

  inputUuids[0] = stream[0]->getId();
  inputTemplateUuids[0] = streamTemplate[0]->getId();
  outputUuids[0] = stream[1]->getId();

  auto node = std::make_shared<ice::NodeDescription>("node1", inputUuids, inputTemplateUuids, outputUuids, 1, 1, 1);
  model1->getNodeDescriptions()->push_back(node);

  inputUuids = new boost::uuids::uuid[1];
  inputTemplateUuids = new boost::uuids::uuid[0];
  outputUuids = new boost::uuids::uuid[1];

  inputUuids[0] = stream[1]->getId();
  outputUuids[0] = stream[2]->getId();

  node = std::make_shared<ice::NodeDescription>("node2", inputUuids, inputTemplateUuids, outputUuids, 1, 0, 1);
  model1->getNodeDescriptions()->push_back(node);

  model2->getStreams()->push_back(stream[0]);
  model2->getStreams()->push_back(stream[1]);
  model2->getStreams()->push_back(stream[2]);
  model2->getStreamTemplates()->push_back(streamTemplate[0]);

  boost::uuids::uuid* inputUuids2 = new boost::uuids::uuid[1];
  boost::uuids::uuid* inputTemplateUuids2 = new boost::uuids::uuid[1];
  boost::uuids::uuid* outputUuids2 = new boost::uuids::uuid[1];

  inputUuids2[0] = stream[0]->getId();
  inputTemplateUuids2[0] = streamTemplate[0]->getId();
  outputUuids2[0] = stream[1]->getId();

  auto node2 = std::make_shared<ice::NodeDescription>("node1", inputUuids2, inputTemplateUuids2, outputUuids2, 1, 1, 1);
  model2->getNodeDescriptions()->push_back(node2);

  inputUuids2 = new boost::uuids::uuid[1];
  inputTemplateUuids2 = new boost::uuids::uuid[0];
  outputUuids2 = new boost::uuids::uuid[1];

  inputUuids2[0] = stream[1]->getId();
  outputUuids2[0] = stream[2]->getId();

  node2 = std::make_shared<ice::NodeDescription>("node2", inputUuids2, inputTemplateUuids2, outputUuids2, 1, 0, 1);
  model2->getNodeDescriptions()->push_back(node2);

  ice::ModelComperator mc;

  auto result = mc.findModelMatches(model1, model2);

  auto match = (*result)[0];
  EXPECT_EQ(1, result->size());
  EXPECT_EQ(2, match->getNodeDescriptions()->size());
  EXPECT_EQ(1, match->getInputTemplates()->size());
  EXPECT_EQ(stream[0]->getId(), match->getInputTemplates()->at(0)->getId());
  EXPECT_EQ(2, match->getOutputStreams()->size());
  EXPECT_EQ(stream[1]->getId(), match->getOutputStreams()->at(0)->getId());
  EXPECT_EQ(stream[2]->getId(), match->getOutputStreams()->at(1)->getId());
  EXPECT_EQ(3, match->getConnectionMatrix()[0]);
  EXPECT_EQ(1, match->getConnectionMatrix()[1]);
  EXPECT_EQ(0, match->getConnectionMatrix()[2]);
  EXPECT_EQ(2, match->getConnectionMatrix()[3]);
}

TEST_F(TestModelComperator, find_matchings4)
{
  auto model1 = std::make_shared<ice::InformationModel>();
  auto model2 = std::make_shared<ice::InformationModel>();

  model1->getStreams()->push_back(stream[0]);
  model1->getStreams()->push_back(stream[1]);
  model1->getStreams()->push_back(stream[2]);
  model1->getStreams()->push_back(stream[3]);
  model1->getStreams()->push_back(stream[4]);
  model1->getStreamTemplates()->push_back(streamTemplate[0]);

  boost::uuids::uuid* inputUuids = new boost::uuids::uuid[1];
  boost::uuids::uuid* inputTemplateUuids = new boost::uuids::uuid[1];
  boost::uuids::uuid* outputUuids = new boost::uuids::uuid[1];

  inputUuids[0] = stream[0]->getId();
  inputTemplateUuids[0] = streamTemplate[0]->getId();
  outputUuids[0] = stream[1]->getId();

  auto node = std::make_shared<ice::NodeDescription>("node1", inputUuids, inputTemplateUuids, outputUuids, 1, 1, 1);
  model1->getNodeDescriptions()->push_back(node);

  inputUuids = new boost::uuids::uuid[1];
  inputTemplateUuids = new boost::uuids::uuid[0];
  outputUuids = new boost::uuids::uuid[1];

  inputUuids[0] = stream[1]->getId();
  outputUuids[0] = stream[2]->getId();

  node = std::make_shared<ice::NodeDescription>("node2", inputUuids, inputTemplateUuids, outputUuids, 1, 0, 1);
  model1->getNodeDescriptions()->push_back(node);

  inputUuids = new boost::uuids::uuid[1];
  inputTemplateUuids = new boost::uuids::uuid[0];
  outputUuids = new boost::uuids::uuid[1];

  inputUuids[0] = stream[3]->getId();
  outputUuids[0] = stream[4]->getId();

  node = std::make_shared<ice::NodeDescription>("node3", inputUuids, inputTemplateUuids, outputUuids, 1, 0, 1);
  model1->getNodeDescriptions()->push_back(node);

  model2->getStreams()->push_back(stream[0]);
  model2->getStreams()->push_back(stream[1]);
  model2->getStreams()->push_back(stream[2]);
  model2->getStreams()->push_back(stream[3]);
  model2->getStreams()->push_back(stream[4]);
  model2->getStreamTemplates()->push_back(streamTemplate[0]);

  boost::uuids::uuid* inputUuids2 = new boost::uuids::uuid[1];
  boost::uuids::uuid* inputTemplateUuids2 = new boost::uuids::uuid[1];
  boost::uuids::uuid* outputUuids2 = new boost::uuids::uuid[1];

  inputUuids2[0] = stream[0]->getId();
  inputTemplateUuids2[0] = streamTemplate[0]->getId();
  outputUuids2[0] = stream[1]->getId();

  auto node2 = std::make_shared<ice::NodeDescription>("node1", inputUuids2, inputTemplateUuids2, outputUuids2, 1, 1, 1);
  model2->getNodeDescriptions()->push_back(node2);

  inputUuids2 = new boost::uuids::uuid[1];
  inputTemplateUuids2 = new boost::uuids::uuid[0];
  outputUuids2 = new boost::uuids::uuid[1];

  inputUuids2[0] = stream[1]->getId();
  outputUuids2[0] = stream[2]->getId();

  node2 = std::make_shared<ice::NodeDescription>("node2", inputUuids2, inputTemplateUuids2, outputUuids2, 1, 0, 1);
  model2->getNodeDescriptions()->push_back(node2);

  inputUuids2 = new boost::uuids::uuid[1];
  inputTemplateUuids2 = new boost::uuids::uuid[0];
  outputUuids2 = new boost::uuids::uuid[1];

  inputUuids2[0] = stream[3]->getId();
  outputUuids2[0] = stream[4]->getId();

  node2 = std::make_shared<ice::NodeDescription>("node3", inputUuids2, inputTemplateUuids2, outputUuids2, 1, 0, 1);
  model2->getNodeDescriptions()->push_back(node2);

  ice::ModelComperator mc;

  auto result = mc.findModelMatches(model1, model2);

  auto match = (*result)[0];
  EXPECT_EQ(1, result->size());
  EXPECT_EQ(2, match->getNodeDescriptions()->size());
  EXPECT_EQ(1, match->getInputTemplates()->size());
  EXPECT_EQ(stream[0]->getId(), match->getInputTemplates()->at(0)->getId());
  EXPECT_EQ(2, match->getOutputStreams()->size());
  EXPECT_EQ(stream[1]->getId(), match->getOutputStreams()->at(0)->getId());
  EXPECT_EQ(stream[2]->getId(), match->getOutputStreams()->at(1)->getId());
  EXPECT_EQ(3, match->getConnectionMatrix()[0]);
  EXPECT_EQ(1, match->getConnectionMatrix()[1]);
  EXPECT_EQ(0, match->getConnectionMatrix()[2]);
  EXPECT_EQ(2, match->getConnectionMatrix()[3]);
}

TEST_F(TestModelComperator, find_matchings5)
{
  auto model1 = std::make_shared<ice::InformationModel>();
  auto model2 = std::make_shared<ice::InformationModel>();

  model1->getStreams()->push_back(stream[0]);
  model1->getStreams()->push_back(stream[1]);
  model1->getStreams()->push_back(stream[2]);
  model1->getStreams()->push_back(stream[3]);
  model1->getStreams()->push_back(stream[4]);
  model1->getStreams()->push_back(stream[5]);
  model1->getStreams()->push_back(stream[6]);
  model1->getStreams()->push_back(stream[7]);
  model1->getStreams()->push_back(stream[8]);
  model1->getStreamTemplates()->push_back(streamTemplate[0]);
  model1->getStreamTemplates()->push_back(streamTemplate[6]);

  boost::uuids::uuid* inputUuids = new boost::uuids::uuid[1];
  boost::uuids::uuid* inputTemplateUuids = new boost::uuids::uuid[1];
  boost::uuids::uuid* outputUuids = new boost::uuids::uuid[1];

  inputUuids[0] = stream[0]->getId();
  inputTemplateUuids[0] = streamTemplate[0]->getId();
  outputUuids[0] = stream[1]->getId();

  auto node = std::make_shared<ice::NodeDescription>("node1", inputUuids, inputTemplateUuids, outputUuids, 1, 1, 1);
  model1->getNodeDescriptions()->push_back(node);

  inputUuids = new boost::uuids::uuid[2];
  inputTemplateUuids = new boost::uuids::uuid[0];
  outputUuids = new boost::uuids::uuid[1];

  inputUuids[0] = stream[1]->getId();
  inputUuids[1] = stream[5]->getId();
  outputUuids[0] = stream[2]->getId();

  node = std::make_shared<ice::NodeDescription>("node2", inputUuids, inputTemplateUuids, outputUuids, 1, 0, 1);
  model1->getNodeDescriptions()->push_back(node);

  inputUuids = new boost::uuids::uuid[1];
  inputTemplateUuids = new boost::uuids::uuid[0];
  outputUuids = new boost::uuids::uuid[1];

  inputUuids[0] = stream[2]->getId();
  outputUuids[0] = stream[3]->getId();

  node = std::make_shared<ice::NodeDescription>("node3", inputUuids, inputTemplateUuids, outputUuids, 1, 0, 1);
  model1->getNodeDescriptions()->push_back(node);

  inputUuids = new boost::uuids::uuid[1];
  inputTemplateUuids = new boost::uuids::uuid[0];
  outputUuids = new boost::uuids::uuid[1];

  inputUuids[0] = stream[4]->getId();
  outputUuids[0] = stream[5]->getId();

  node = std::make_shared<ice::NodeDescription>("node4", inputUuids, inputTemplateUuids, outputUuids, 1, 0, 1);
  model1->getNodeDescriptions()->push_back(node);

  inputUuids = new boost::uuids::uuid[1];
  inputTemplateUuids = new boost::uuids::uuid[1];
  outputUuids = new boost::uuids::uuid[1];

  inputUuids[0] = stream[6]->getId();
  inputTemplateUuids[0] = streamTemplate[6]->getId();
  outputUuids[0] = stream[7]->getId();

  node = std::make_shared<ice::NodeDescription>("node5", inputUuids, inputTemplateUuids, outputUuids, 1, 1, 1);
  model1->getNodeDescriptions()->push_back(node);

  inputUuids = new boost::uuids::uuid[1];
  inputTemplateUuids = new boost::uuids::uuid[0];
  outputUuids = new boost::uuids::uuid[1];

  inputUuids[0] = stream[7]->getId();
  outputUuids[0] = stream[8]->getId();

  node = std::make_shared<ice::NodeDescription>("node6", inputUuids, inputTemplateUuids, outputUuids, 1, 0, 1);
  model1->getNodeDescriptions()->push_back(node);

  model2->getStreams()->push_back(stream[0]);
  model2->getStreams()->push_back(stream[1]);
  model2->getStreams()->push_back(stream[2]);
  model2->getStreams()->push_back(stream[3]);
  model2->getStreams()->push_back(stream[4]);
  model2->getStreams()->push_back(stream[5]);
  model2->getStreams()->push_back(stream[6]);
  model2->getStreams()->push_back(stream[7]);
  model2->getStreams()->push_back(stream[8]);
  model2->getStreamTemplates()->push_back(streamTemplate[0]);
  model2->getStreamTemplates()->push_back(streamTemplate[6]);

  boost::uuids::uuid* inputUuids2 = new boost::uuids::uuid[1];
  boost::uuids::uuid* inputTemplateUuids2 = new boost::uuids::uuid[1];
  boost::uuids::uuid* outputUuids2 = new boost::uuids::uuid[1];

  inputUuids2[0] = stream[0]->getId();
  inputTemplateUuids2[0] = streamTemplate[0]->getId();
  outputUuids2[0] = stream[1]->getId();

  auto node2 = std::make_shared<ice::NodeDescription>("node1", inputUuids2, inputTemplateUuids2, outputUuids2, 1, 1, 1);
  model2->getNodeDescriptions()->push_back(node2);

  inputUuids2 = new boost::uuids::uuid[2];
  inputTemplateUuids2 = new boost::uuids::uuid[0];
  outputUuids2 = new boost::uuids::uuid[1];

  inputUuids2[0] = stream[1]->getId();
  inputUuids2[1] = stream[5]->getId();
  outputUuids2[0] = stream[2]->getId();

  node2 = std::make_shared<ice::NodeDescription>("node2", inputUuids2, inputTemplateUuids2, outputUuids2, 1, 0, 1);
  model2->getNodeDescriptions()->push_back(node2);

  inputUuids2 = new boost::uuids::uuid[1];
  inputTemplateUuids2 = new boost::uuids::uuid[0];
  outputUuids2 = new boost::uuids::uuid[1];

  inputUuids2[0] = stream[2]->getId();
  outputUuids2[0] = stream[3]->getId();

  node2 = std::make_shared<ice::NodeDescription>("node3", inputUuids2, inputTemplateUuids2, outputUuids2, 1, 0, 1);
  model2->getNodeDescriptions()->push_back(node2);

  inputUuids2 = new boost::uuids::uuid[1];
  inputTemplateUuids2 = new boost::uuids::uuid[0];
  outputUuids2 = new boost::uuids::uuid[1];

  inputUuids2[0] = stream[4]->getId();
  outputUuids2[0] = stream[5]->getId();

  node2 = std::make_shared<ice::NodeDescription>("node4", inputUuids2, inputTemplateUuids2, outputUuids2, 1, 0, 1);
  model2->getNodeDescriptions()->push_back(node2);

  inputUuids2 = new boost::uuids::uuid[1];
  inputTemplateUuids2 = new boost::uuids::uuid[1];
  outputUuids2 = new boost::uuids::uuid[1];

  inputUuids2[0] = stream[6]->getId();
  inputTemplateUuids2[0] = streamTemplate[6]->getId();
  outputUuids2[0] = stream[7]->getId();

  node2 = std::make_shared<ice::NodeDescription>("node5", inputUuids2, inputTemplateUuids2, outputUuids2, 1, 1, 1);
  model2->getNodeDescriptions()->push_back(node2);

  inputUuids2 = new boost::uuids::uuid[1];
  inputTemplateUuids2 = new boost::uuids::uuid[0];
  outputUuids2 = new boost::uuids::uuid[1];

  inputUuids2[0] = stream[7]->getId();
  outputUuids2[0] = stream[8]->getId();

  node2 = std::make_shared<ice::NodeDescription>("node6", inputUuids2, inputTemplateUuids2, outputUuids2, 1, 0, 1);
  model2->getNodeDescriptions()->push_back(node2);

  ice::ModelComperator mc;

  auto result = mc.findModelMatches(model1, model2);

  EXPECT_EQ(2, result->size());

  auto match = (*result)[0];
  EXPECT_EQ(3, match->getNodeDescriptions()->size());
  EXPECT_EQ(1, match->getInputTemplates()->size());
  EXPECT_EQ(stream[0]->getId(), match->getInputTemplates()->at(0)->getId());
  EXPECT_EQ(3, match->getOutputStreams()->size());
  EXPECT_EQ(stream[1]->getId(), match->getOutputStreams()->at(0)->getId());
  EXPECT_EQ(stream[2]->getId(), match->getOutputStreams()->at(1)->getId());
  EXPECT_EQ(stream[3]->getId(), match->getOutputStreams()->at(2)->getId());
  EXPECT_EQ(3, match->getConnectionMatrix()[0]);
  EXPECT_EQ(1, match->getConnectionMatrix()[1]);
  EXPECT_EQ(0, match->getConnectionMatrix()[2]);
  EXPECT_EQ(0, match->getConnectionMatrix()[3]);
  EXPECT_EQ(2, match->getConnectionMatrix()[4]);
  EXPECT_EQ(1, match->getConnectionMatrix()[5]);
  EXPECT_EQ(0, match->getConnectionMatrix()[6]);
  EXPECT_EQ(0, match->getConnectionMatrix()[7]);
  EXPECT_EQ(2, match->getConnectionMatrix()[8]);

  match = (*result)[1];
  EXPECT_EQ(2, match->getNodeDescriptions()->size());
  EXPECT_EQ(1, match->getInputTemplates()->size());
  EXPECT_EQ(stream[6]->getId(), match->getInputTemplates()->at(0)->getId());
  EXPECT_EQ(2, match->getOutputStreams()->size());
  EXPECT_EQ(stream[7]->getId(), match->getOutputStreams()->at(0)->getId());
  EXPECT_EQ(stream[8]->getId(), match->getOutputStreams()->at(1)->getId());
  EXPECT_EQ(3, match->getConnectionMatrix()[0]);
  EXPECT_EQ(1, match->getConnectionMatrix()[1]);
  EXPECT_EQ(0, match->getConnectionMatrix()[2]);
  EXPECT_EQ(2, match->getConnectionMatrix()[3]);
}

}

