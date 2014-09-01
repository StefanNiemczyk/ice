/*
 * text_XMLReader.cpp
 *
 *  Created on: May 27, 2014
 *      Author: sni
 */

#include <iostream>
#include <string>
#include <time.h>
#include <typeinfo>
#include <memory>

#include "ice/XMLReader.h"
#include "ice/processing/Node.h"

#include "gtest/gtest.h"

namespace
{

class TestXMLReader : public ::testing::Test
{
protected:
  time_t start_time_;

  virtual void SetUp()
  {
    start_time_ = time(NULL);
  }

  virtual void TearDown()
  {
    const time_t end_time = time(NULL);

    EXPECT_TRUE(end_time - start_time_ <= 5) << "The test took too long.";
  }
};

void testResult(ice::XMLReader& reader)
{
  //check information
    EXPECT_EQ(2, reader.getInformations()->size());

    int found = 0;

    for (auto info : *reader.getInformations())
    {
      if (info->uuid == "39f9b426-741c-4b89-8698-98ec2505db01")
      {
        found++;
        EXPECT_EQ("Position", info->type);
        EXPECT_EQ("Position", info->topic);
        EXPECT_EQ("Position of a Robot on the field", info->_desc);
        EXPECT_EQ(1, info->nested.size());

        EXPECT_EQ("39f9b426-741c-4b89-8698-98ec2505db02", info->nested[0]->uuid);
        EXPECT_EQ("Position", info->nested[0]->type);
        EXPECT_EQ("raw", info->nested[0]->topic);
        EXPECT_EQ("Raw position of a Robot on the field", info->nested[0]->_desc);
        EXPECT_EQ(0, info->nested[0]->nested.size());
      }
      else if (info->uuid == "39f9b426-741c-4b89-8698-98ec2505db03")
      {
        found++;
        EXPECT_EQ("List[Position]", info->type);
        EXPECT_EQ("Obstacle", info->topic);
        EXPECT_EQ("Position of obstacles", info->_desc);
        EXPECT_EQ(1, info->nested.size());

        EXPECT_EQ("39f9b426-741c-4b89-8698-98ec2505db04", info->nested[0]->uuid);
        EXPECT_EQ("List[Position]", info->nested[0]->type);
        EXPECT_EQ("fused", info->nested[0]->topic);
        EXPECT_EQ("Raw position of an obstacle on the field", info->nested[0]->_desc);
        EXPECT_EQ(0, info->nested[0]->nested.size());
      }
    }

    EXPECT_EQ(2, found);

    //check streams
    found = 0;

    EXPECT_EQ(4, reader.getStreams()->size());

    for (auto stream : *reader.getStreams())
    {
      if (stream->name == "/Position|own")
      {
        found++;
        EXPECT_EQ("39f9b426-741c-4b89-8698-98ec2505db01", stream->informationUuid);
        EXPECT_EQ("own", stream->provider);
        EXPECT_EQ("active", stream->sharingState);
        EXPECT_EQ(10, stream->sharingMaxCount);
        EXPECT_EQ(100, stream->size);
        EXPECT_EQ("Own position on the field", stream->_desc);
      }
      else if (stream->name == "/Position/raw|own")
      {
        found++;
        EXPECT_EQ("39f9b426-741c-4b89-8698-98ec2505db02", stream->informationUuid);
        EXPECT_EQ("own", stream->provider);
        EXPECT_EQ("inactive", stream->sharingState);
        EXPECT_EQ(0, stream->sharingMaxCount);
        EXPECT_EQ(50, stream->size);
        EXPECT_EQ("Raw position on the field", stream->_desc);
      }
      else if (stream->name == "/Obstacle|own")
      {
        found++;
        EXPECT_EQ("39f9b426-741c-4b89-8698-98ec2505db03", stream->informationUuid);
        EXPECT_EQ("own", stream->provider);
        EXPECT_EQ("active", stream->sharingState);
        EXPECT_EQ(10, stream->sharingMaxCount);
        EXPECT_EQ(50, stream->size);
        EXPECT_EQ("Obstacle positions on the field", stream->_desc);
      }
      else if (stream->name == "/Obstacle/fused|own")
      {
        found++;
        EXPECT_EQ("39f9b426-741c-4b89-8698-98ec2505db04", stream->informationUuid);
        EXPECT_EQ("own", stream->provider);
        EXPECT_EQ("active", stream->sharingState);
        EXPECT_EQ(10, stream->sharingMaxCount);
        EXPECT_EQ(20, stream->size);
        EXPECT_EQ("Fused list of obstacles", stream->_desc);
      }
    }

    EXPECT_EQ(4, found);

    //check stream requests
    found = 0;

    EXPECT_EQ(2, reader.getStreamTemplates()->size());

    for (auto stream : *reader.getStreamTemplates())
    {
      if (stream->name == "/Position|?provider")
      {
        found++;
        EXPECT_EQ("39f9b426-741c-4b89-8698-98ec2505db01", stream->informationUuid);
        EXPECT_EQ("$other", stream->provider);
        EXPECT_EQ(20, stream->size);
        EXPECT_EQ("Position of other robots on the field", stream->_desc);
      }
      else if (stream->name == "/Obstacle|?provider")
      {
        found++;
        EXPECT_EQ("39f9b426-741c-4b89-8698-98ec2505db03", stream->informationUuid);
        EXPECT_EQ("$other", stream->provider);
        EXPECT_EQ(20, stream->size);
        EXPECT_EQ("List of streams of the given providers", stream->_desc);
      }
    }

    EXPECT_EQ(2, found);

    //check nodes
    found = 0;

    EXPECT_EQ(3, reader.getNodes()->size());

    for (auto node : *reader.getNodes())
    {
      if (node->name == "SimpleSourceNode")
      {
        found++;
        EXPECT_EQ(ice::NodeType::SOURCE, node->type);
        EXPECT_EQ("Camera", node->source);
        EXPECT_EQ("SimpleSourceNode", node->className);
        EXPECT_EQ(-1, node->trigger);
        EXPECT_EQ(0, node->inputs.size());
        EXPECT_EQ(0, node->inputTemplates.size());
        EXPECT_EQ(1, node->outputs.size());
        EXPECT_EQ(0, node->configs.size());
        EXPECT_EQ("Source node which provides the raw position, is this required?", node->_desc);

        EXPECT_EQ("/Position/raw|own", node->outputs[0]->name);
      }
      else if (node->name == "SimpleProcessingNode")
      {
        found++;
        EXPECT_EQ(ice::NodeType::PROCESSING, node->type);
        EXPECT_EQ("", node->source);
        EXPECT_EQ("smothing", node->className);
        EXPECT_EQ(30, node->trigger);
        EXPECT_EQ(1, node->inputs.size());
        EXPECT_EQ(0, node->inputTemplates.size());
        EXPECT_EQ(1, node->outputs.size());
        EXPECT_EQ(0, node->configs.size());
        EXPECT_EQ("Processing node, creates the position based on the raw position", node->_desc);

        EXPECT_EQ("/Position/raw|own", node->inputs[0]->name);
        EXPECT_EQ(true, node->inputs[0]->trigger);
        EXPECT_EQ("/Position|own", node->outputs[0]->name);
      }
      else if (node->name == "ObstacleClustering")
      {
        found++;
        EXPECT_EQ(ice::NodeType::PROCESSING, node->type);
        EXPECT_EQ("", node->source);
        EXPECT_EQ("ObstacleFusing", node->className);
        EXPECT_EQ(-1, node->trigger);
        EXPECT_EQ(1, node->inputs.size());
        EXPECT_EQ(1, node->inputTemplates.size());
        EXPECT_EQ(1, node->outputs.size());
        EXPECT_EQ(2, node->configs.size());
        EXPECT_EQ("", node->_desc);

        EXPECT_EQ("/Obstacle|own", node->inputs[0]->name);
        EXPECT_EQ(true, node->inputs[0]->trigger);

        EXPECT_EQ("/Obstacle|?provider", node->inputTemplates[0]->name);

        EXPECT_EQ("/Obstacle/fused|own", node->outputs[0]->name);

        EXPECT_EQ("5", node->configs["history"]);
        EXPECT_EQ("true", node->configs["clustering"]);
      }
    }

    EXPECT_EQ(3, found);
}

TEST_F(TestXMLReader, simple_test)
{
  ice::XMLReader reader;
  bool result = reader.readFile("data/simple_example.xml");

  EXPECT_TRUE(result);

  testResult(reader);
}

TEST_F(TestXMLReader, split_file_test)
{
  ice::XMLReader reader;
  bool result = reader.readFiles( {"data/simple_example_split_1.xml", "data/simple_example_split_2.xml",
                                   "data/simple_example_split_3.xml"});

  EXPECT_TRUE(result);

  testResult(reader);
}

TEST_F(TestXMLReader, clear)
{
  ice::XMLReader reader;
  bool result = reader.readFile("data/simple_example.xml");

  reader.clear();

  EXPECT_EQ(0, reader.getInformations()->size());
  EXPECT_EQ(0, reader.getStreams()->size());
  EXPECT_EQ(0, reader.getStreamTemplates()->size());
  EXPECT_EQ(0, reader.getNodes()->size());
}

}

