/*
 * test_Node.cpp
 *
 *  Created on: Jun 3, 2014
 *      Author: sni
 */

#include <iostream>
#include <string>
#include <time.h>
#include <typeinfo>
#include <memory>

#include "ice/container/Position.h"
#include "ice/processing/Node.h"
#include "etc/EngineStuff.cpp"

#include "gtest/gtest.h"

namespace
{

class SimpleNode : public ice::Node
{
public:

  static std::shared_ptr<ice::Node> createNode()
  {
    return std::make_shared<SimpleNode>();
  }

  SimpleNode() :
      Node()
  {

  }

  virtual std::string getClassName()
  {
    return "";
  }

  virtual const int newEvent(std::shared_ptr<ice::InformationElement<ice::GContainer>> element,
                             std::shared_ptr<ice::InformationCollection> collection)
  {
    return 0;
  }

  virtual int performNode()
  {
    return 0;
  }
};

TEST(TestNode, register_create)
{
  ice::Node::creatorFunc func = &SimpleNode::createNode;
  int resultValue = ice::Node::registerNodeCreator("SimpleNode", func);
  EXPECT_EQ(0, resultValue);

  resultValue = ice::Node::registerNodeCreator("SimpleNode", &SimpleNode::createNode);
  EXPECT_EQ(1, resultValue);

  std::shared_ptr<ice::Node> node = ice::Node::createNode("SimpleNode");
  EXPECT_TRUE((node ? true : false));

  node = ice::Node::createNode("asdfasdf");
  EXPECT_FALSE((node ? true : false));
}

TEST(TestNode, set_node)
{
  ice::Node::creatorFunc func = &SetTestNode::createNode;
  int resultValue = ice::Node::registerNodeCreator("SetTestNode", func);
  EXPECT_EQ(0, resultValue);

  std::shared_ptr<ice::Node> node = ice::Node::createNode("SetTestNode");
  EXPECT_TRUE((node ? true : false));

  auto handler = std::make_shared<ice::EventHandler>(2, 100);
  auto spec = std::make_shared<ice::InformationSpecification>("", "type", "scope", "rep");
  std::map<std::string, int> metadatas;
  auto description = std::make_shared<ice::CollectionDescription>(spec, "name,", "provider", "source", metadatas);
  auto input = std::make_shared<ice::InformationSet<ice::Position>>(description, handler);
  auto output = std::make_shared<ice::InformationSet<ice::Position>>(description, handler);

  node->setEventHandler(handler);
  node->addInputSet(input, true);
  node->addOutputSet(output);

  resultValue = node->init();
  node->activate();
  EXPECT_EQ(0, resultValue);

  auto pos = std::make_shared<ice::Position>();
  pos->x = 1;
  pos->y = 2;
  pos->z = 3;
  input->add("muh", pos);

  std::this_thread::sleep_for(std::chrono::milliseconds {500});

  ASSERT_EQ(1, output->getSize());
  auto posOut = output->get("muh");
  ASSERT_TRUE((posOut != nullptr));

  ASSERT_EQ(0, posOut->getInformation()->x);
  ASSERT_EQ(1, posOut->getInformation()->y);
  ASSERT_EQ(2, posOut->getInformation()->z);
}

}

