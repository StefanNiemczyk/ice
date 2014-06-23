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

#include "ice/processing/Node.h"

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

  virtual int performNode()
  {
    return 0;
  }
};

class TestNode : public ::testing::Test
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

TEST_F(TestNode, register_create)
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

}

