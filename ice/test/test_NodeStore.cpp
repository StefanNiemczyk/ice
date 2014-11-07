#include <gtest/gtest.h>

#include "ice/ICEngine.h"
#include "ice/processing/NodeStore.h"

#include "etc/EngineStuff.cpp"
#include "etc/TestTime.cpp"

TEST(NodeStoreTest, create)
{
  ice::Node::registerNodeCreator("SimpleSourceNode", &SimpleSourceNode::createNode);
  ice::Node::registerNodeCreator("smothing", &SmothingNode::createNode);


  auto streamFactory = std::make_shared<TestFactory>();
  auto timeFactory = std::make_shared<TestTimeFactory>();
  std::shared_ptr<ice::ICEngine> engine = std::make_shared<ice::ICEngine>(timeFactory, streamFactory);
  ice::NodeStore store(engine);

  std::map<std::string, std::string> config;
  auto node = store.registerNode(ice::NodeType::PROCESSING, "smothing", "SmothingProcessingNode", "testEntity1", config);

  ASSERT_TRUE(node ? true : false);
}
