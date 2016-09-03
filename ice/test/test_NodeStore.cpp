#include <gtest/gtest.h>

#include "ice/ICEngine.h"
#include "ice/processing/NodeStore.h"

#include "etc/EngineStuff.cpp"

TEST(NodeStoreTest, create)
{
  ice::Node::registerNodeCreator("SimpleSourceNode", &SimpleSourceNode::createNode);
  ice::Node::registerNodeCreator("smothing", &SmothingNode::createNode);


  auto timeFactory = std::make_shared<ice::SimpleTimeFactory>();

  std::shared_ptr<ice::Configuration> config = std::make_shared<ice::Configuration>();
  config->ontologyIri = "http://vs.uni-kassel.de/IceTest";
  config->ontologyIriOwnEntity = "http://vs.uni-kassel.de/IceTest#TestSystem";
  std::shared_ptr<ice::ICEngine> engine = std::make_shared<ice::ICEngine>(config);
  auto streamFactory = std::make_shared<TestFactory>(engine);
  engine->setTimeFactory(timeFactory);
  engine->setStreamFactory(streamFactory);

  engine->init();

  std::map<std::string, std::string> configNode;
//  const NodeType type, const std::string className, const std::string name,
//                                       const ont::entity entity, const ont::entity entityRelated,
//                                       std::map<std::string, std::string> config,
  auto node = engine->getNodeStore()->registerNode(ice::NodeType::PROCESSING, "smothing", "SmothingProcessingNode", "testEntity1", "none", configNode);

  ASSERT_TRUE(node ? true : false);
}
