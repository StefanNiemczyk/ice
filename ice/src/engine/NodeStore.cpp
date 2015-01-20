/*
 * NodeStore.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: sni
 */

#include "ice/processing/NodeStore.h"
#include "ice/ICEngine.h"
#include "easylogging++.h"

namespace ice
{

NodeStore::NodeStore(std::weak_ptr<ICEngine> engine)
{
  _log = el::Loggers::getLogger("NodeStore");
  this->engine = engine;
}

NodeStore::~NodeStore()
{
  //
}

int NodeStore::addNode(std::shared_ptr<Node> node)
{
  std::lock_guard<std::mutex> guard(this->mtx_);

  for (auto nodeItr : this->nodes)
  {
    if (nodeItr->getNodeDescription()->getName() == node->getNodeDescription()->getName())
      return 1;
  }

  this->nodes.push_back(node);

  return 0;
}

std::shared_ptr<Node> NodeStore::getNode(const std::string nodeName, const ont::entity entity)
{
  if ("" == nodeName || entity == "")
  {
    std::shared_ptr<Node> ptr;
    return ptr;
  }

  std::lock_guard<std::mutex> guard(this->mtx_);

  for (auto node : this->nodes)
  {
    auto desc = node->getNodeDescription();
    if (desc->getName() == nodeName && desc->getEntity() == entity)
      return node;
  }

  std::shared_ptr<Node> ptr;
  return ptr;
}

std::shared_ptr<Node> NodeStore::registerNode(const NodeType type, const std::string className, const std::string name,
                                              const ont::entity entity, std::map<std::string, std::string> config,
                                              const std::string source)
{
  auto node = this->getNode(name, entity);

  if (node)
  {
    _log->info("Node '%v' already registered for entity '%v'", name.c_str(), entity.c_str());
    return node;
  }

  auto desc = std::make_shared<NodeDescription>(type, className, name, entity);

  if (type == NodeType::SOURCE)
  {
    desc->setSource(source);
  }

  node = Node::createNode(className);

  if (false == node)
  {
    _log->error("Node '%v' could not be created for entity '%v'. Register missing?", name.c_str(),
                entity.c_str());
    return node;
  }

  node->setNodeDescription(desc);
  node->setConfiguration(config);

  return node;
}

bool NodeStore::existNodeCreator(const std::string className)
{
  return Node::existNodeCreator(className);
}

void NodeStore::cleanUpUnusedNodes(std::vector<std::shared_ptr<Node>> &usedNodes)
{
  _log->verbose(1, "Start removing unused nodes");
  int counter = 0;

  for (int i = 0; i < this->nodes.size(); ++i)
  {
    auto node = this->nodes.at(i);
    bool found = false;

    for (auto usedNode : usedNodes)
    {
      if (usedNode == node)
      {
        found = true;
        break;
      }
    }

    if (found)
      continue;

    _log->info("Remove unused node %v", node->toString().c_str());
    counter++;

    node->deactivate();
    node->destroy();

    --i;
  }

  _log->info("Clean up node store: '%v' nodes are removed", counter);
}

void NodeStore::cleanUpNodes(std::vector<std::shared_ptr<Node>> &nodesToCleanUp)
{
  _log->verbose(1, "Start removing nodes");
  int counter = 0;

  for (auto node : nodesToCleanUp)
  {
    _log->info("Remove node %v", node->toString().c_str());
    counter++;

    node->deactivate();
    node->destroy();
  }

  _log->info("Clean up node store: '%v' nodes are removed", counter);
}

//bool NodeStore::addDescriptionsToInformationModel(std::shared_ptr<InformationModel> informationModel)
//{
//  std::lock_guard<std::mutex> guard(this->mtx_);
//  bool returnValue = false;
//
//  for (auto node : this->nodes)
//  {
//    informationModel->getNodeDescriptions()->push_back(node->getNodeDescription());
//    returnValue = true;
//  }
//
//  return returnValue;
//}

} /* namespace ice */
