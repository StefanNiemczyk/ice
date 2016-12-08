/*
 * NodeStore.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: sni
 */

#include "ice/model/updateStrategie/UpdateStrategie.h"
#include "ice/processing/NodeStore.h"
#include "ice/ICEngine.h"
#include "ice/Time.h"
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
  this->cleanUp();
}

void NodeStore::init()
{
  auto e = this->engine.lock();
  this->timeFactory = e->getTimeFactory();
  this->gcontainerFactory = e->getGContainerFactory();
}

void NodeStore::cleanUp()
{
  for (auto &node : this->nodes)
  {
    node->deactivate();
    node->destroy();
  }

  this->nodes.clear();
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
  if ("" == nodeName && entity == "")
  {
    return nullptr;
  }

  std::lock_guard<std::mutex> guard(this->mtx_);

  for (auto node : this->nodes)
  {
    auto desc = node->getNodeDescription();
    if (desc->getName() == nodeName && desc->getEntity() == entity)
    {
      return node;
    }
  }

  _log->info("No node found for nodename '%v'", nodeName);

  return nullptr;
}

std::shared_ptr<Node> NodeStore::registerNode(const NodeType type, const std::string className, const std::string name,
                                              const ont::entity entity, const ont::entity entityRelated,
                                              std::map<std::string, std::string> &config, const std::string source)
{
  auto node = this->getNode(name, entity);

  if (node)
  {
    _log->info("Node '%v' already registered for entity '%v' / '%v'", name, entity, entityRelated);
    return node;
  }

  auto desc = std::make_shared<NodeDescription>(type, className, name, entity, entityRelated);

  if (type == NodeType::SOURCE)
  {
    desc->setSource(source);
  }

  auto creator = Node::getNodeCreator(className);

  if (creator== nullptr)
  {
    _log->error("Node '%v' could not be created for entity '%v'. Register missing?", name,
                entity);
    return node;
  }

  if (creator->defect)
  {
    _log->error("Node '%v' could not be created for entity '%v'. Node is marked as defekt", name,
                entity);
    return node;
  }

  node = (creator->func)();
  node->setNodeStore(this->shared_from_this());
  node->setNodeDescription(desc);
  node->setConfiguration(config);
  node->setCreatorName(className);
  node->setTimeFactory(this->timeFactory);
  node->setGContainerFactory(this->gcontainerFactory);

  this->addNode(node);

  return node;
}

void NodeStore::handleNodeFailure(std::string className)
{
  _log->info("Handle node failure for creator '%v'", className);
  auto creator = Node::getNodeCreator(className);

  if (creator == nullptr)
  {
    _log->info("Handle node failure for unknown creator '%v', skipped", className);
    return;
  }

  if (creator->defect)
  {
    _log->info("Handle known node failure for creator '%v', skipped", className);
    return;
  }

  creator->defect = true;

  {
    if (engine.expired())
      return;

    auto e = engine.lock();
    e->getUpdateStrategie()->onNodeFailure(className);
  }
}

void NodeStore::handleNodeRepair(std::string className)
{
  _log->info("Handle node repair for creator '%v'", className);
  auto creator = Node::getNodeCreator(className);

  if (creator == nullptr)
  {
    _log->info("Handle node repair for unknown creator '%v', skipped", className);
    return;
  }

  if (creator->defect == false)
  {
    _log->info("Handle known node repair for creator '%v', skipped", className);
    return;
  }

  creator->defect = false;

  {
    if (engine.expired())
      return;

    auto e = engine.lock();
    e->getUpdateStrategie()->onNodeRepair(className);
  }
}

bool NodeStore::existNodeCreator(const std::string className)
{
  return Node::existNodeCreator(className);
}

void NodeStore::unregisterAndCleanUp(std::shared_ptr<Entity> &entity,
                                     std::vector<std::shared_ptr<Node>> &nodesToCleanUp)
{
  _log->verbose(1, "Start unregistering engine from nodes");
  int counter = 0;

  for (int i = 0; i < this->nodes.size(); ++i)
  {
    auto node = this->nodes.at(i);
    bool found = false;

    for (auto node : nodesToCleanUp)
    {
      node->unregisterEntity(entity);
    }
  }

  this->cleanUpNodes();
}

void NodeStore::cleanUpNodes()
{
  _log->verbose(1, "Start removing unused nodes");
  int counter = 0;

  for (auto node : this->nodes)
  {
    if (node->getRegisteredEngineCount() > 0)
      continue;

    _log->info("Remove node %v", node->toString());
    counter++;

    node->deactivate();
    node->destroy();
  }

  _log->info("Clean up node store: '%v' nodes are removed", counter);
}

void NodeStore::cleanUpNodes(std::shared_ptr<Entity> &entity)
{
  _log->verbose(1, "Start removing nodes for entity '%v'", entity->toString());
  int counter = 0;

  for (auto node : this->nodes)
  {
    node->unregisterEntity(entity);

    if (node->getRegisteredEngineCount() > 0)
      continue;

    _log->info("Remove node %v", node->toString());
    counter++;

    node->deactivate();
    node->destroy();
  }

  _log->info("Clean up node store: '%v' nodes are removed", counter);
}

void NodeStore::cleanUpNodes(std::vector<std::shared_ptr<Node>> &nodesToCleanUp)
{
  _log->verbose(1, "Start removing nodes");
  int counter = 0;

  for (auto node : nodesToCleanUp)
  {
    _log->info("Remove node %v", node->toString());
    counter++;

    node->deactivate();
    node->destroy();
  }

  _log->info("Clean up node store: '%v' nodes are removed", counter);
}

} /* namespace ice */
