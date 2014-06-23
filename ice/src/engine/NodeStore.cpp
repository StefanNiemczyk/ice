/*
 * NodeStore.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: sni
 */

#include "ice/processing/NodeStore.h"
#include "ice/ICEngine.h"

namespace ice
{

NodeStore::NodeStore(std::weak_ptr<ICEngine> engine)
{
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
    if (nodeItr->getName() == node->getName())
      return 1;
  }

  this->nodes.push_back(node);

  return 0;
}

std::shared_ptr<Node> NodeStore::getNode(const std::string nodeName)
{
  if ("" == nodeName)
  {
    std::shared_ptr<Node> ptr;
    return ptr;
  }

  std::lock_guard<std::mutex> guard(this->mtx_);

  for (auto node : this->nodes)
  {
    if (node->getName() == nodeName)
      return node;
  }

  std::shared_ptr<Node> ptr;
  return ptr;
}

bool NodeStore::addDescriptionsToInformationModel(std::shared_ptr<InformationModel> informationModel)
{
  std::lock_guard<std::mutex> guard(this->mtx_);
  bool returnValue = false;

  for (auto node : this->nodes)
  {
    informationModel->getNodeDescriptions()->push_back(node->getNodeDescription());
    returnValue = true;
  }

  return returnValue;
}

} /* namespace ice */
