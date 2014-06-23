/*
 * InformationStreamTemplate.cpp
 *
 *  Created on: Jun 2, 2014
 *      Author: sni
 */

#include "ice/information/InformationStreamTemplate.h"
#include "ice/information/StreamFactory.h"
#include "ice/processing/Node.h"

namespace ice
{
InformationStreamTemplate::InformationStreamTemplate(std::shared_ptr<StreamFactory> streamFactory,
                                                     std::string className, const std::string name,
                                                     std::weak_ptr<InformationType> informationType,
                                                     std::shared_ptr<EventHandler> eventHandler,
                                                     std::shared_ptr<InformationSpecification> specification,
                                                     int streamSize, std::string provider, std::string description) :
    BaseInformationStream(name, informationType, eventHandler, specification, provider, description)
{
  this->streamFactory = streamFactory;
  this->className = className;
  this->streamSize = streamSize;
}

InformationStreamTemplate::~InformationStreamTemplate()
{
  // nothing to do here
}

std::shared_ptr<BaseInformationStream> InformationStreamTemplate::createBaseStream(const std::string provider)
{
  std::string name = this->name;

  name.replace(name.find("?provider"), 9, provider);

  //const std::string& className,
  //const std::string name,
  //std::weak_ptr<InformationType> informationType,
  //std::shared_ptr<EventHandler> eventHandler,
  //std::shared_ptr<InformationSpecification> specification,
  //int streamSize,
  //std::string provider,
  //std::string description
  auto baseStream = this->streamFactory->createStream(this->className, name, this->informationType, this->eventHandler,
                                                      this->specification, this->streamSize, provider,
                                                      this->description, this->shared);

  baseStream->setStreamTemplate(this->shared_from_this());

  {
    std::lock_guard<std::mutex> guard(this->_mtx);
    this->streamsCreated.push_back(baseStream);

    for (auto node : this->registeredNodes)
    {
      node.node->addInput(baseStream, node.trigger, false);
    }
  }

  return baseStream;
}

int InformationStreamTemplate::registerNode(std::shared_ptr<Node> node, bool trigger)
{
  std::lock_guard<std::mutex> guard(this->_mtx);

  for (auto nodeItr : this->registeredNodes)
  {
    if (node == nodeItr.node)
    {
      return 1;
    }
  }

  RegisteredNode rn;

  rn.node = node;
  rn.trigger = trigger;

  this->registeredNodes.push_back(rn);

  for (auto stream : this->streamsCreated)
  {
    if (false == stream.expired())
    {
      node->addInput(stream.lock(), trigger, false);
    }
  }

  return 0;
}

int InformationStreamTemplate::unregisterNode(std::shared_ptr<Node> node)
{
  std::lock_guard<std::mutex> guard(this->_mtx);
  int index = -1;

  for (int i = 0; i < this->registeredNodes.size(); ++i)
  {
    std::shared_ptr<Node> nodeItr = this->registeredNodes[i].node;
    if (node == nodeItr)
    {
      index = i;
      break;
    }
  }
  if (index != -1)
  {
    this->registeredNodes.erase(this->registeredNodes.begin() + index);

    for (auto stream : this->streamsCreated)
    {
      if (stream.expired())
        continue;

      auto baseStream = stream.lock();

      node->removeInput(baseStream);
    }

    return 0;
  }

  return 1;
}

const std::type_info* InformationStreamTemplate::getTypeInfo() const
{
  const std::type_info* ptr = &typeid(this);

  return ptr;
}

std::shared_ptr<StreamTemplateDescription> ice::InformationStreamTemplate::getStreamTemplateDescription()
{
  if (this->streamTemplateDescription)
    return this->streamTemplateDescription;

  std::lock_guard<std::mutex> guard(this->_mtx);

  if (this->streamTemplateDescription)
    return this->streamTemplateDescription;

  this->streamTemplateDescription = std::make_shared<StreamTemplateDescription>(this->getSpecification()->getUUID());

  return this->streamTemplateDescription;
}

void InformationStreamTemplate::allEngineStatesUnregistered()
{
}

} /* namespace ice */
