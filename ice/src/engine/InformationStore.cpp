/*
 * InformationStore.cpp
 *
 *  Created on: May 19, 2014
 *      Author: sni
 */

#include "ice/information/InformationStore.h"
#include "ice/ICEngine.h"

namespace ice
{

InformationStore::InformationStore(std::weak_ptr<ICEngine> engine)
{
  this->engine = engine;
  std::shared_ptr<ICEngine> engineObject;

  if (engine.expired())
    return;

  engineObject = engine.lock();

  this->eventHandler = engineObject->getEventHandler();
  this->config = engineObject->getConfig();
  this->_log = Logger::get("InformationStore");
}

InformationStore::InformationStore(std::shared_ptr<EventHandler> eventHandler)
{
  this->eventHandler = eventHandler;
  this->_log = Logger::get("InformationStore");
}

InformationStore::~InformationStore()
{
  // TODO Auto-generated destructor stub
}

boost::uuids::uuid InformationStore::getUUIDByName(std::string name)
{
  std::shared_ptr<InformationType> ptr;
  boost::uuids::uuid uuid;

  if (name == "")
    return uuid;

  for (auto type : this->informationTypes)
  {
    if (type->getSpecification()->getName() == name)
    {
      uuid = type->getSpecification()->getUUID();
      break;
    }
  }

  return uuid;
}

std::shared_ptr<InformationType> InformationStore::registerInformationType(
    std::shared_ptr<InformationSpecification> specification)
{
  std::shared_ptr<InformationType> ptr;

  ptr = this->getInformationType(specification->getUUID());

  if (ptr)
    return ptr;

  auto type = std::make_shared<InformationType>(this->shared_from_this(), specification);
  this->informationTypes.push_back(type);

  return type;
}

std::shared_ptr<InformationType> InformationStore::getInformationType(const boost::uuids::uuid& uuid) const
{
  std::shared_ptr<InformationType> ptr;

  for (auto type : this->informationTypes)
  {
    if (type->getSpecification()->getUUID() == uuid)
    {
      ptr = type;
      break;
    }
  }

  return ptr;
}

std::shared_ptr<InformationType> InformationStore::getInformationType(const std::string& name) const
{
  std::shared_ptr<InformationType> ptr;

  if (name == "")
    return ptr;

  for (auto type : this->informationTypes)
  {
    if (type->getSpecification()->getName() == name)
    {
      return type;
    }
  }

  return ptr;
}

std::shared_ptr<Configuration> InformationStore::getConfig() const
{
  return this->config;
}

std::shared_ptr<EventHandler> InformationStore::getEventHandler() const
{
  return this->eventHandler;
}

int InformationStore::addStream(std::shared_ptr<BaseInformationStream> stream)
{
  if (this->streams.find(stream->getName()) != this->streams.end())
  {
    return 1;
  }

  _log->debug("addStream", "Adding stream %s to map", stream->getName().c_str());

  this->streams.insert(std::pair<std::string, std::shared_ptr<BaseInformationStream>>(stream->getName(), stream));

  return 0;
}

std::shared_ptr<BaseInformationStream> InformationStore::getBaseStream(const std::string streamName)
{
  if (this->streams.find(streamName) == this->streams.end())
  {
    std::shared_ptr<BaseInformationStream> stream;
    return stream;
  }

  return this->streams[streamName];
}

std::shared_ptr<BaseInformationStream> InformationStore::getBaseStream(
    const std::shared_ptr<StreamDescription> streamDescription)
{
  auto type = this->getInformationType(streamDescription->getUuid());

  if (false == type)
  {
    std::shared_ptr<BaseInformationStream> ptr;
    return ptr;
  }

  return type->getBaseStream(streamDescription);
}

std::shared_ptr<BaseInformationStream> InformationStore::getBaseStream(
    const std::shared_ptr<StreamTemplateDescription> streamTemplateDescription)
{
  auto type = this->getInformationType(streamTemplateDescription->getUuid());

  if (false == type)
  {
    std::shared_ptr<BaseInformationStream> ptr;
    return ptr;
  }

  return type->getBaseStream(streamTemplateDescription);
}

int InformationStore::addStreamTemplate(std::shared_ptr<InformationStreamTemplate> streamTemplate)
{
  if (this->streamTemplates.find(streamTemplate->getName()) != this->streamTemplates.end())
  {
    return 1;
  }

  this->streamTemplates.insert(
      std::pair<std::string, std::shared_ptr<InformationStreamTemplate>>(streamTemplate->getName(), streamTemplate));

  return 0;
}

std::shared_ptr<InformationStreamTemplate> InformationStore::getStreamTemplate(const std::string streamTemplateName)
{
  if (this->streamTemplates.find(streamTemplateName) == this->streamTemplates.end())
  {
    std::shared_ptr<InformationStreamTemplate> streamTemplate;
    return streamTemplate;
  }

  return this->streamTemplates[streamTemplateName];
}

bool ice::InformationStore::addDescriptionsToInformationModel(std::shared_ptr<InformationModel> informationModel)
{
  std::lock_guard<std::mutex> guard(this->_mtx);
  bool returnValue = false;

  for (auto it = this->streams.begin(); it != this->streams.end(); ++it)
  {
    informationModel->getStreams()->push_back(it->second->getStreamDescription());
    returnValue = true;
  }

  for (auto it = this->streamTemplates.begin(); it != this->streamTemplates.end(); ++it)
  {
    informationModel->getStreamTemplates()->push_back(it->second->getStreamTemplateDescription());
    returnValue = true;
  }

  return returnValue;
}

} /* namespace ice */

