/*
 * InformationStore.cpp
 *
 *  Created on: May 19, 2014
 *      Author: sni
 */

#include "ice/information/InformationStore.h"

#include "ice/ICEngine.h"
#include "ice/information/StreamFactory.h"
#include "easylogging++.h"

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
  this->streamFactory = engineObject->getStreamFactory();
  this->_log = el::Loggers::getLogger("InformationStore");
}

InformationStore::InformationStore(std::shared_ptr<EventHandler> eventHandler)
{
  this->eventHandler = eventHandler;
  this->_log = el::Loggers::getLogger("InformationStore");
}

InformationStore::~InformationStore()
{
  // TODO Auto-generated destructor stub
}

std::shared_ptr<BaseInformationStream> InformationStore::registerBaseStream(
    std::string dataType, std::shared_ptr<InformationSpecification> specification, const std::string name,
    const int streamSize, std::map<std::string, int> metadata, std::string provider, std::string sourceSystem)
{
  auto ptr = this->getBaseStream(specification.get(), provider, sourceSystem);

  //stream already registered
  if (ptr)
  {
    _log->warn("InformationStore: Duplicated Stream with '%s', '%s', '%s'",
                  specification->toString().c_str(), provider.c_str(), sourceSystem.c_str());
    return ptr;
  }

  auto desc = std::make_shared<StreamDescription>(specification, name, provider, sourceSystem, metadata);
  auto stream = this->streamFactory->createStream(dataType, desc, this->eventHandler, streamSize);

  if (stream)
  {
    _log->debug("Created stream with '%s', '%s', '%s'", specification->toString().c_str(),
                provider.c_str(), sourceSystem.c_str());
    this->streams.push_back(stream);
  }
  else
  {
    _log->error("Stream with '%s', '%s', '%s' could not be created", specification->toString().c_str(),
                provider.c_str(), sourceSystem.c_str());
  }
  return stream;
}

//boost::uuids::uuid InformationStore::getUUIDByName(std::string name)
//{
//  std::shared_ptr<InformationType> ptr;
//  boost::uuids::uuid uuid;
//
//  if (name == "")
//    return uuid;
//
//  for (auto type : this->informationTypes)
//  {
//    if (type->getSpecification()->getName() == name)
//    {
//      uuid = type->getSpecification()->getUUID();
//      break;
//    }
//  }
//
//  return uuid;
//}

//std::shared_ptr<InformationType> InformationStore::registerInformationType(
//    std::shared_ptr<InformationSpecification> specification)
//{
//  std::shared_ptr<InformationType> ptr;
//
//  ptr = this->getInformationType(specification->getUUID());
//
//  if (ptr)
//    return ptr;
//
//  auto type = std::make_shared<InformationType>(this->shared_from_this(), specification);
//  this->informationTypes.push_back(type);
//
//  return type;
//}
//
//std::shared_ptr<InformationType> InformationStore::getInformationType(const boost::uuids::uuid& uuid) const
//{
//  std::shared_ptr<InformationType> ptr;
//
//  for (auto type : this->informationTypes)
//  {
//    if (type->getSpecification()->getUUID() == uuid)
//    {
//      ptr = type;
//      break;
//    }
//  }
//
//  return ptr;
//}

//std::shared_ptr<InformationType> InformationStore::getInformationType(const std::string& name) const
//{
//  std::shared_ptr<InformationType> ptr;
//
//  if (name == "")
//    return ptr;
//
//  for (auto type : this->informationTypes)
//  {
//    if (type->getSpecification()->getName() == name)
//    {
//      return type;
//    }
//  }
//
//  return ptr;
//}

std::shared_ptr<Configuration> InformationStore::getConfig() const
{
  return this->config;
}

std::shared_ptr<EventHandler> InformationStore::getEventHandler() const
{
  return this->eventHandler;
}

//int InformationStore::addStream(std::shared_ptr<BaseInformationStream> stream)
//{
//  if (this->streamMap.find(stream->getName()) != this->streamMap.end())
//  {
//    return 1;
//  }
//
//  _log->debug("Adding stream %s to map", stream->getName().c_str());
//
//  this->streamMap[stream->getName()] = stream;
//
//  return 0;
//}
std::shared_ptr<BaseInformationStream> InformationStore::getBaseStream(InformationSpecification * specification)
{
  return this->getBaseStream(specification, "", "");
}

std::shared_ptr<BaseInformationStream> InformationStore::getBaseStream(InformationSpecification *specification,
                                                                       std::string provider, std::string sourceSystem)
{
  _log->debug("Get stream by '%s', '%s', '%s'", specification->toString().c_str(), provider.c_str(),
              sourceSystem.c_str());

  std::vector<std::shared_ptr<BaseInformationStream>> selected;

  for (auto stream : this->streams)
  {
    auto spec = stream->getStreamDescription();
    if (*spec->getInformationSpecification() == *specification)
    {
      if (provider != "")
      {
        if (provider == spec->getProvider() && sourceSystem == spec->getSourceSystem())
        {
          selected.push_back(stream);
        }
      }
      else
      {
        selected.push_back(stream);
      }
    }
  }

  if (selected.size() == 0)
  {
    return std::shared_ptr<BaseInformationStream>();
  }

  return selectBestStream(&selected);
}

std::shared_ptr<BaseInformationStream> InformationStore::selectBestStream(
    std::vector<std::shared_ptr<BaseInformationStream>> *streams)
{
  if (streams->size() == 0)
  {
    return std::shared_ptr<BaseInformationStream>();
  }

  auto best = streams->at(0);

  for (int i = 0; i < streams->size(); ++i)
  {
    // TODO
  }

  return best;
}

std::shared_ptr<BaseInformationStream> InformationStore::getBaseStream(
    const std::shared_ptr<StreamDescription> streamDescription)
{
  return this->getBaseStream(streamDescription->getInformationSpecification().get(), streamDescription->getProvider(),
                             streamDescription->getSourceSystem());
}

//std::shared_ptr<BaseInformationStream> InformationStore::getBaseStream(
//    const std::shared_ptr<StreamTemplateDescription> streamTemplateDescription)
//{
//  auto type = this->getInformationType(streamTemplateDescription->getId());
//
//  if (false == type)
//  {
//    std::shared_ptr<BaseInformationStream> ptr;
//    return ptr;
//  }
//
//  return type->getBaseStream(streamTemplateDescription);
//}

//int InformationStore::addStreamTemplate(std::shared_ptr<InformationStreamTemplate> streamTemplate)
//{
//  if (this->streamTemplates.find(streamTemplate->getName()) != this->streamTemplates.end())
//  {
//    return 1;
//  }
//
//  this->streamTemplates.insert(
//      std::pair<std::string, std::shared_ptr<InformationStreamTemplate>>(streamTemplate->getName(), streamTemplate));
//
//  return 0;
//}

//std::shared_ptr<InformationStreamTemplate> InformationStore::getStreamTemplate(const std::string streamTemplateName)
//{
//  if (this->streamTemplates.find(streamTemplateName) == this->streamTemplates.end())
//  {
//    std::shared_ptr<InformationStreamTemplate> streamTemplate;
//    return streamTemplate;
//  }
//
//  return this->streamTemplates[streamTemplateName];
//}

bool ice::InformationStore::addDescriptionsToInformationModel(std::shared_ptr<InformationModel> informationModel)
{
  std::lock_guard<std::mutex> guard(this->_mtx);
  bool returnValue = false;

//  for (auto it = this->streamMap.begin(); it != this->streamMap.end(); ++it)
//  {
//    informationModel->getStreams()->push_back(it->second->getStreamDescription());
//    returnValue = true;
//  }

//  for (auto it = this->streamTemplates.begin(); it != this->streamTemplates.end(); ++it)
//  {
//    informationModel->getStreamTemplates()->push_back(it->second->getStreamTemplateDescription());
//    returnValue = true;
//  }

  return returnValue;
}

void InformationStore::cleanUpStreams()
{
  std::lock_guard<std::mutex> guard(this->_mtx);
  _log->verbose(1, "Start removing unused streams");
  int counter = 0;

  for (int i=0; i < this->streams.size(); ++i)
  {
    auto stream = this->streams.at(i);
    _log->info("Checking stream '%s', reference count %d", stream->toString().c_str(), stream.use_count());

    if (stream.use_count() == 2)
    {
      _log->info("Remove unused stream '%s'", stream->toString().c_str());

      ++counter;
      this->streams.erase(this->streams.begin() + i);
      --i;
    }
  }

  _log->info("Clean up information store: '%d' streams are removed", counter);
}

} /* namespace ice */

