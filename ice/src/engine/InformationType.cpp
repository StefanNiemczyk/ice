/*
 * InformationType.cpp
 *
 *  Created on: May 28, 2014
 *      Author: sni
 */

#include "ice/information/InformationType.h"

namespace ice
{

InformationType::InformationType(std::weak_ptr<InformationStore> informationStore,
                                 std::shared_ptr<InformationSpecification> specification)
{
  this->informationStore = informationStore;
  this->specification = specification;
}

InformationType::~InformationType()
{
  //nothing to do here
}

std::shared_ptr<BaseInformationStream> InformationType::registerStream(std::shared_ptr<BaseInformationStream> stream,
                                                                       bool* notAdded)
{
  auto ptr = this->getBaseStream(stream->getName());

  //stream already registered
  if (ptr)
  {
    *notAdded = true;
    return ptr;
  }

  //Specification not equal
  if (this->specification->getUUID() != stream->getSpecification()->getUUID())
  {
    *notAdded = true;
    return ptr;
  }

  int returnVel = this->registerStreamInStore(stream);

  if (returnVel == 1)
  {
    std::cout << "InformationStore: Duplicated Stream with name '" << stream->getName() << "'" << std::endl;
  }

  *notAdded = false;
  this->streams.push_back(stream);

  return stream;
}

std::shared_ptr<InformationSpecification> InformationType::getSpecification() const
{
  return this->specification;
}

std::shared_ptr<EventHandler> InformationType::getEventHandler()
{
  std::shared_ptr<InformationStore> store = this->informationStore.lock();
  return store->getEventHandler();
}

std::shared_ptr<BaseInformationStream> InformationType::getBaseStream(const std::string& name)
{
  std::shared_ptr<BaseInformationStream> returnPtr;

  if (name == "")
    return returnPtr;

  for (auto stream : this->streams)
  {
    if (stream->getName() == name)
    {
      return stream;
    }
  }

  return returnPtr;
}
std::shared_ptr<BaseInformationStream> InformationType::getBaseStream(
    const std::shared_ptr<StreamDescription> streamDescription)
{
  std::shared_ptr<BaseInformationStream> returnPtr;

  if (false == streamDescription)
    return returnPtr;

  for (auto stream : this->streams)
  {
    if (stream->getProvider() == "own")
    {
      return stream;
    }
  }

  return returnPtr;
}

std::shared_ptr<BaseInformationStream> InformationType::getBaseStream(
    const std::shared_ptr<StreamTemplateDescription> streamTemplateDescription)
{
  std::shared_ptr<BaseInformationStream> returnPtr;

  if (false == streamTemplateDescription)
    return returnPtr;

  for (auto stream : this->streams)
  {
    if (stream->getProvider() == "own")
    {
      return stream;
    }
  }

  return returnPtr;
}

int InformationType::registerStreamTemplate(std::shared_ptr<InformationStreamTemplate> streamTemplate)
{
  for (auto st : this->templates)
  {
    if (st->getName() == streamTemplate->getName())
      return 1;
  }

  int returnVel = this->registerStreamTemplateInStore(streamTemplate);

  if (returnVel == 1)
  {
    std::cout << "InformationStore: Duplicated Stream with name '" << streamTemplate->getName() << "'" << std::endl;
  }

  this->templates.push_back(streamTemplate);

  return 0;
}

std::shared_ptr<InformationStreamTemplate> InformationType::getStreamTemplate(const std::string streamTemplateName)
{
  for (auto st : this->templates)
  {
    if (st->getName() == streamTemplateName)
      return st;
  }

  std::shared_ptr<InformationStreamTemplate> ptr;
  return ptr;
}

bool InformationType::existsStreamTemplate(const std::shared_ptr<StreamDescription> streamDescription)
{
  return this->templates.size() > 0;
}

bool InformationType::existsStreamTemplate(const std::shared_ptr<StreamTemplateDescription> streamTemplateDescription)
{
  return this->templates.size() > 0;
}

std::shared_ptr<BaseInformationStream> InformationType::createStreamFromTemplate(
    const std::shared_ptr<StreamDescription> streamDescription, std::string provider)
{
  if (this->templates.size() > 0)
  {
    // Check if exists
    std::string name = this->templates.at(0)->getName();
    name.replace(name.find("?provider"), 9, provider);
    auto existing = this->informationStore.lock()->getBaseStream(name);

    if (existing)
      return existing;

    auto stream = this->templates.at(0)->createBaseStream(provider);

    int returnVel = this->registerStreamInStore(stream);

    if (returnVel == 1)
    {
      std::cout << "InformationStore: Duplicated Stream with name '" << stream->getName() << "'" << std::endl;
    }

    this->streams.push_back(stream);

    return stream;
  }

  std::shared_ptr<BaseInformationStream> ptr;
  return ptr;
}

std::shared_ptr<BaseInformationStream> InformationType::createStreamFromTemplate(
    const std::shared_ptr<StreamTemplateDescription> streamTemplateDescription, std::string provider)
{
  if (this->templates.size() > 0)
  {
    auto stream = this->templates.at(0)->createBaseStream(provider);

    int returnVel = this->registerStreamInStore(stream);

    if (returnVel == 1)
    {
      std::cout << "InformationStore: Duplicated Stream with name '" << stream->getName() << "'" << std::endl;
    }

    this->streams.push_back(stream);

    return stream;
  }

  std::shared_ptr<BaseInformationStream> ptr;
  return ptr;
}

int InformationType::registerStreamInStore(std::shared_ptr<BaseInformationStream> stream)
{
  std::shared_ptr<InformationStore> store = this->informationStore.lock();
  return store->addStream(stream);
}

int InformationType::registerStreamTemplateInStore(std::shared_ptr<InformationStreamTemplate> streamTemplate)
{
  std::shared_ptr<InformationStore> store = this->informationStore.lock();
  return store->addStreamTemplate(streamTemplate);
}

} /* namespace ice */
